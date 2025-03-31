#![no_std]
#![no_main]

use core::cell::RefCell;

use arrayvec::ArrayVec;
use bt_hci::controller::ExternalController;
use bytemuck::bytes_of;
use critical_section::Mutex;
use defmt::{debug, error, info, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::{join::join, select::select};
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    rng::Rng,
    time,
    timer::{systimer::SystemTimer, timg::TimerGroup},
    Config,
};
use esp_println as _;
use esp_wifi::{ble::controller::BleConnector, init, EspWifiController};
// use esp_backtrace as _;
use panic_rtt_target as _;
use trouble_host::prelude::*;

use crate::{
    hx711::Hx711,
    progressor::{
        ControlOpCode,
        DataPoint,
        DataPointChannel,
        DeviceState,
        MeasurementTaskStatus,
        ResponseCode,
        CONNECTIONS_MAX,
        L2CAP_CHANNELS_MAX,
        L2CAP_MTU,
        SCAN_RESPONSE_DATA,
    },
};

// GATT Server definition
#[gatt_server]
pub struct Server {
    progressor: ProgressorService,
}

/// Tindeq Progressor service
#[gatt_service(uuid = "7e4e1701-1ea6-40c9-9dcc-13d34ffead57")]
struct ProgressorService {
    /// Data Point - for receiving data from the Progressor
    #[characteristic(uuid = "7e4e1702-1ea6-40c9-9dcc-13d34ffead57", notify)]
    pub data_point: [u8; 14], // Buffer for received data

    /// Control Point - for sending commands to the Progressor
    #[characteristic(uuid = "7e4e1703-1ea6-40c9-9dcc-13d34ffead57", write)]
    pub control_point: [u8; 14], // Buffer for command data
}

pub mod hx711;
pub mod progressor;

// Helper macro for static allocation
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

/// Static tracking the state of the device
static DEVICE_STATE: Mutex<RefCell<DeviceState>> = Mutex::new(RefCell::new(DeviceState {
    measurement_status: MeasurementTaskStatus::Disabled,
    tared: false,
    start_time: 0,
    calibration_points: [-1.0, -1.0],
}));

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    // System initialization
    let config = Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Allocate 72KB of heap memory
    esp_alloc::heap_allocator!(size: 72 * 1024);

    debug!("{}", Hx711::get_calibration());

    // Initialize BLE controller
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let esp_wifi_ctrl = &*mk_static!(
        EspWifiController<'static>,
        init(
            timg0.timer0,
            Rng::new(peripherals.RNG),
            peripherals.RADIO_CLK,
        )
        .unwrap()
    );

    // Initialize load cell pins
    let clock_pin = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());
    let data_pin = Input::new(
        peripherals.GPIO4,
        InputConfig::default().with_pull(Pull::None),
    );
    let delay = Delay::new();

    // Initialize embassy
    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);

    // Initialize BLE
    let bluetooth = peripherals.BT;
    let connector = BleConnector::new(esp_wifi_ctrl, bluetooth);
    let controller: ExternalController<_, 20> = ExternalController::new(connector);
    let address: Address = Address::random([0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a]);
    // info!("Our address = {}", address);
    let mut resources: HostResources<CONNECTIONS_MAX, L2CAP_CHANNELS_MAX, L2CAP_MTU> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        runner,
        ..
    } = stack.build();

    info!("Starting advertising and GATT service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "Progressor",
        appearance: &appearance::UNKNOWN,
    }))
    .unwrap();

    // Data point channel for communication between tasks
    let channel = mk_static!(DataPointChannel, Channel::new());

    // Spawn tasks
    spawner
        .spawn(measurement_task(channel, clock_pin, data_pin, delay))
        .unwrap();

    let _ = join(ble_task(runner), async {
        loop {
            match advertise(&mut peripheral, &server).await {
                Ok(conn) => {
                    // run until any task ends (usually because the connection has been closed),
                    // then return to advertising state.
                    select(
                        gatt_events_task(&server, &conn, channel),
                        data_processing_task(&server, &conn, channel),
                    )
                    .await;
                }
                Err(e) => {
                    let e = defmt::Debug2Format(&e);
                    panic!("BLE error: {:?}", e);
                }
            }
        }
    })
    .await;

    // Idle loop
    loop {
        Timer::after(Duration::from_millis(50)).await;
    }
}

async fn ble_task<C: Controller>(mut runner: Runner<'_, C>) {
    loop {
        if let Err(e) = runner.run().await {
            let e = defmt::Debug2Format(&e);
            panic!("BLE error: {:?}", e);
        }
    }
}

#[embassy_executor::task]
async fn measurement_task(
    channel: &'static DataPointChannel,
    clock_pin: Output<'static>,
    data_pin: Input<'static>,
    delay: Delay,
) {
    let mut load_cell = Hx711::new(data_pin, clock_pin, delay);

    loop {
        // Get current device state
        let (status, start_time) = critical_section::with(|cs| {
            let state = DEVICE_STATE.borrow_ref(cs);
            (state.measurement_status, state.start_time)
        });

        match status {
            MeasurementTaskStatus::Disabled => {
                // Do nothing when disabled
                Timer::after(Duration::from_millis(10)).await;
            }
            MeasurementTaskStatus::Tare | MeasurementTaskStatus::SoftTare => {
                // Perform taring operation
                load_cell.tare().await;

                critical_section::with(|cs| {
                    let mut state = DEVICE_STATE.borrow_ref_mut(cs);
                    state.tared = true;

                    // For soft tare, immediately enable measurements
                    if status == MeasurementTaskStatus::SoftTare {
                        state.measurement_status = MeasurementTaskStatus::Enabled;
                    }
                });

                Timer::after(Duration::from_millis(10)).await;
            }
            MeasurementTaskStatus::Enabled => {
                send_weight_measurement(&mut load_cell, start_time, channel).await;
            }
            MeasurementTaskStatus::Calibration(weight) => {
                // Use the load cell's own calibration method to collect a calibration point
                let calibration_point = load_cell.perform_calibration(weight).await;

                critical_section::with(|cs| {
                    let mut state = DEVICE_STATE.borrow_ref_mut(cs);

                    // Store calibration point (either first or second)
                    if state.calibration_points[0] == -1.0 {
                        state.calibration_points[0] = calibration_point;
                    } else {
                        state.calibration_points[1] = calibration_point;
                    }

                    // Disable measurement mode after capturing point
                    state.measurement_status = MeasurementTaskStatus::Disabled;

                    // Calculate and apply calibration if we have both points
                    if state.calibration_points[0] != -1.0 && state.calibration_points[1] != -1.0 {
                        let success =
                            load_cell.apply_two_point_calibration(state.calibration_points, weight);
                        if !success {
                            error!(
                                "Failed to apply calibration points: {:?}",
                                state.calibration_points
                            );
                        }
                    }
                });
            }
            MeasurementTaskStatus::DefaultCalibration => {
                // Reset calibration to default values
                if let Err(e) = load_cell.default_calibration() {
                    error!(
                        "Error applying default calibration: {:?}",
                        defmt::Debug2Format(&e)
                    );
                }
                critical_section::with(|cs| {
                    let mut state = DEVICE_STATE.borrow_ref_mut(cs);
                    state.measurement_status = MeasurementTaskStatus::Disabled;
                });
            }
        }
    }
}

/// Send a weight measurement data point with current timestamp
async fn send_weight_measurement(
    load_cell: &mut Hx711<'_>,
    start_time: u32,
    channel: &'static DataPointChannel,
) {
    let weight = load_cell.read_calibrated().await;
    let timestamp = (time::Instant::now().duration_since_epoch()).as_micros() as u32 - start_time;

    debug!(
        "Sending measurement: Weight: {}kg, Timestamp: {:?}",
        weight,
        timestamp as f32 / 1000000.0
    );

    let response = ResponseCode::WeightMeasurement(weight, timestamp);
    let data_point = DataPoint::from(response);
    data_point.send(channel);
}

/// Stream Events until the connection closes.
///
/// This function will handle the GATT events and process them.
/// This is how we interact with read and write requests.
async fn gatt_events_task(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_>,
    channel: &'static DataPointChannel,
) -> Result<(), Error> {
    let control_point = server.progressor.control_point;
    loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => {
                info!("Device disconnected: {:?}", reason);
                break;
            }
            GattConnectionEvent::Gatt { event } => {
                match event {
                    Ok(event) => {
                        if let GattEvent::Write(write_event) = &event {
                            if write_event.handle() == control_point.handle {
                                // Process control point command
                                let cmd_data = write_event.data();
                                let cmd_type = cmd_data[0]; // Command type
                                let op_code = ControlOpCode::from(cmd_type);
                                info!("Control Point Received: {:?}", op_code);

                                critical_section::with(|cs| {
                                    let mut device_state = DEVICE_STATE.borrow_ref_mut(cs);
                                    op_code.process(cmd_data, channel, &mut device_state);
                                });
                            }
                        }

                        // This step is also performed at drop(), but writing it explicitly is necessary
                        // in order to ensure reply is sent.
                        match event.accept() {
                            Ok(reply) => {
                                reply.send().await;
                            }
                            Err(_e) => warn!("Error sending response"),
                        }
                    }
                    Err(_e) => warn!("Error processing event"),
                }
            }
            _ => {}
        }
    }
    info!("BLE task finished");
    Ok(())
}

/// Process data and send notifications to the client
async fn data_processing_task(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_>,
    channel: &'static DataPointChannel,
) {
    let data_point_handle = server.progressor.data_point;

    loop {
        let data_point = channel.receive().await;
        debug!("Sending Data Point: {:?}", data_point);

        // Create a properly sized array for notification
        let mut notification_data = [0u8; 14];

        // Use bytes_of to get the raw bytes from the DataPoint struct
        let data_bytes = bytes_of(&data_point);

        // Copy the data to a properly sized array
        notification_data[..data_bytes.len().min(14)]
            .copy_from_slice(&data_bytes[..data_bytes.len().min(14)]);

        // Send notification with the data packet
        if let Err(e) = data_point_handle.notify(conn, &notification_data).await {
            info!("Error sending Data Point: {:?}", defmt::Debug2Format(&e));
            break;
        }
    }
}

/// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
pub async fn advertise<'a, 'b, C: Controller>(
    peripheral: &mut Peripheral<'a, C>,
    server: &'b Server<'_>,
) -> Result<GattConnection<'a, 'b>, BleHostError<C::Error>> {
    let advertising_data = advertising_data(b"Progressor_7125").expect("Valid advertising data");

    debug!("Advertising BLE");
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: advertising_data.as_slice(),
                scan_data: SCAN_RESPONSE_DATA,
            },
        )
        .await?;
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    info!("BLE connection established");
    Ok(conn)
}

fn advertising_data(name: &[u8]) -> Result<ArrayVec<u8, 27>, ()> {
    // BLE AD type and flag constants
    const AD_TYPE_FLAGS: u8 = 0x01;
    const AD_TYPE_COMPLETE_LOCAL_NAME: u8 = 0x09;
    const FLAG_LE_GENERAL_DISC_MODE: u8 = 0x02;
    const FLAG_BR_EDR_NOT_SUPPORTED: u8 = 0x04;

    let mut advertising_data: ArrayVec<u8, 27> = ArrayVec::new();

    // Add flags
    advertising_data.push(2); // Length of flag field (1 byte for type + 1 byte for value)
    advertising_data.push(AD_TYPE_FLAGS);
    advertising_data.push(FLAG_LE_GENERAL_DISC_MODE | FLAG_BR_EDR_NOT_SUPPORTED);

    // Add name (1 byte for type + name bytes)
    let name_len = name.len();
    if name_len > 24 {
        // Maximum allowed size (27 - 3 bytes used for flags)
        return Err(());
    }

    advertising_data.push(name_len as u8 + 1);
    advertising_data.push(AD_TYPE_COMPLETE_LOCAL_NAME);
    advertising_data
        .try_extend_from_slice(name)
        .map_err(|_| ())?;

    Ok(advertising_data)
}
