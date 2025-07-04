#![no_std]
#![no_main]

use core::cell::RefCell;

use bt_hci::controller::ExternalController;
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
use panic_rtt_target as _;
use trouble_host::prelude::*;

use crate::{
    ble::{advertise, Server, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX, L2CAP_MTU},
    hx711::Hx711,
    progressor::{
        ControlOpCode,
        DataPoint,
        DataPointChannel,
        DeviceState,
        MeasurementTaskStatus,
        ResponseCode,
    },
};

pub mod ble;
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
    calibration_points: [None, None],
}));

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    // System initialization
    let config = Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Allocate 72KB of heap memory
    esp_alloc::heap_allocator!(size: 72 * 1024);

    debug!("{}", Hx711::get_calibration_factor().unwrap());

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
    // Use the last 6 bytes of the DEVIC_NAME for the address
    let device_name = env!("DEVICE_NAME");
    let mut buff: [u8; 6] = [0u8; 6];
    buff.copy_from_slice(&device_name.as_bytes()[device_name.len() - 6..]);
    buff[5] |= 0xC0;
    let address: Address = Address::random(buff);
    let mut resources: HostResources<
        DefaultPacketPool,
        CONNECTIONS_MAX,
        L2CAP_CHANNELS_MAX,
        L2CAP_MTU,
    > = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        runner,
        ..
    } = stack.build();

    info!("Starting advertising and GATT service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: device_name,
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
            match advertise(device_name, &mut peripheral, &server).await {
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

async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
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
    load_cell.tare().await;

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
            MeasurementTaskStatus::Tare => {
                // Perform taring operation
                load_cell.tare().await;

                critical_section::with(|cs| {
                    let mut state = DEVICE_STATE.borrow_ref_mut(cs);
                    state.tared = true;
                    state.measurement_status = MeasurementTaskStatus::Disabled;
                });
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
                    if state.calibration_points[0].is_none() {
                        state.calibration_points[0] = Some(calibration_point);
                    } else {
                        state.calibration_points[1] = Some(calibration_point);

                        // Calculate and apply calibration if we have both points
                        if let (Some(point1), Some(point2)) =
                            (state.calibration_points[0], state.calibration_points[1])
                        {
                            if !load_cell.apply_two_point_calibration([point1, point2], weight) {
                                error!(
                                    "Failed to apply calibration points: {:?}",
                                    state.calibration_points
                                );
                            }
                        }
                    }

                    // Disable measurement mode after capturing point
                    state.measurement_status = MeasurementTaskStatus::Disabled;
                });
            }
            MeasurementTaskStatus::DefaultCalibration => {
                // Reset calibration to default values
                if let Err(e) = load_cell.default_calibration_factor() {
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

        // Add a short delay to prevent tight loops
        if status == MeasurementTaskStatus::Disabled {
            Timer::after(Duration::from_millis(10)).await;
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
async fn gatt_events_task<P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
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
                if let Ok(event) = event {
                    // Handle write events to the control point
                    if let GattEvent::Write(write_event) = &event {
                        if write_event.handle() == control_point.handle {
                            let cmd_data = write_event.data();
                            let op_code = ControlOpCode::from(cmd_data[0]);
                            info!("Control Point Received: {:?}", op_code);

                            critical_section::with(|cs| {
                                let mut device_state = DEVICE_STATE.borrow_ref_mut(cs);
                                op_code.process(cmd_data, channel, &mut device_state);
                            });
                        }
                    }

                    // Ensure reply is sent
                    if let Ok(reply) = event.accept() {
                        reply.send().await;
                    } else {
                        warn!("Error sending response");
                    }
                } else {
                    warn!("Error processing event");
                }
            }
            _ => {}
        }
    }

    info!("BLE task finished");
    critical_section::with(|cs| {
        let mut device_state = DEVICE_STATE.borrow_ref_mut(cs);
        device_state.stop_measurement();
    });

    Ok(())
}

/// Process data and send notifications to the client
async fn data_processing_task<P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
    channel: &'static DataPointChannel,
) {
    let data_point_handle = server.progressor.data_point;

    loop {
        let data_point = channel.receive().await;
        debug!("Sending Data Point: {:?}", data_point);

        // Send notification with the data packet
        if let Err(e) = data_point_handle.notify(conn, &data_point).await {
            info!("Error sending Data Point: {:?}", defmt::Debug2Format(&e));
            break;
        }
    }
}
