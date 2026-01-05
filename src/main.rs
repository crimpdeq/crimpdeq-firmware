#![no_std]
#![no_main]

use core::cell::RefCell;

use bt_hci::controller::ExternalController;
use critical_section::Mutex;
use defmt::{debug, error, info, warn};
use embassy_executor::Spawner;
use embassy_futures::{join::join, select::select};
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use esp_hal::{
    analog::adc::{Adc, AdcCalCurve, AdcConfig, AdcPin, Attenuation},
    clock::CpuClock,
    delay::Delay,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    interrupt::software::SoftwareInterruptControl,
    peripherals,
    rtc_cntl::Rtc,
    time,
    timer::timg::TimerGroup,
    Async,
    Config,
};
use esp_radio::ble::controller::BleConnector;
use esp_storage::FlashStorage;
use panic_rtt_target as _;
use static_cell::StaticCell;
use trouble_host::prelude::*;

extern crate alloc;

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
        let x = STATIC_CELL.uninit().write($val);
        x
    }};
}

/// Static tracking the state of the device
static DEVICE_STATE: Mutex<RefCell<DeviceState>> = Mutex::new(RefCell::new(DeviceState {
    measurement_status: MeasurementTaskStatus::Disabled,
    tared: false,
    start_time: 0,
    calibration_points: [None, None],
    battery_voltage: 4300,
    ble_disconnection_time: None,
}));

// ESP-IDF App Descriptor
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // Initialize RTT for defmt logging
    rtt_target::rtt_init_defmt!();

    // System initialization
    let config = Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Allocate 72KB of heap memory
    esp_alloc::heap_allocator!(size: 72 * 1024);

    // Initialize RTOS
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    // Initialize radio
    static RADIO: StaticCell<esp_radio::Controller<'static>> = StaticCell::new();
    let radio = RADIO.init(esp_radio::init().unwrap());

    // Initialize BLE
    let bluetooth = peripherals.BT;
    let connector = BleConnector::new(radio, bluetooth, Default::default()).unwrap();
    let controller: ExternalController<_, 1> = ExternalController::new(connector);

    // Initialize load cell pins
    let clock_pin = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());
    let data_pin = Input::new(
        peripherals.GPIO4,
        InputConfig::default().with_pull(Pull::None),
    );
    let delay = Delay::new();

    // Initialize Flash Storage
    let flash = FlashStorage::new(peripherals.FLASH);

    // Initialize RTC
    let rtc = Rtc::new(peripherals.LPWR);

    // Initialize battery voltage reading
    let mut adc_config = AdcConfig::new();
    let analog_pin = peripherals.GPIO1;
    let battery_pin =
        adc_config.enable_pin_with_cal::<_, AdcCalCurve<_>>(analog_pin, Attenuation::_11dB);
    let battery_adc = Adc::new(peripherals.ADC1, adc_config).into_async();

    // Use the last 6 bytes of the DEVICE_NAME for the address
    let device_name = env!("DEVICE_NAME");
    let name_bytes = device_name.as_bytes();
    let mut address_seed = [0u8; 6];
    let seed_len = address_seed.len();
    if name_bytes.len() >= seed_len {
        address_seed.copy_from_slice(&name_bytes[name_bytes.len() - seed_len..]);
    } else {
        address_seed[..name_bytes.len()].copy_from_slice(name_bytes);
    }
    address_seed[5] |= 0xC0;
    let address: Address = Address::random(address_seed);
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

    // Start idle timer: if no BLE connection happens within TIMEOUT_MS, deep_sleep_task will sleep.
    critical_section::with(|cs| {
        DEVICE_STATE.borrow_ref_mut(cs).on_ble_disconnected();
    });

    // Spawn tasks
    spawner
        .spawn(measurement_task(channel, clock_pin, data_pin, delay, flash))
        .unwrap();
    spawner
        .spawn(battery_voltage_task(battery_adc, battery_pin))
        .unwrap();
    spawner.spawn(deep_sleep_task(rtc)).unwrap();

    let _ = join(ble_task(runner), async {
        loop {
            match advertise(device_name, &mut peripheral, &server).await {
                Ok(conn) => {
                    info!("BLE connection established");
                    critical_section::with(|cs| {
                        DEVICE_STATE.borrow_ref_mut(cs).on_ble_connected();
                    });
                    // run until any task ends (usually because the connection has been closed),
                    // then return to advertising state.
                    select(
                        gatt_events_task(&server, &conn, channel),
                        data_processing_task(&server, &conn, channel),
                    )
                    .await;
                    critical_section::with(|cs| {
                        DEVICE_STATE.borrow_ref_mut(cs).stop_measurement();
                    });
                    critical_section::with(|cs| {
                        let mut state = DEVICE_STATE.borrow_ref_mut(cs);
                        state.on_ble_disconnected();
                        debug!(
                            "BLE connection closed, disconnection time: {:?}",
                            state.ble_disconnection_time
                        );
                    });
                }
                Err(e) => {
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
            panic!("BLE error: {:?}", e);
        }
    }
}

#[embassy_executor::task]
async fn deep_sleep_task(mut rtc: Rtc<'static>) {
    const TIMEOUT_MS: u32 = 5 * 60 * 1000; // 5 minutes

    loop {
        let elapsed_ms = critical_section::with(|cs| {
            DEVICE_STATE
                .borrow_ref(cs)
                .get_ble_disconnection_elapsed_ms()
        });

        if let Some(elapsed) = elapsed_ms {
            debug!("BLE disconnected for {:?} ms", elapsed);

            if elapsed >= TIMEOUT_MS {
                info!(
                    "Entering deep sleep after {} minutes of BLE disconnection",
                    TIMEOUT_MS / 60000
                );
                Timer::after(Duration::from_millis(10)).await;
                rtc.sleep_deep(&[]);
            }
        }
        Timer::after(Duration::from_secs(10)).await;
    }
}

#[embassy_executor::task]
async fn battery_voltage_task(
    mut adc: Adc<'static, peripherals::ADC1<'static>, Async>,
    mut pin: AdcPin<
        peripherals::GPIO1<'static>,
        peripherals::ADC1<'static>,
        AdcCalCurve<peripherals::ADC1<'static>>,
    >,
) {
    loop {
        // Read the battery voltage 20 times and average the results
        let mut adc_voltage_mv: u32 = 0;
        for _ in 0..20 {
            adc_voltage_mv += adc.read_oneshot(&mut pin).await as u32;
            Timer::after(Duration::from_millis(10)).await;
        }
        let adc_voltage_mv = (adc_voltage_mv / 20) as u16;
        debug!("ADC voltage: {:?}", adc_voltage_mv);

        // Calculate battery voltage using voltage divider formula
        // Voltage divider: R1=33k, R2=10k
        // Formula: V_battery = V_adc * (R1 + R2) / R2
        let battery_voltage_mv = (adc_voltage_mv as u32 * 43) / 10;
        info!("Battery voltage: {:?}", battery_voltage_mv);

        // Update device state
        critical_section::with(|cs| {
            let mut state = DEVICE_STATE.borrow_ref_mut(cs);
            state.battery_voltage = battery_voltage_mv;
        });
        Timer::after(Duration::from_secs(45)).await;
    }
}

#[embassy_executor::task]
async fn measurement_task(
    channel: &'static DataPointChannel,
    clock_pin: Output<'static>,
    data_pin: Input<'static>,
    delay: Delay,
    flash: FlashStorage<'static>,
) {
    let mut load_cell = Hx711::new(data_pin, clock_pin, delay, flash);
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
                let calibration_point = load_cell.perform_calibration().await;

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
            MeasurementTaskStatus::GetCalibration => {
                load_cell.get_calibration_factor().unwrap();
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
    let now = (time::Instant::now().duration_since_epoch()).as_micros() as u32;
    let timestamp = now.wrapping_sub(start_time);

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
                // Handle write events to the control point
                if let GattEvent::Write(write_event) = &event {
                    if write_event.handle() == control_point.handle {
                        let cmd_data = write_event.data();
                        let Some(&op_code_byte) = cmd_data.first() else {
                            warn!("Control Point write with empty payload");
                            continue;
                        };
                        let op_code = ControlOpCode::from(op_code_byte);
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
