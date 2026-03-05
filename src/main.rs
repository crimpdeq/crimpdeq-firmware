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
    Async,
    Config,
    analog::adc::{Adc, AdcCalCurve, AdcConfig, AdcPin, Attenuation},
    clock::CpuClock,
    delay::Delay,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    interrupt::software::SoftwareInterruptControl,
    peripherals,
    rtc_cntl::Rtc,
    time,
    timer::timg::TimerGroup,
};
use esp_radio::ble::controller::BleConnector;
use esp_storage::FlashStorage;
use panic_rtt_target as _;
use static_cell::StaticCell;
use trouble_host::prelude::*;

use crate::{
    ble::{CONNECTIONS_MAX, L2CAP_CHANNELS_MAX, L2CAP_MTU, Server, advertise},
    hx711::Hx711,
    progressor::{
        CalibrationPoint,
        ControlOpCode,
        ControlResponses,
        DEVICE_ID_SIZE,
        DataPoint,
        DataPointChannel,
        DeviceState,
        MAX_CALIBRATION_POINTS,
        MeasurementTaskStatus,
        ResponseCode,
        WEIGHT_SAMPLES_PER_PACKET,
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
static DEVICE_STATE: Mutex<RefCell<DeviceState>> = Mutex::new(RefCell::new(DeviceState::new()));

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

    let device_name = env!("DEVICE_NAME");
    let device_id = parse_device_id_hex(env!("DEVICE_ID")).unwrap_or_else(|| {
        panic!(
            "Invalid DEVICE_ID '{}', expected exactly {} hex chars",
            env!("DEVICE_ID"),
            DEVICE_ID_SIZE * 2
        )
    });

    // Derive BLE random static address from DEVICE_ID to avoid collisions
    // when multiple devices share the same advertised name.
    let mut address_seed = device_id;
    // Set random static address bits (two MSBs must be 1)
    address_seed[5] = (address_seed[5] & 0x3F) | 0xC0;
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
                        gatt_events_task(&server, &conn, channel, device_id),
                        data_processing_task(&server, &conn, channel),
                    )
                    .await;
                    critical_section::with(|cs| {
                        let mut state = DEVICE_STATE.borrow_ref_mut(cs);
                        state.stop_measurement();
                        state.on_ble_disconnected();
                        debug!(
                            "BLE connection closed, disconnection time: {:?}",
                            state.ble_disconnection_time
                        );
                    });
                }
                Err(e) => {
                    error!("BLE advertise error: {:?}", e);
                    Timer::after(Duration::from_millis(250)).await;
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
            error!("BLE runner error: {:?}", defmt::Debug2Format(&e));
            Timer::after(Duration::from_millis(250)).await;
        }
    }
}

fn hex_nibble(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        b'A'..=b'F' => Some(byte - b'A' + 10),
        _ => None,
    }
}

fn parse_device_id_hex(device_id: &str) -> Option<[u8; DEVICE_ID_SIZE]> {
    const HEX_CHARS_PER_BYTE: usize = 2;

    let input = device_id.as_bytes();
    if input.len() != DEVICE_ID_SIZE * HEX_CHARS_PER_BYTE {
        return None;
    }

    let mut parsed = [0u8; DEVICE_ID_SIZE];
    let mut i = 0;
    while i < DEVICE_ID_SIZE {
        let hi = hex_nibble(input[i * HEX_CHARS_PER_BYTE])?;
        let lo = hex_nibble(input[i * HEX_CHARS_PER_BYTE + 1])?;
        parsed[i] = (hi << 4) | lo;
        i += 1;
    }

    Some(parsed)
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
    if let Err(e) = load_cell.tare().await {
        error!("Initial tare failed: {:?}", defmt::Debug2Format(&e));
    }
    let mut measurement_batch = [(0.0_f32, 0_u32); WEIGHT_SAMPLES_PER_PACKET];
    let mut measurement_batch_len = 0usize;

    loop {
        // Get current device state
        let (status, start_time) = critical_section::with(|cs| {
            let state = DEVICE_STATE.borrow_ref(cs);
            (state.measurement_status, state.start_time)
        });

        if status != MeasurementTaskStatus::Enabled && measurement_batch_len > 0 {
            send_weight_measurements(channel, &measurement_batch[..measurement_batch_len]).await;
            measurement_batch_len = 0;
        }

        match status {
            MeasurementTaskStatus::Disabled => {
                // Do nothing when disabled
            }
            MeasurementTaskStatus::Tare => {
                // Perform taring operation
                if let Err(e) = load_cell.tare().await {
                    error!("Tare operation failed: {:?}", defmt::Debug2Format(&e));
                }

                critical_section::with(|cs| {
                    let mut state = DEVICE_STATE.borrow_ref_mut(cs);
                    state.measurement_status = MeasurementTaskStatus::Disabled;
                });
            }
            MeasurementTaskStatus::Enabled => {
                match sample_weight_measurement(&mut load_cell, start_time).await {
                    Ok(sample) => {
                        measurement_batch[measurement_batch_len] = sample;
                        measurement_batch_len += 1;

                        if measurement_batch_len == WEIGHT_SAMPLES_PER_PACKET {
                            send_weight_measurements(channel, &measurement_batch).await;
                            measurement_batch_len = 0;
                        }
                    }
                    Err(e) => {
                        error!(
                            "Skipping weight sample after HX711 read error: {:?}",
                            defmt::Debug2Format(&e)
                        );
                        Timer::after(Duration::from_millis(5)).await;
                    }
                }
            }
            MeasurementTaskStatus::Calibration(weight) => {
                if !weight.is_finite() || weight < 0.0 {
                    error!("Ignoring invalid calibration weight: {}", weight);
                    critical_section::with(|cs| {
                        DEVICE_STATE.borrow_ref_mut(cs).measurement_status =
                            MeasurementTaskStatus::Disabled;
                    });
                    continue;
                }

                // Use the load cell's own calibration method to collect a calibration point
                let calibration_point = match load_cell.perform_calibration().await {
                    Ok(point) => point,
                    Err(e) => {
                        error!("Calibration sampling failed: {:?}", defmt::Debug2Format(&e));
                        critical_section::with(|cs| {
                            DEVICE_STATE.borrow_ref_mut(cs).measurement_status =
                                MeasurementTaskStatus::Disabled;
                        });
                        continue;
                    }
                };
                if !calibration_point.is_finite() {
                    error!(
                        "Ignoring invalid calibration raw point: {}",
                        calibration_point
                    );
                    critical_section::with(|cs| {
                        DEVICE_STATE.borrow_ref_mut(cs).measurement_status =
                            MeasurementTaskStatus::Disabled;
                    });
                    continue;
                }

                let (calibration_points, calibration_point_count) = critical_section::with(|cs| {
                    let mut state = DEVICE_STATE.borrow_ref_mut(cs);
                    let new_point: CalibrationPoint = (calibration_point, weight);
                    if state.calibration_point_count < MAX_CALIBRATION_POINTS {
                        let index = state.calibration_point_count;
                        state.calibration_points[index] = new_point;
                        state.calibration_point_count += 1;
                    } else {
                        warn!(
                            "Calibration point buffer full (max {}), ignoring new point",
                            MAX_CALIBRATION_POINTS
                        );
                    }

                    // Disable measurement mode after capturing point
                    state.measurement_status = MeasurementTaskStatus::Disabled;
                    (state.calibration_points, state.calibration_point_count)
                });

                if calibration_point_count >= 2 {
                    let points = &calibration_points[..calibration_point_count];
                    if !load_cell.apply_multi_point_calibration(points) {
                        error!("Failed to apply calibration points: {:?}", points);
                    } else {
                        notify_calibration_factor(channel, load_cell.current_calibration_factor())
                            .await;
                        notify_calibration_points(channel, points).await;
                    }
                } else {
                    info!("Calibration needs at least two points before applying.");
                }
            }
            MeasurementTaskStatus::DefaultCalibration => {
                // Reset calibration to default values
                if let Err(e) = load_cell.default_calibration_factor() {
                    error!(
                        "Error applying default calibration: {:?}",
                        defmt::Debug2Format(&e)
                    );
                } else {
                    notify_calibration_factor(channel, load_cell.current_calibration_factor())
                        .await;
                }
                critical_section::with(|cs| {
                    let mut state = DEVICE_STATE.borrow_ref_mut(cs);
                    state.calibration_point_count = 0;
                    state.measurement_status = MeasurementTaskStatus::Disabled;
                });
            }
            MeasurementTaskStatus::GetCalibration => {
                match load_cell.get_calibration_factor() {
                    Ok(factor) => {
                        channel
                            .send(DataPoint::from(ResponseCode::CalibrationFactor(factor)))
                            .await;
                    }
                    Err(e) => {
                        error!(
                            "Failed to read calibration factor: {:?}",
                            defmt::Debug2Format(&e)
                        );
                    }
                }
                let (calibration_points, calibration_point_count) = critical_section::with(|cs| {
                    let mut state = DEVICE_STATE.borrow_ref_mut(cs);
                    let calibration_points = state.calibration_points;
                    let calibration_point_count = state.calibration_point_count;
                    state.measurement_status = MeasurementTaskStatus::Disabled;
                    (calibration_points, calibration_point_count)
                });
                notify_calibration_points(channel, &calibration_points[..calibration_point_count])
                    .await;
                if calibration_point_count > 0 {
                    info!(
                        "Calibration points: {:?}",
                        &calibration_points[..calibration_point_count]
                    );
                } else {
                    info!("Calibration points empty (possibly lost after device reset)");
                }
            }
        }

        // Add a short delay to prevent tight loops
        if status == MeasurementTaskStatus::Disabled {
            Timer::after(Duration::from_millis(10)).await;
        }
    }
}

/// Collect one weight measurement sample with current timestamp.
async fn sample_weight_measurement(
    load_cell: &mut Hx711<'_>,
    start_time: u32,
) -> Result<(f32, u32), hx711::Hx711Error> {
    let weight = load_cell.read_calibrated().await?;
    let now = (time::Instant::now().duration_since_epoch()).as_micros() as u32;
    let timestamp = now.wrapping_sub(start_time);

    Ok((weight, timestamp))
}

/// Send a batch of weight measurement samples as one notification.
async fn send_weight_measurements(channel: &'static DataPointChannel, measurements: &[(f32, u32)]) {
    debug!("Sending {} batched measurements", measurements.len());
    channel
        .send(DataPoint::weight_measurements(measurements))
        .await;
}

async fn notify_calibration_points(
    channel: &'static DataPointChannel,
    calibration_points: &[CalibrationPoint],
) {
    for (raw_value, weight) in calibration_points {
        debug!("Notifying calibration point: {:?}", (raw_value, weight));
        channel
            .send(DataPoint::from(ResponseCode::CalibrationPoint(
                *raw_value, *weight,
            )))
            .await;
    }
}

async fn notify_calibration_factor(channel: &'static DataPointChannel, calibration_factor: f32) {
    debug!("Notifying calibration factor: {:?}", calibration_factor);
    channel
        .send(DataPoint::from(ResponseCode::CalibrationFactor(
            calibration_factor,
        )))
        .await;
}

/// Stream Events until the connection closes.
///
/// This function will handle the GATT events and process them.
/// This is how we interact with read and write requests.
async fn gatt_events_task<P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
    channel: &'static DataPointChannel,
    device_id: [u8; DEVICE_ID_SIZE],
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
                if let GattEvent::Write(write_event) = &event
                    && write_event.handle() == control_point.handle
                {
                    let cmd_data = write_event.data();
                    let Some(&op_code_byte) = cmd_data.first() else {
                        warn!("Control Point write with empty payload");
                        continue;
                    };
                    let op_code = match ControlOpCode::try_from(op_code_byte) {
                        Ok(op_code) => op_code,
                        Err(()) => {
                            warn!("Invalid OpCode received: {:#x}", op_code_byte);
                            continue;
                        }
                    };
                    info!("Control Point Received: {:?}", op_code);

                    let mut responses = ControlResponses::new();
                    critical_section::with(|cs| {
                        let mut device_state = DEVICE_STATE.borrow_ref_mut(cs);
                        op_code.process(cmd_data, &mut device_state, device_id, &mut responses);
                    });
                    for response in responses {
                        channel.send(response).await;
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
