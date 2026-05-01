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
    timer::timg::TimerGroup,
};
use esp_radio::ble::controller::BleConnector;
use esp_storage::FlashStorage;
use panic_rtt_target as _;
use trouble_host::prelude::*;

use crate::{
    ble::{CONNECTIONS_MAX, L2CAP_CHANNELS_MAX, L2CAP_MTU, Server, advertise},
    hx711::Hx711,
    progressor::{
        CalibrationPoint,
        ControlOpCode,
        DataPoint,
        DataPointChannel,
        DeviceState,
        MAX_CALIBRATION_POINTS,
        MeasurementTaskStatus,
        ResponseCode,
        SleepReason,
        SleepState,
        WeightMeasurementBatch,
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
    start_time: 0,
    calibration_points: [(0.0, 0.0); MAX_CALIBRATION_POINTS],
    calibration_point_count: 0,
    battery_voltage: 4300,
    last_activity_time_ms: 0,
    ble_connected: false,
    sleep_state: SleepState::Awake,
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

    // Initialize BLE
    let bluetooth = peripherals.BT;
    let ble_config =
        esp_radio::ble::Config::default().with_default_tx_power(esp_radio::ble::TxPower::P20);
    let connector = BleConnector::new(bluetooth, ble_config).unwrap();
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

    // Start inactivity tracking from boot so the device can auto-sleep if left unused.
    critical_section::with(|cs| {
        DEVICE_STATE.borrow_ref_mut(cs).on_ble_disconnected();
    });

    // Spawn tasks
    spawner.spawn(measurement_task(channel, clock_pin, data_pin, delay, flash).unwrap());
    spawner.spawn(battery_voltage_task(battery_adc, battery_pin).unwrap());
    spawner.spawn(deep_sleep_task(rtc).unwrap());

    let _ = join(ble_task(runner), async {
        loop {
            match advertise(device_name, &mut peripheral, &server).await {
                Ok(conn) => {
                    info!("BLE connection established");

                    let params = trouble_host::prelude::RequestedConnParams {
                        min_connection_interval: Duration::from_millis(15),
                        max_connection_interval: Duration::from_millis(45),
                        max_latency: 0,
                        min_event_length: Duration::from_millis(0),
                        max_event_length: Duration::from_millis(0),
                        supervision_timeout: Duration::from_millis(4000),
                    };
                    if let Err(e) = conn.raw().update_connection_params(&stack, &params).await {
                        warn!(
                            "Failed to request connection params: {:?}",
                            defmt::Debug2Format(&e)
                        );
                    }

                    channel.clear();
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
                    channel.clear();
                    critical_section::with(|cs| {
                        let mut state = DEVICE_STATE.borrow_ref_mut(cs);
                        state.stop_measurement();
                        state.on_ble_disconnected();
                        debug!("BLE connection closed, inactivity timer restarted");
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
    const IDLE_TIMEOUT_MS: u32 = 4 * 60 * 1000; // 4 minutes

    loop {
        let (sleep_state, measurement_status, inactivity_ms, ble_connected) =
            critical_section::with(|cs| {
                let state = DEVICE_STATE.borrow_ref(cs);
                (
                    state.sleep_state,
                    state.measurement_status,
                    state.get_inactivity_elapsed_ms(),
                    state.is_ble_connected(),
                )
            });

        match sleep_state {
            SleepState::Awake => {
                if measurement_status == MeasurementTaskStatus::Disabled {
                    debug!(
                        "Device idle for {:?} ms (connected: {}, timeout: {:?} ms)",
                        inactivity_ms, ble_connected, IDLE_TIMEOUT_MS
                    );

                    if inactivity_ms >= IDLE_TIMEOUT_MS {
                        critical_section::with(|cs| {
                            DEVICE_STATE
                                .borrow_ref_mut(cs)
                                .request_sleep(SleepReason::IdleTimeout);
                        });
                    }
                }
            }
            SleepState::Requested(reason) => {
                debug!(
                    "Waiting for peripherals to power down before sleep: {:?}",
                    reason
                );
            }
            SleepState::Ready(reason) => {
                info!("Entering deep sleep: {:?}", reason);
                Timer::after(Duration::from_millis(20)).await;
                rtc.sleep_deep(&[]);
            }
        }

        Timer::after(Duration::from_secs(1)).await;
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
    let mut measurement_buffer = WeightMeasurementBatch::new();
    let mut hx711_powered_down = false;

    loop {
        // Get current device state
        let (sleep_state, status, start_time) = critical_section::with(|cs| {
            let state = DEVICE_STATE.borrow_ref(cs);
            (
                state.sleep_state,
                state.measurement_status,
                state.start_time,
            )
        });

        if hx711_powered_down && sleep_state == SleepState::Awake {
            info!("Waking HX711 after sleep request was cancelled");
            load_cell.power_up();
            hx711_powered_down = false;
        }

        match sleep_state {
            SleepState::Requested(reason) => {
                if !hx711_powered_down {
                    info!("Powering down HX711 before deep sleep: {:?}", reason);
                    measurement_buffer.clear();
                    load_cell.power_down();
                    hx711_powered_down = true;
                    critical_section::with(|cs| {
                        DEVICE_STATE.borrow_ref_mut(cs).mark_sleep_ready();
                    });
                }
                Timer::after(Duration::from_millis(20)).await;
                continue;
            }
            SleepState::Ready(_) => {
                Timer::after(Duration::from_millis(50)).await;
                continue;
            }
            SleepState::Awake => {}
        }

        match status {
            MeasurementTaskStatus::Disabled => {
                if !measurement_buffer.is_empty() {
                    crate::progressor::DataPoint::weight_measurement(core::mem::take(
                        &mut measurement_buffer,
                    ))
                    .send(channel)
                    .await;
                }
            }
            MeasurementTaskStatus::Tare => {
                // Perform taring operation
                if let Err(e) = load_cell.tare().await {
                    error!("Tare failed: {:?}", defmt::Debug2Format(&e));
                }

                critical_section::with(|cs| {
                    let mut state = DEVICE_STATE.borrow_ref_mut(cs);
                    state.measurement_status = MeasurementTaskStatus::Disabled;
                });
            }
            MeasurementTaskStatus::Enabled => {
                let weight = match load_cell.read_calibrated().await {
                    Ok(weight) => weight,
                    Err(e) => {
                        error!(
                            "Failed to read weight measurement: {:?}",
                            defmt::Debug2Format(&e)
                        );
                        continue;
                    }
                };
                let now = (esp_hal::time::Instant::now().duration_since_epoch()).as_micros() as u32;
                let timestamp = now.wrapping_sub(start_time);
                measurement_buffer.push((weight, timestamp));
                if measurement_buffer.is_full() {
                    crate::progressor::DataPoint::weight_measurement(core::mem::take(
                        &mut measurement_buffer,
                    ))
                    .send(channel)
                    .await;
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
                    Ok(calibration_point) => calibration_point,
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
                        DataPoint::from(ResponseCode::CalibrationFactor(factor))
                            .send(channel)
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
                    state.measurement_status = MeasurementTaskStatus::Disabled;
                    (state.calibration_points, state.calibration_point_count)
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

async fn notify_calibration_points(
    channel: &'static DataPointChannel,
    calibration_points: &[CalibrationPoint],
) {
    for (raw_value, weight) in calibration_points {
        debug!("Notifying calibration point: {:?}", (raw_value, weight));
        DataPoint::from(ResponseCode::CalibrationPoint(*raw_value, *weight))
            .send(channel)
            .await;
    }
}

async fn notify_calibration_factor(channel: &'static DataPointChannel, calibration_factor: f32) {
    debug!("Notifying calibration factor: {:?}", calibration_factor);
    DataPoint::from(ResponseCode::CalibrationFactor(calibration_factor))
        .send(channel)
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
) -> Result<(), Error> {
    let control_point = server.progressor.control_point;
    loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => {
                info!("Device disconnected: {:?}", reason);
                break;
            }
            GattConnectionEvent::Gatt { event } => {
                let immediate_response = if let GattEvent::Write(write_event) = &event
                    && write_event.handle() == control_point.handle
                {
                    let cmd_data = write_event.data();
                    match cmd_data.first().copied() {
                        Some(op_code_byte) => match ControlOpCode::try_from(op_code_byte) {
                            Ok(op_code) => {
                                info!("Control Point Received: {:?}", op_code);
                                critical_section::with(|cs| {
                                    let mut device_state = DEVICE_STATE.borrow_ref_mut(cs);
                                    if op_code.counts_as_activity() {
                                        device_state.record_activity();
                                    }
                                    op_code.process(cmd_data, &mut device_state)
                                })
                            }
                            Err(()) => {
                                critical_section::with(|cs| {
                                    DEVICE_STATE.borrow_ref_mut(cs).record_activity();
                                });
                                warn!("Ignoring unsupported OpCode: {:#x}", op_code_byte);
                                None
                            }
                        },
                        None => {
                            critical_section::with(|cs| {
                                DEVICE_STATE.borrow_ref_mut(cs).record_activity();
                            });
                            warn!("Control Point write with empty payload");
                            None
                        }
                    }
                } else {
                    None
                };

                // Ensure reply is sent
                if let Ok(reply) = event.accept() {
                    reply.send().await;
                } else {
                    warn!("Error sending response");
                }

                if let Some(response) = immediate_response {
                    response.send(channel).await;
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
    let data_point_handle = &server.progressor.data_point;

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
