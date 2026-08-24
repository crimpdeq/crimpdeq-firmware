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
    clock::CpuClock,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    i2c::master::{Config as I2cConfig, I2c},
    interrupt::software::SoftwareInterruptControl,
    rmt::Rmt,
    rtc_cntl::Rtc,
    spi::{
        Mode as SpiMode,
        master::{Config as SpiConfig, Spi},
    },
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_hal_smartled::{SmartLedsAdapterAsync, buffer_size_async};
use esp_radio::ble::controller::BleConnector;
use esp_storage::FlashStorage;
use max170xx::asynch::Max17048;
use panic_rtt_target as _;
use smart_leds::{RGB8, SmartLedsWriteAsync as _, brightness};
use trouble_host::prelude::*;

use crate::{
    ads1220::Ads1220,
    ble::{CONNECTIONS_MAX, L2CAP_CHANNELS_MAX, L2CAP_MTU, Server, advertise},
    progressor::{
        CalibrationPoint,
        ControlOpCode,
        DataPoint,
        DataPointChannel,
        DeviceState,
        MAX_CALIBRATION_POINTS,
        MeasurementTaskStatus,
        ResponseCode,
        SleepReadySource,
        SleepReason,
        SleepState,
        WeightMeasurementBatch,
    },
};

pub mod ads1220;
pub mod ble;
pub mod progressor;

const STATUS_LED_COUNT: usize = 1;
const STATUS_LED_RMT_BUFFER_SIZE: usize = buffer_size_async(STATUS_LED_COUNT);
const STATUS_LED_BRIGHTNESS: u8 = 24;
const STATUS_LED_LOW_BATTERY_MV: u32 = 3500;
const STATUS_LED_CHARGING_RATE_THRESHOLD: f32 = 0.1;
const STATUS_LED_OFF: RGB8 = RGB8 { r: 0, g: 0, b: 0 };
const STATUS_LED_DISCONNECTED: RGB8 = RGB8 { r: 0, g: 0, b: 255 };
const STATUS_LED_CONNECTED: RGB8 = RGB8 { r: 0, g: 255, b: 0 };
const STATUS_LED_LOW_BATTERY: RGB8 = RGB8 { r: 255, g: 0, b: 0 };

#[derive(Clone, Copy, PartialEq)]
enum StatusLedMode {
    Off,
    Disconnected,
    Connected,
    LowBattery,
}

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
    battery_charging: false,
    last_activity_time_ms: 0,
    ble_connected: false,
    sleep_state: SleepState::Awake,
    sleep_measurement_ready: false,
    sleep_status_led_ready: false,
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

    // Initialize the revision-3 ADS1220 load-cell front end.
    // GPIO5=SCLK, GPIO4=DIN/MOSI, GPIO3=/CS, GPIO1=DOUT/MISO.
    let load_cell_spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(2))
            .with_mode(SpiMode::_1),
    )
    .expect("Failed to initialize ADS1220 SPI")
    .with_sck(peripherals.GPIO5)
    .with_mosi(peripherals.GPIO4)
    .with_miso(peripherals.GPIO1)
    .into_async();
    let load_cell_chip_select =
        Output::new(peripherals.GPIO3, Level::High, OutputConfig::default());

    // Initialize Flash Storage
    let flash = FlashStorage::new(peripherals.FLASH);

    // Initialize RTC
    let rtc = Rtc::new(peripherals.LPWR);

    // Initialize MAX17048 fuel gauge over I2C.
    // PCB nets are labelled IO6_SDA/IO7_SCL, but the MAX17048 TDFN datasheet maps
    // pin 7 to SCL and pin 8 to SDA while the KiCad symbol had those two swapped.
    // Actual gauge connections: GPIO7=SDA, GPIO6=SCL, GPIO10=/ALRT,
    // CELL/VDD=+BATT, QSTRT=GND. /ALRT is open-drain and has no external pull-up.
    let _battery_alert_pin = Input::new(
        peripherals.GPIO10,
        InputConfig::default().with_pull(Pull::Up),
    );
    let battery_i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .expect("Failed to initialize battery I2C")
        .with_sda(peripherals.GPIO7)
        .with_scl(peripherals.GPIO6)
        .into_async();
    let battery_gauge = Max17048::new(battery_i2c);

    // Initialize WS2812B status LED on GPIO2 using RMT.
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80))
        .expect("Failed to initialize RMT")
        .into_async();
    let status_led_buffer = mk_static!(
        [esp_hal::rmt::PulseCode; STATUS_LED_RMT_BUFFER_SIZE],
        [esp_hal::rmt::PulseCode::default(); STATUS_LED_RMT_BUFFER_SIZE]
    );
    let status_led = SmartLedsAdapterAsync::new(rmt.channel0, peripherals.GPIO2, status_led_buffer);

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
    spawner.spawn(measurement_task(channel, load_cell_spi, load_cell_chip_select, flash).unwrap());
    spawner.spawn(battery_gauge_task(battery_gauge).unwrap());
    spawner.spawn(status_led_task(status_led).unwrap());
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

fn status_led_mode() -> StatusLedMode {
    critical_section::with(|cs| {
        let state = DEVICE_STATE.borrow_ref(cs);
        if state.sleep_state != SleepState::Awake {
            StatusLedMode::Off
        } else if state.battery_voltage <= STATUS_LED_LOW_BATTERY_MV {
            StatusLedMode::LowBattery
        } else if state.is_ble_connected() {
            StatusLedMode::Connected
        } else {
            StatusLedMode::Disconnected
        }
    })
}

async fn set_status_led(
    led: &mut SmartLedsAdapterAsync<'static, STATUS_LED_RMT_BUFFER_SIZE>,
    color: RGB8,
) {
    if let Err(e) = led
        .write(brightness([color].into_iter(), STATUS_LED_BRIGHTNESS))
        .await
    {
        warn!(
            "Failed to update WS2812B status LED: {:?}",
            defmt::Debug2Format(&e)
        );
    }
}

async fn wait_for_sleep_request(timeout: Duration) -> bool {
    const POLL_INTERVAL_MS: u64 = 50;

    let timeout_ms = timeout.as_millis();
    let mut elapsed_ms = 0;

    while elapsed_ms < timeout_ms {
        let sleep_state = critical_section::with(|cs| DEVICE_STATE.borrow_ref(cs).sleep_state);
        if sleep_state != SleepState::Awake {
            return true;
        }

        let step_ms = (timeout_ms - elapsed_ms).min(POLL_INTERVAL_MS);
        Timer::after(Duration::from_millis(step_ms)).await;
        elapsed_ms += step_ms;
    }

    false
}

#[embassy_executor::task]
async fn status_led_task(mut led: SmartLedsAdapterAsync<'static, STATUS_LED_RMT_BUFFER_SIZE>) {
    set_status_led(&mut led, STATUS_LED_OFF).await;
    let mut sleep_led_ready = false;

    loop {
        let sleep_state = critical_section::with(|cs| DEVICE_STATE.borrow_ref(cs).sleep_state);
        match sleep_state {
            SleepState::Requested(reason) => {
                if !sleep_led_ready {
                    info!("Turning off status LED before deep sleep: {:?}", reason);
                    set_status_led(&mut led, STATUS_LED_OFF).await;
                    sleep_led_ready = true;
                    critical_section::with(|cs| {
                        DEVICE_STATE
                            .borrow_ref_mut(cs)
                            .mark_sleep_ready(SleepReadySource::StatusLed);
                    });
                }
                Timer::after(Duration::from_millis(20)).await;
                continue;
            }
            SleepState::Ready(_) => {
                Timer::after(Duration::from_millis(20)).await;
                continue;
            }
            SleepState::Awake => {
                sleep_led_ready = false;
            }
        }

        match status_led_mode() {
            StatusLedMode::Off => {
                set_status_led(&mut led, STATUS_LED_OFF).await;
                let _ = wait_for_sleep_request(Duration::from_secs(1)).await;
            }
            StatusLedMode::Disconnected => {
                set_status_led(&mut led, STATUS_LED_DISCONNECTED).await;
                let _ = wait_for_sleep_request(Duration::from_secs(1)).await;
            }
            StatusLedMode::Connected => {
                set_status_led(&mut led, STATUS_LED_CONNECTED).await;
                let _ = wait_for_sleep_request(Duration::from_secs(1)).await;
            }
            StatusLedMode::LowBattery => {
                set_status_led(&mut led, STATUS_LED_LOW_BATTERY).await;
                if wait_for_sleep_request(Duration::from_millis(250)).await {
                    continue;
                }
                set_status_led(&mut led, STATUS_LED_OFF).await;
                let _ = wait_for_sleep_request(Duration::from_millis(750)).await;
            }
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

                Timer::after(Duration::from_secs(1)).await;
            }
            SleepState::Requested(reason) => {
                debug!(
                    "Waiting for peripherals to power down before sleep: {:?}",
                    reason
                );
                Timer::after(Duration::from_millis(20)).await;
            }
            SleepState::Ready(reason) => {
                info!("Entering deep sleep: {:?}", reason);
                Timer::after(Duration::from_millis(20)).await;
                rtc.sleep_deep(&[]);
            }
        }
    }
}

#[embassy_executor::task]
async fn battery_gauge_task(mut gauge: Max17048<I2c<'static, Async>>) {
    loop {
        let voltage = gauge.voltage().await;
        let soc = gauge.soc().await;
        let charge_rate = gauge.charge_rate().await;

        match (voltage, soc, charge_rate) {
            (Ok(voltage), Ok(soc), Ok(charge_rate)) => {
                let battery_voltage_mv = (voltage * 1000.0) as u32;
                let battery_charging = charge_rate > STATUS_LED_CHARGING_RATE_THRESHOLD;
                info!(
                    "Battery: {:?} mV, SOC: {:?}%, charge rate: {:?}%/h",
                    battery_voltage_mv, soc, charge_rate
                );

                // Update device state
                critical_section::with(|cs| {
                    let mut state = DEVICE_STATE.borrow_ref_mut(cs);
                    state.battery_voltage = battery_voltage_mv;
                    state.battery_charging = battery_charging;
                });
            }
            (voltage, soc, charge_rate) => {
                warn!(
                    "Failed to read MAX17048 battery gauge: voltage={:?}, soc={:?}, charge_rate={:?}",
                    defmt::Debug2Format(&voltage),
                    defmt::Debug2Format(&soc),
                    defmt::Debug2Format(&charge_rate)
                );
            }
        }

        Timer::after(Duration::from_secs(45)).await;
    }
}

async fn initialize_load_cell(load_cell: &mut Ads1220<'_>) {
    let mut attempt = 1u32;
    loop {
        match load_cell.initialize().await {
            Ok(()) => return,
            Err(error) => {
                error!(
                    "ADS1220 initialization attempt {} failed: {:?}",
                    attempt,
                    defmt::Debug2Format(&error)
                );
                attempt = attempt.saturating_add(1);
                Timer::after(Duration::from_millis(250)).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn measurement_task(
    channel: &'static DataPointChannel,
    spi: Spi<'static, Async>,
    chip_select: Output<'static>,
    flash: FlashStorage<'static>,
) {
    let mut load_cell = Ads1220::new(spi, chip_select, flash);
    initialize_load_cell(&mut load_cell).await;
    if let Err(e) = load_cell.tare().await {
        error!("Initial tare failed: {:?}", defmt::Debug2Format(&e));
    }
    let mut measurement_buffer = WeightMeasurementBatch::new();
    let mut ads1220_powered_down = false;

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

        if ads1220_powered_down && sleep_state == SleepState::Awake {
            info!("Waking ADS1220 after sleep request was cancelled");
            if let Err(error) = load_cell.power_up().await {
                error!(
                    "Failed to wake ADS1220, reinitializing: {:?}",
                    defmt::Debug2Format(&error)
                );
                initialize_load_cell(&mut load_cell).await;
            }
            ads1220_powered_down = false;
        }

        match sleep_state {
            SleepState::Requested(reason) => {
                if !ads1220_powered_down {
                    info!("Powering down ADS1220 before deep sleep: {:?}", reason);
                    measurement_buffer.clear();
                    if let Err(error) = load_cell.power_down().await {
                        error!(
                            "Failed to power down ADS1220: {:?}",
                            defmt::Debug2Format(&error)
                        );
                    }
                    ads1220_powered_down = true;
                    critical_section::with(|cs| {
                        DEVICE_STATE
                            .borrow_ref_mut(cs)
                            .mark_sleep_ready(SleepReadySource::Measurement);
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
                let mut disconnect_after_response = false;
                let immediate_response = if let GattEvent::Write(write_event) = &event
                    && write_event.handle() == control_point.handle
                {
                    let cmd_data = write_event.data();
                    match cmd_data.first().copied() {
                        Some(op_code_byte) => match ControlOpCode::try_from(op_code_byte) {
                            Ok(op_code) => {
                                info!("Control Point Received: {:?}", op_code);
                                disconnect_after_response =
                                    matches!(op_code, ControlOpCode::Shutdown);
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

                if disconnect_after_response {
                    info!("Disconnecting BLE link for shutdown request");
                    conn.raw().disconnect();
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
