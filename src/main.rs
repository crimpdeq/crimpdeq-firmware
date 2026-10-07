#![no_std]
#![no_main]

use core::cell::RefCell;

use bt_hci::{
    cmd::le::{LeConnUpdate, LeReadLocalSupportedFeatures},
    controller::{ControllerCmdAsync, ControllerCmdSync, ExternalController},
};
use critical_section::Mutex;
use defmt::{debug, error, info, warn};
use embassy_executor::Spawner;
use embassy_futures::{
    join::join,
    select::{Either, select, select3},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, watch::Watch};
use embassy_time::{Duration, Timer};
use esp_hal::{
    Async,
    Config,
    clock::CpuClock,
    delay::Delay,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    i2c::master::{Config as I2cConfig, I2c},
    rmt::Rmt,
    rtc_cntl::sleep::{LowPower, RtcSleepConfig},
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_hal_smartled::{RmtSmartLeds, WS2812B_TIMING, buffer_size, color_order};
use esp_radio::ble::controller::BleConnector;
use esp_storage::FlashStorage;
use max170xx::Max17048;
use smart_leds::{RGB8, SmartLedsWriteAsync as _, brightness};
use trouble_host::prelude::*;

use crate::{
    ble::{CONNECTIONS_MAX, DEVICE_NAME, L2CAP_CHANNELS_MAX, L2CAP_MTU, Server, advertise},
    hx711::Hx711,
    progressor::{
        CalibrationPoint,
        ControlOpCode,
        DataPoint,
        DataPointChannel,
        DeviceState,
        LOAD_CELL_COMMANDS,
        LoadCellCommand,
        MAX_CALIBRATION_POINTS,
        MeasurementTaskStatus,
        ResponseCode,
        SleepReadySource,
        SleepReason,
        SleepState,
        WeightMeasurementBatch,
    },
};

pub mod ble;
pub mod hx711;
pub mod progressor;

const STATUS_LED_COUNT: usize = 1;
const STATUS_LED_RMT_BUFFER_SIZE: usize = buffer_size::<RGB8>(STATUS_LED_COUNT);
const STATUS_LED_BRIGHTNESS: u8 = 24;
const STATUS_LED_LOW_BATTERY_MV: u32 = 3500;
/// Charge rate, in %/h, above which the battery counts as charging.
const CHARGING_RATE_THRESHOLD: f32 = 0.1;
/// Battery voltage at or below which the device warns and enters deep sleep. Below about 3.3 V
/// the 3V3 regulator drops out.
const LOW_BATTERY_SHUTDOWN_MV: u32 = 3300;
/// Consecutive low battery readings needed before shutting down, to ignore load transients.
const LOW_BATTERY_SHUTDOWN_READINGS: u8 = 2;
const STATUS_LED_OFF: RGB8 = RGB8 { r: 0, g: 0, b: 0 };
const STATUS_LED_DISCONNECTED: RGB8 = RGB8 { r: 0, g: 0, b: 255 };
const STATUS_LED_CONNECTED: RGB8 = RGB8 { r: 0, g: 255, b: 0 };
const STATUS_LED_LOW_BATTERY: RGB8 = RGB8 { r: 255, g: 0, b: 0 };
/// Delay before retrying a failed BLE operation.
const BLE_RETRY_DELAY: Duration = Duration::from_secs(1);
type StatusLed = RmtSmartLeds<'static, STATUS_LED_RMT_BUFFER_SIZE, Async, RGB8, color_order::Grb>;

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
static DEVICE_STATE: Mutex<RefCell<DeviceState>> = Mutex::new(RefCell::new(DeviceState::new()));

/// Logs the panic over RTT. Debug builds then halt so a debugger can inspect the panic; release
/// builds reset the chip, so a panic does not leave the device hung with the radio and peripherals
/// powered.
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    critical_section::with(|_| {
        error!("{}", defmt::Display2Format(info));

        #[cfg(debug_assertions)]
        loop {
            core::hint::spin_loop();
        }

        #[cfg(not(debug_assertions))]
        esp_hal::system::software_reset()
    })
}

/// Number of tasks that wait for device state changes: measurement, status LED, deep sleep and
/// connection parameters.
const STATE_CHANGE_RECEIVERS: usize = 4;
/// Signals device state changes, so tasks can wait for them instead of polling.
static STATE_CHANGED: Watch<CriticalSectionRawMutex, (), STATE_CHANGE_RECEIVERS> = Watch::new();

/// Updates the device state and, if it changed, wakes the tasks that wait for state changes.
fn update_device_state<R>(update: impl FnOnce(&mut DeviceState) -> R) -> R {
    let (result, changed) = critical_section::with(|cs| {
        let mut state = DEVICE_STATE.borrow_ref_mut(cs);
        let previous = state.clone();
        let result = update(&mut state);
        (result, *state != previous)
    });
    if changed {
        STATE_CHANGED.sender().send(());
    }
    result
}

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
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

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

    // Initialize low-power control.
    let low_power = LowPower::new(peripherals.LPWR);

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
    let status_led_rmt_rate = Rate::from_mhz(80);
    let rmt = Rmt::new(peripherals.RMT, status_led_rmt_rate)
        .expect("Failed to initialize RMT")
        .into_async();
    let status_led = StatusLed::new(
        WS2812B_TIMING,
        rmt.channel0,
        peripherals.GPIO2,
        status_led_rmt_rate,
    )
    .expect("Failed to initialize status LED");

    // Use the last 6 bytes of the DEVICE_NAME for the address
    let name_bytes = DEVICE_NAME.as_bytes();
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
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(address)
        .build();
    let mut peripheral = stack.peripheral();
    let runner = stack.runner();

    info!("Starting advertising and GATT service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: DEVICE_NAME,
        appearance: &appearance::UNKNOWN,
    }))
    .unwrap();

    // Data point channel for communication between tasks
    let channel = mk_static!(DataPointChannel, Channel::new());

    // Start inactivity tracking from boot so the device can auto-sleep if left unused.
    update_device_state(|state| state.on_ble_disconnected());

    // Spawn tasks
    spawner.spawn(measurement_task(channel, clock_pin, data_pin, delay, flash).unwrap());
    spawner.spawn(battery_gauge_task(battery_gauge, channel).unwrap());
    spawner.spawn(status_led_task(status_led).unwrap());
    spawner.spawn(deep_sleep_task(low_power).unwrap());

    join(ble_task(runner), async {
        loop {
            match advertise(&mut peripheral, &server).await {
                Ok(conn) => {
                    info!("BLE connection established");

                    channel.clear();
                    update_device_state(|state| state.on_ble_connected());
                    // run until any task ends (usually because the connection has been closed),
                    // then return to advertising state.
                    select3(
                        gatt_events_task(&server, &conn, channel),
                        data_processing_task(&server, &conn, channel),
                        connection_params_task(&stack, &conn),
                    )
                    .await;
                    channel.clear();
                    update_device_state(|state| {
                        state.stop_measurement();
                        state.on_ble_disconnected();
                        debug!("BLE connection closed, inactivity timer restarted");
                    });
                }
                Err(e) => {
                    error!("BLE advertising failed: {:?}", defmt::Debug2Format(&e));
                    Timer::after(BLE_RETRY_DELAY).await;
                }
            }
        }
    })
    .await;

    unreachable!("BLE tasks never return")
}

async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(e) = runner.run().await {
            error!("BLE runner failed: {:?}", defmt::Debug2Format(&e));
            Timer::after(BLE_RETRY_DELAY).await;
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

async fn set_status_led(led: &mut StatusLed, color: RGB8) {
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

#[embassy_executor::task]
async fn status_led_task(mut led: StatusLed) {
    let mut state_changes = STATE_CHANGED
        .receiver()
        .expect("Missing state change receiver for the status LED task");
    set_status_led(&mut led, STATUS_LED_OFF).await;
    let mut current_mode = StatusLedMode::Off;
    let mut sleep_led_ready = false;

    loop {
        let sleep_state = critical_section::with(|cs| DEVICE_STATE.borrow_ref(cs).sleep_state);
        match sleep_state {
            SleepState::Requested(reason) => {
                if !sleep_led_ready {
                    info!("Turning off status LED before deep sleep: {:?}", reason);
                    set_status_led(&mut led, STATUS_LED_OFF).await;
                    current_mode = StatusLedMode::Off;
                    sleep_led_ready = true;
                    update_device_state(|state| {
                        state.mark_sleep_ready(SleepReadySource::StatusLed)
                    });
                }
                state_changes.changed().await;
                continue;
            }
            SleepState::Ready(_) => {
                state_changes.changed().await;
                continue;
            }
            SleepState::Awake => {
                sleep_led_ready = false;
            }
        }

        let mode = status_led_mode();
        if mode == StatusLedMode::LowBattery {
            // Blink until the state changes.
            set_status_led(&mut led, STATUS_LED_LOW_BATTERY).await;
            if let Either::First(_) = select(
                Timer::after(Duration::from_millis(250)),
                state_changes.changed(),
            )
            .await
            {
                set_status_led(&mut led, STATUS_LED_OFF).await;
                select(
                    Timer::after(Duration::from_millis(750)),
                    state_changes.changed(),
                )
                .await;
            }
            current_mode = StatusLedMode::LowBattery;
            continue;
        }

        // Only write the LED when its color changes.
        if mode != current_mode {
            let color = match mode {
                StatusLedMode::Off | StatusLedMode::LowBattery => STATUS_LED_OFF,
                StatusLedMode::Disconnected => STATUS_LED_DISCONNECTED,
                StatusLedMode::Connected => STATUS_LED_CONNECTED,
            };
            set_status_led(&mut led, color).await;
            current_mode = mode;
        }
        state_changes.changed().await;
    }
}

#[embassy_executor::task]
async fn deep_sleep_task(mut low_power: LowPower<'static>) {
    const IDLE_TIMEOUT_MS: u32 = 4 * 60 * 1000; // 4 minutes
    const DEEP_SLEEP_FAILSAFE_SECS: u64 = 10 * 365 * 24 * 60 * 60;

    let mut state_changes = STATE_CHANGED
        .receiver()
        .expect("Missing state change receiver for the deep sleep task");

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
                        update_device_state(|state| state.request_sleep(SleepReason::IdleTimeout));
                        continue;
                    }

                    // Wait for the idle timeout or for activity that restarts it.
                    let remaining_ms = IDLE_TIMEOUT_MS - inactivity_ms;
                    select(
                        Timer::after(Duration::from_millis(remaining_ms.into())),
                        state_changes.changed(),
                    )
                    .await;
                } else {
                    state_changes.changed().await;
                }
            }
            SleepState::Requested(reason) => {
                debug!(
                    "Waiting for peripherals to power down before sleep: {:?}",
                    reason
                );
                state_changes.changed().await;
            }
            SleepState::Ready(reason) => {
                info!("Entering deep sleep: {:?}", reason);
                Timer::after(Duration::from_millis(20)).await;
                // Drive the WS2812B data line low and hold it so the LED does not latch
                // noise while the digital domain is powered off.
                // SAFETY: the status LED is already off and its task stops writing once sleep
                // is ready. No await follows, so it cannot run again before `sleep_deep`, which
                // does not return; the hold is released on the next boot.
                let mut status_led_pin = Output::new(
                    unsafe { esp_hal::peripherals::GPIO2::steal() },
                    Level::Low,
                    OutputConfig::default(),
                );
                status_led_pin.set_pad_hold(true);
                low_power.set_wakeup_deadline(
                    esp_hal::time::Instant::now()
                        + esp_hal::time::Duration::from_secs(DEEP_SLEEP_FAILSAFE_SECS),
                );
                low_power.sleep_deep(RtcSleepConfig::deep());
            }
        }
    }
}

#[embassy_executor::task]
async fn battery_gauge_task(
    mut gauge: Max17048<I2c<'static, Async>>,
    channel: &'static DataPointChannel,
) {
    let mut low_battery_readings = 0;

    loop {
        let voltage = gauge.voltage().await;
        let soc = gauge.soc().await;
        let charge_rate = gauge.charge_rate().await;

        match (voltage, soc, charge_rate) {
            (Ok(voltage), Ok(soc), Ok(charge_rate)) => {
                let battery_voltage_mv = (voltage * 1000.0) as u32;
                let battery_charging = charge_rate > CHARGING_RATE_THRESHOLD;
                info!(
                    "Battery: {:?} mV, SOC: {:?}%, charge rate: {:?}%/h",
                    battery_voltage_mv, soc, charge_rate
                );

                // Update device state
                let ble_connected = update_device_state(|state| {
                    state.battery_voltage = battery_voltage_mv;
                    state.is_ble_connected()
                });

                if battery_voltage_mv <= LOW_BATTERY_SHUTDOWN_MV && !battery_charging {
                    low_battery_readings += 1;
                } else {
                    low_battery_readings = 0;
                }

                if low_battery_readings >= LOW_BATTERY_SHUTDOWN_READINGS {
                    warn!(
                        "Battery at {:?} mV, entering deep sleep",
                        battery_voltage_mv
                    );
                    if ble_connected {
                        if channel
                            .try_send(DataPoint::from(ResponseCode::LowPowerWarning))
                            .is_err()
                        {
                            warn!("Failed to queue low power warning");
                        }
                        // Give the notification time to reach the client.
                        Timer::after(Duration::from_millis(500)).await;
                    }
                    update_device_state(|state| state.request_sleep(SleepReason::LowBattery));
                }
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

#[embassy_executor::task]
async fn measurement_task(
    channel: &'static DataPointChannel,
    clock_pin: Output<'static>,
    data_pin: Input<'static>,
    delay: Delay,
    flash: FlashStorage<'static>,
) {
    let mut load_cell = Hx711::new(data_pin, clock_pin, delay, flash);
    if let Err(e) = load_cell.settle().await {
        error!("Initial HX711 settle failed: {:?}", defmt::Debug2Format(&e));
    }
    if let Err(e) = load_cell.tare().await {
        error!("Initial tare failed: {:?}", defmt::Debug2Format(&e));
    }
    let mut measurement_buffer = WeightMeasurementBatch::new();
    let mut state_changes = STATE_CHANGED
        .receiver()
        .expect("Missing state change receiver for the measurement task");

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

        match sleep_state {
            SleepState::Requested(reason) => {
                if !load_cell.is_powered_down() {
                    info!("Powering down HX711 before deep sleep: {:?}", reason);
                    load_cell.power_down();
                }
                measurement_buffer.clear();
                LOAD_CELL_COMMANDS.clear();
                update_device_state(|state| state.mark_sleep_ready(SleepReadySource::Measurement));
                state_changes.changed().await;
                continue;
            }
            SleepState::Ready(_) => {
                state_changes.changed().await;
                continue;
            }
            SleepState::Awake => {}
        }

        if let Ok(command) = LOAD_CELL_COMMANDS.try_receive() {
            if command.uses_load_cell() && load_cell.is_powered_down() {
                info!("Waking HX711 for {:?}", command);
                if let Err(e) = load_cell.wake().await {
                    error!("HX711 wake failed: {:?}", defmt::Debug2Format(&e));
                }
            }
            run_load_cell_command(command, &mut load_cell, channel).await;
            continue;
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
                if !load_cell.is_powered_down() {
                    debug!("Powering down idle HX711");
                    load_cell.power_down();
                }
                select(
                    state_changes.changed(),
                    LOAD_CELL_COMMANDS.ready_to_receive(),
                )
                .await;
            }
            MeasurementTaskStatus::Enabled => {
                if load_cell.is_powered_down() {
                    info!("Waking HX711");
                    if let Err(e) = load_cell.wake().await {
                        error!("HX711 wake failed: {:?}", defmt::Debug2Format(&e));
                    }
                    // Start the timestamps once the HX711 delivers settled readings.
                    update_device_state(|state| {
                        if state.measurement_status == MeasurementTaskStatus::Enabled {
                            state.start_time = (esp_hal::time::Instant::now()
                                .duration_since_epoch())
                            .as_micros() as u32;
                        }
                    });
                    // The state may have changed while the HX711 settled.
                    continue;
                }

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
        }
    }
}

/// Runs a one-shot load cell command.
async fn run_load_cell_command(
    command: LoadCellCommand,
    load_cell: &mut Hx711<'static>,
    channel: &'static DataPointChannel,
) {
    match command {
        LoadCellCommand::Tare => {
            if let Err(e) = load_cell.tare().await {
                error!("Tare failed: {:?}", defmt::Debug2Format(&e));
            }
        }
        LoadCellCommand::Calibrate(weight) => {
            if !weight.is_finite() || weight < 0.0 {
                error!("Ignoring invalid calibration weight: {}", weight);
                return;
            }

            // Use the load cell's own calibration method to collect a calibration point
            let calibration_point = match load_cell.perform_calibration().await {
                Ok(calibration_point) => calibration_point,
                Err(e) => {
                    error!("Calibration sampling failed: {:?}", defmt::Debug2Format(&e));
                    return;
                }
            };
            if !calibration_point.is_finite() {
                error!(
                    "Ignoring invalid calibration raw point: {}",
                    calibration_point
                );
                return;
            }

            let (calibration_points, calibration_point_count) = update_device_state(|state| {
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
        LoadCellCommand::DefaultCalibration => {
            // Reset calibration to default values
            if let Err(e) = load_cell.default_calibration_factor() {
                error!(
                    "Error applying default calibration: {:?}",
                    defmt::Debug2Format(&e)
                );
            } else {
                notify_calibration_factor(channel, load_cell.current_calibration_factor()).await;
            }
            update_device_state(|state| state.calibration_point_count = 0);
        }
        LoadCellCommand::GetCalibration => {
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
                let state = DEVICE_STATE.borrow_ref(cs);
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

/// Connection parameters while measuring.
const MEASURING_CONNECTION_PARAMS: RequestedConnParams = RequestedConnParams {
    min_connection_interval: Duration::from_millis(15),
    max_connection_interval: Duration::from_millis(45),
    max_latency: 0,
    min_event_length: Duration::from_millis(0),
    max_event_length: Duration::from_millis(0),
    supervision_timeout: Duration::from_millis(4000),
};

/// Connection parameters while connected and not measuring. The peripheral latency only delays
/// commands from the client, by up to 250 ms.
const IDLE_CONNECTION_PARAMS: RequestedConnParams = RequestedConnParams {
    min_connection_interval: Duration::from_millis(30),
    max_connection_interval: Duration::from_millis(50),
    max_latency: 4,
    min_event_length: Duration::from_millis(0),
    max_event_length: Duration::from_millis(0),
    supervision_timeout: Duration::from_millis(4000),
};

/// Requests connection parameters that match the measurement state.
async fn connection_params_task<C, P>(stack: &Stack<'_, C, P>, conn: &GattConnection<'_, '_, P>)
where
    C: Controller
        + ControllerCmdAsync<LeConnUpdate>
        + ControllerCmdSync<LeReadLocalSupportedFeatures>,
    P: PacketPool,
{
    let Some(mut state_changes) = STATE_CHANGED.receiver() else {
        error!("Missing state change receiver for the connection parameters task");
        return core::future::pending().await;
    };
    let mut measuring = None;

    loop {
        let now_measuring = critical_section::with(|cs| {
            DEVICE_STATE.borrow_ref(cs).measurement_status == MeasurementTaskStatus::Enabled
        });
        if measuring != Some(now_measuring) {
            let params = if now_measuring {
                &MEASURING_CONNECTION_PARAMS
            } else {
                &IDLE_CONNECTION_PARAMS
            };
            if let Err(e) = conn.raw().update_connection_params(stack, params).await {
                warn!(
                    "Failed to request connection params: {:?}",
                    defmt::Debug2Format(&e)
                );
            }
            measuring = Some(now_measuring);
        }
        state_changes.changed().await;
    }
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
                    write_event.with_data(|_, cmd_data| match cmd_data.first().copied() {
                        Some(op_code_byte) => match ControlOpCode::try_from(op_code_byte) {
                            Ok(op_code) => {
                                info!("Control Point Received: {:?}", op_code);
                                disconnect_after_response =
                                    matches!(op_code, ControlOpCode::Shutdown);
                                update_device_state(|device_state| {
                                    if op_code.counts_as_activity() {
                                        device_state.record_activity();
                                    }
                                    op_code.process(cmd_data, device_state)
                                })
                            }
                            Err(()) => {
                                update_device_state(|state| state.record_activity());
                                warn!("Ignoring unsupported OpCode: {:#x}", op_code_byte);
                                None
                            }
                        },
                        None => {
                            update_device_state(|state| state.record_activity());
                            warn!("Control Point write with empty payload");
                            None
                        }
                    })
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
    update_device_state(|device_state| {
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
        if let Err(e) = data_point_handle.notify(conn, &data_point, false).await {
            info!("Error sending Data Point: {:?}", defmt::Debug2Format(&e));
            break;
        }
    }
}
