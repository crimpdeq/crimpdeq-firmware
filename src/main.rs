#![no_std]
#![no_main]

use core::cell::RefCell;

use arrayvec::ArrayVec;
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
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Channel, TrySendError},
    watch::Watch,
};
use embassy_time::{Duration, Instant, Timer};
use esp_hal::{
    Async,
    Blocking,
    Config,
    clock::CpuClock,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    i2c::master::{Config as I2cConfig, I2c},
    rmt::Rmt,
    rtc_cntl::sleep::{LowPower, RtcSleepConfig},
    spi::{
        Mode as SpiMode,
        master::{Config as SpiConfig, Spi},
    },
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_hal_smartled::{RmtSmartLeds, Timing, buffer_size, color_order};
use esp_radio::ble::controller::BleConnector;
use esp_storage::FlashStorage;
use max170xx::Max17048;
use smart_leds::{RGB8, SmartLedsWriteAsync as _, brightness};
use trouble_host::prelude::*;

use crate::{
    ads1220::{Ads1220, DEFAULT_CALIBRATION_FACTOR, fit_calibration_factor},
    ble::{CONNECTIONS_MAX, DEVICE_NAME, L2CAP_CHANNELS_MAX, L2CAP_MTU, Server, advertise},
    progressor::{
        BatteryStatus,
        CONTROL_POINT_MAX_PAYLOAD_SIZE,
        CalibrationPoint,
        ControlOpCode,
        ControlResponseChannel,
        DataPoint,
        DeviceState,
        LOAD_CELL_COMMANDS,
        LoadCellCommand,
        MAX_BATCH_DURATION,
        MAX_CALIBRATION_POINTS,
        MeasurementDataChannel,
        MeasurementTaskStatus,
        ResponseCode,
        SleepReadySource,
        SleepReason,
        SleepState,
        WeightMeasurementBatch,
        measurement_batch_size_for_att_mtu,
    },
    storage::{Settings, Storage},
};

mod ads1220;
mod ble;
mod progressor;
mod storage;

const STATUS_LED_COUNT: usize = 1;
const STATUS_LED_RMT_BUFFER_SIZE: usize = buffer_size::<RGB8>(STATUS_LED_COUNT);
const STATUS_LED_BRIGHTNESS: u8 = 24;
/// Revision-3 D4 is a Worldsemi WS2812B-V6: T0H 220-380 ns, T1H/T0L/T1L 580-1000 ns,
/// bit period >= 1.25 us, reset > 280 us. The crate's `WS2812B_TIMING` sends a 400 ns T0H, which
/// the V6 decodes as a one bit, so every color shows as white.
const STATUS_LED_TIMING: Timing = Timing {
    time_0_high: 300,
    time_0_low: 950,
    time_1_high: 800,
    time_1_low: 800,
    reset_us: 300,
};
const STATUS_LED_LOW_BATTERY_MV: u32 = 3500;
/// Charge rate, in %/h, above which the battery counts as charging.
const CHARGING_RATE_THRESHOLD: f32 = 0.1;
/// Battery voltage at or below which the device warns and enters deep sleep. Below about 3.3 V
/// the 3V3 regulator drops out.
const LOW_BATTERY_SHUTDOWN_MV: u32 = 3300;
/// Matches the MAX17048 VCELL update period in hibernate.
const BATTERY_READ_PERIOD: Duration = Duration::from_secs(45);
/// Registers that the max170xx driver does not expose.
const MAX17048_ADDRESS: u8 = 0x36;
const MAX17048_MODE: u8 = 0x06;
const MAX17048_MODE_HIBSTAT: u16 = 1 << 12;
const MAX17048_HIBRT: u8 = 0x0A;
/// Power-on HIBRT: hibernate after 6 min below 26.6 %/h, which the device's load stays under, and
/// exit on a 60 mV step. VCELL then updates every 45 s instead of every 250 ms.
const MAX17048_HIBRT_DEFAULT: u16 = 0x8030;
const MAX17048_HIBRT_DISABLED: u16 = 0x0000;
/// Longest wait for the MAX17048 to leave hibernate: one hibernate ADC period plus its 3.5 % time
/// base error.
const MAX17048_HIBERNATE_EXIT_TIMEOUT: Duration = Duration::from_secs(47);
/// VCELL averages four 250 ms active-mode conversions.
const MAX17048_ACTIVE_VCELL_SETTLE: Duration = Duration::from_secs(1);
const STATUS_LED_OFF: RGB8 = RGB8 { r: 0, g: 0, b: 0 };
const STATUS_LED_DISCONNECTED: RGB8 = RGB8 { r: 0, g: 0, b: 255 };
const STATUS_LED_CONNECTED: RGB8 = RGB8 { r: 0, g: 255, b: 0 };
const STATUS_LED_LOW_BATTERY: RGB8 = RGB8 { r: 255, g: 0, b: 0 };
/// Delay before retrying failed BLE advertising.
const BLE_RETRY_DELAY: Duration = Duration::from_secs(1);
/// Attempts to store a settings change before giving up, so a flash that keeps failing cannot
/// hold off deep sleep.
const SETTINGS_WRITE_ATTEMPTS: u32 = 3;
/// Delay before retrying a failed settings write.
const SETTINGS_WRITE_RETRY_DELAY: Duration = Duration::from_secs(1);
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

/// Number of tasks that wait for device state changes: measurement, status LED, deep sleep,
/// storage and connection state.
const STATE_CHANGE_RECEIVERS: usize = 5;
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

fn update_measurement_batch_size(att_mtu: u16) {
    let batch_size = measurement_batch_size_for_att_mtu(att_mtu);
    let previous = update_device_state(|state| {
        core::mem::replace(&mut state.measurement_batch_size, batch_size)
    });

    if previous != batch_size {
        info!(
            "ATT MTU {} selects {} measurements per packet",
            att_mtu, batch_size
        );
    }
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

    // Read the persisted settings before BLE starts and before any task uses them.
    let mut storage = Storage::new(FlashStorage::new(peripherals.FLASH));
    let stored_settings = storage.read();
    update_device_state(|state| state.settings = stored_settings);

    // Initialize BLE
    let bluetooth = peripherals.BT;
    // Each transmission couples into the ADS1220 input and offsets the conversion it overlaps:
    // with no load and the default calibration, +20 dBm gave single-sample spikes up to 1.4 kg,
    // +9 dBm up to 0.12 kg, and 0 dBm stayed within the ADC noise. +9 dBm trades those smaller
    // spikes for BLE range.
    let ble_config =
        esp_radio::ble::Config::default().with_default_tx_power(esp_radio::ble::TxPower::P9);
    let connector = BleConnector::new(bluetooth, ble_config).unwrap();
    let controller: ExternalController<_, 1> = ExternalController::new(connector);

    // Initialize the revision-3 ADS1220 load-cell front end.
    // GPIO5=SCLK, GPIO4=DIN/MOSI, GPIO3=/CS, GPIO1=DOUT/DRDY.
    // GPIO1 is both the SPI MISO and the data ready input. DOUT/DRDY is high impedance while /CS
    // is high, so the pull-up keeps it from reading as data ready.
    let load_cell_data_ready = Input::new(
        peripherals.GPIO1,
        InputConfig::default().with_pull(Pull::Up),
    );
    let load_cell_spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(2))
            .with_mode(SpiMode::_1),
    )
    .expect("Failed to initialize ADS1220 SPI")
    .with_sck(peripherals.GPIO5)
    .with_mosi(peripherals.GPIO4)
    .with_miso(load_cell_data_ready.peripheral_input());
    let load_cell_chip_select =
        Output::new(peripherals.GPIO3, Level::High, OutputConfig::default());

    // Initialize low-power control.
    let low_power = LowPower::new(peripherals.LPWR);

    // Initialize MAX17048 fuel gauge over I2C.
    // GPIO7=SDA (U5.8, IO7_SDA, R21), GPIO6=SCL (U5.7, IO6_SCL, R20),
    // GPIO10=/ALRT (U5.5, IO10_ALRT, R22). CELL/VDD=+BATT, QSTRT=GND.
    // Revision-3 R20/R21/R22 provide 10k pull-ups on SCL/SDA/ALRT;
    // retain the internal ALRT pull-up.
    let _battery_alert_pin = Input::new(
        peripherals.GPIO10,
        InputConfig::default().with_pull(Pull::Up),
    );
    let battery_i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .expect("Failed to initialize battery I2C")
        .with_sda(peripherals.GPIO7)
        .with_scl(peripherals.GPIO6)
        .into_async();

    // Initialize WS2812B-V6 status LED on GPIO2 (U1.5, IO2_LED, R13 0R to D4 DIN) using RMT.
    let status_led_rmt_rate = Rate::from_mhz(80);
    let rmt = Rmt::new(peripherals.RMT, status_led_rmt_rate)
        .expect("Failed to initialize RMT")
        .into_async();
    let status_led = StatusLed::new(
        STATUS_LED_TIMING,
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

    // Keep control responses independent from the lossy measurement ring.
    let control_channel = mk_static!(ControlResponseChannel, Channel::new());
    let measurement_channel = mk_static!(MeasurementDataChannel, Channel::new());

    // Start inactivity tracking from boot so the device can auto-sleep if left unused.
    update_device_state(|state| state.on_ble_disconnected());

    // Run the measurement task above the BLE host, so packet transmission does not delay ADC
    // reads past the next conversion at high sample rates.
    let measurement_executor = mk_static!(
        esp_rtos::embassy::InterruptExecutor<2>,
        esp_rtos::embassy::InterruptExecutor::new(peripherals.FROM_CPU_INTR2)
    );
    let measurement_spawner = measurement_executor.start(esp_hal::interrupt::Priority::Priority2);

    // Spawn tasks
    measurement_spawner.spawn(
        measurement_task(
            control_channel,
            measurement_channel,
            load_cell_spi,
            load_cell_chip_select,
            load_cell_data_ready,
        )
        .unwrap(),
    );
    spawner.spawn(storage_task(storage, stored_settings).unwrap());
    spawner.spawn(battery_gauge_task(battery_i2c, control_channel).unwrap());
    spawner.spawn(status_led_task(status_led).unwrap());
    spawner.spawn(deep_sleep_task(low_power).unwrap());

    join(ble_task(runner), async {
        loop {
            match advertise(&mut peripheral, &server).await {
                Ok(conn) => {
                    let initial_att_mtu = conn.raw().att_mtu();
                    update_measurement_batch_size(initial_att_mtu);
                    info!("Initial ATT MTU: {}", initial_att_mtu);

                    let initial_params = conn.raw().params();
                    info!(
                        "Initial BLE parameters: interval={} us, latency={}",
                        initial_params.conn_interval.as_micros(),
                        initial_params.peripheral_latency
                    );

                    control_channel.clear();
                    measurement_channel.clear();
                    update_device_state(|state| state.on_ble_connected());
                    // run until any task ends (usually because the connection has been closed),
                    // then return to advertising state.
                    select3(
                        gatt_events_task(&server, &conn, control_channel),
                        data_processing_task(&server, &conn, control_channel, measurement_channel),
                        connection_state_task(&stack, &conn),
                    )
                    .await;
                    // Mark the link closed before clearing, so a reply blocked on the full
                    // channel completes and the measurement task stops queuing more.
                    update_device_state(|state| {
                        state.on_ble_disconnected();
                        debug!("BLE connection closed, inactivity timer restarted");
                    });
                    control_channel.clear();
                    measurement_channel.clear();
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

/// Runs the BLE host.
///
/// Running the host again starts with an HCI reset, which stops advertising and drops the
/// connection that the rest of the BLE code still waits on. A failure resets the chip instead, so
/// the whole BLE stack restarts together.
async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    if let Err(e) = runner.run().await {
        error!("BLE runner failed: {:?}", defmt::Debug2Format(&e));
    }
    error!("BLE runner stopped, resetting");
    esp_hal::system::software_reset()
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
    const DISCONNECTED_IDLE_TIMEOUT_MS: u32 = 4 * 60 * 1000;
    /// Covers long rests between sets, while a client left connected cannot keep the device awake
    /// indefinitely. Only power cycling wakes the device from deep sleep.
    const CONNECTED_IDLE_TIMEOUT_MS: u32 = 30 * 60 * 1000;
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
                    let idle_timeout_ms = if ble_connected {
                        CONNECTED_IDLE_TIMEOUT_MS
                    } else {
                        DISCONNECTED_IDLE_TIMEOUT_MS
                    };
                    debug!(
                        "Device idle for {:?} ms (connected: {}, timeout: {:?} ms)",
                        inactivity_ms, ble_connected, idle_timeout_ms
                    );

                    if inactivity_ms >= idle_timeout_ms {
                        update_device_state(|state| state.request_sleep(SleepReason::IdleTimeout));
                        continue;
                    }

                    // Wait for the idle timeout or for activity that restarts it.
                    let remaining_ms = idle_timeout_ms - inactivity_ms;
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
                    "Waiting for peripherals to power down and BLE to disconnect before sleep: {:?}",
                    reason
                );
                state_changes.changed().await;
            }
            SleepState::Ready(reason) => {
                Timer::after(Duration::from_millis(20)).await;
                // Advertising continues until deep sleep, so a connection during the delay can
                // cancel the sleep request or send it back to wait for a disconnect.
                let still_ready = critical_section::with(|cs| {
                    DEVICE_STATE.borrow_ref(cs).sleep_state == SleepState::Ready(reason)
                });
                if !still_ready {
                    continue;
                }
                info!("Entering deep sleep: {:?}", reason);
                // The WS2812B and the ADS1220 stay on +3V3 while the digital domain is powered
                // off. Hold the LED data line low so the LED does not latch noise, and the
                // ADS1220 /CS high, as it has no pull-up, so noise cannot clock in a command.
                // SAFETY: the status LED is already off and the ADS1220 powered down, and their
                // tasks stop driving the pins once sleep is ready. No await follows, so they
                // cannot run again before `sleep_deep`, which does not return; the holds are
                // released on the next boot.
                let mut status_led_pin = Output::new(
                    unsafe { esp_hal::peripherals::GPIO2::steal() },
                    Level::Low,
                    OutputConfig::default(),
                );
                status_led_pin.set_pad_hold(true);
                let mut load_cell_chip_select = Output::new(
                    unsafe { esp_hal::peripherals::GPIO3::steal() },
                    Level::High,
                    OutputConfig::default(),
                );
                load_cell_chip_select.set_pad_hold(true);
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
    mut i2c: I2c<'static, Async>,
    channel: &'static ControlResponseChannel,
) {
    // The gauge stays powered from the battery, so a power cut during a low battery check would
    // leave hibernate disabled.
    restore_max17048_hibernate(&mut i2c).await;

    loop {
        if read_battery(&mut i2c)
            .await
            .is_some_and(BatteryReading::is_low)
            && confirm_low_battery(&mut i2c).await
        {
            warn!("Battery low, entering deep sleep");
            if critical_section::with(|cs| DEVICE_STATE.borrow_ref(cs).is_ble_connected()) {
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

        Timer::after(BATTERY_READ_PERIOD).await;
    }
}

/// Persists settings changes once measurement stops.
///
/// Flash writes from the priority-2 measurement executor while streaming made the BLE controller
/// interrupt panic and reboot the device (#83), so this task writes from thread mode and only
/// while not measuring.
#[embassy_executor::task]
async fn storage_task(mut storage: Storage, mut stored: Settings) {
    let mut state_changes = STATE_CHANGED
        .receiver()
        .expect("Missing state change receiver for the storage task");
    let mut failed_attempts = 0;

    loop {
        let (write_pending, measuring, settings) = critical_section::with(|cs| {
            let state = DEVICE_STATE.borrow_ref(cs);
            (
                state.settings_write_pending,
                state.measurement_status == MeasurementTaskStatus::Enabled,
                state.settings,
            )
        });

        if write_pending && !measuring {
            // A failed write keeps `stored`, so the next attempt retries it.
            if settings.sample_rate != stored.sample_rate {
                match storage.write_sample_rate(settings.sample_rate) {
                    Ok(()) => {
                        info!("Sample rate stored: {} SPS", settings.sample_rate.sps());
                        stored.sample_rate = settings.sample_rate;
                    }
                    Err(e) => error!("Failed to store the sample rate: {:?}", e),
                }
            }
            if settings.calibration_factor != stored.calibration_factor {
                match storage.write_calibration_factor(settings.calibration_factor) {
                    Ok(()) => {
                        info!("Calibration factor stored: {}", settings.calibration_factor);
                        stored.calibration_factor = settings.calibration_factor;
                    }
                    Err(e) => error!("Failed to store the calibration factor: {:?}", e),
                }
            }
            if settings != stored {
                failed_attempts += 1;
                if failed_attempts < SETTINGS_WRITE_ATTEMPTS {
                    select(
                        Timer::after(SETTINGS_WRITE_RETRY_DELAY),
                        state_changes.changed(),
                    )
                    .await;
                    continue;
                }
                error!("Settings not stored after {} attempts", failed_attempts);
            }
            failed_attempts = 0;
            update_device_state(|state| state.end_settings_write(settings));
        }

        state_changes.changed().await;
    }
}

#[derive(Clone, Copy)]
struct BatteryReading {
    voltage_mv: u32,
    charging: bool,
}

impl BatteryReading {
    fn is_low(self) -> bool {
        self.voltage_mv <= LOW_BATTERY_SHUTDOWN_MV && !self.charging
    }
}

/// Reads the gauge and stores the result in the device state.
async fn read_battery(i2c: &mut I2c<'static, Async>) -> Option<BatteryReading> {
    let mut gauge = Max17048::new(i2c);
    let voltage = gauge.voltage().await;
    let soc = gauge.soc().await;
    let charge_rate = gauge.charge_rate().await;

    let (Ok(voltage), Ok(soc), Ok(charge_rate)) = (&voltage, &soc, &charge_rate) else {
        warn!(
            "Failed to read MAX17048 battery gauge: voltage={:?}, soc={:?}, charge_rate={:?}",
            defmt::Debug2Format(&voltage),
            defmt::Debug2Format(&soc),
            defmt::Debug2Format(&charge_rate)
        );
        return None;
    };
    let reading = BatteryReading {
        voltage_mv: (voltage * 1000.0) as u32,
        charging: *charge_rate > CHARGING_RATE_THRESHOLD,
    };
    info!(
        "Battery: {:?} mV, SOC: {:?}%, charge rate: {:?}%/h",
        reading.voltage_mv, soc, charge_rate
    );

    update_device_state(|state| {
        state.battery_voltage = reading.voltage_mv;
        state.battery_status = Some(BatteryStatus {
            // Rounded; the model can report slightly above 100 % when full.
            soc_percent: (soc.clamp(0.0, 100.0) + 0.5) as u8,
            charging: reading.charging,
        });
    });
    Some(reading)
}

/// Takes a second reading after a low one, to ignore load transients. Hibernate is disabled for
/// it, so the reading is a new VCELL sample within seconds instead of the next one up to 45 s
/// later.
async fn confirm_low_battery(i2c: &mut I2c<'static, Async>) -> bool {
    if !update_device_state(DeviceState::begin_battery_gauge_check) {
        return false;
    }

    let confirmed =
        match write_max17048_register(i2c, MAX17048_HIBRT, MAX17048_HIBRT_DISABLED).await {
            Ok(()) if wait_for_max17048_active(i2c).await => {
                Timer::after(MAX17048_ACTIVE_VCELL_SETTLE).await;
                read_battery(i2c).await.is_some_and(BatteryReading::is_low)
            }
            Ok(()) => false,
            Err(e) => {
                warn!(
                    "Failed to disable MAX17048 hibernate: {:?}",
                    defmt::Debug2Format(&e)
                );
                false
            }
        };

    // Also after a failed write, which the gauge may still have applied.
    restore_max17048_hibernate(i2c).await;
    update_device_state(DeviceState::end_battery_gauge_check);
    confirmed
}

/// Writes the default HIBRT, retrying transient I2C failures. When every attempt fails, deep sleep
/// still proceeds: staying awake draws far more than the gauge in active mode, and the next boot
/// restores HIBRT.
async fn restore_max17048_hibernate(i2c: &mut I2c<'static, Async>) {
    const ATTEMPTS: u8 = 5;
    for attempt in 1..=ATTEMPTS {
        match write_max17048_register(i2c, MAX17048_HIBRT, MAX17048_HIBRT_DEFAULT).await {
            Ok(()) => return,
            Err(e) => warn!(
                "Failed to restore MAX17048 hibernate thresholds ({}/{}): {:?}",
                attempt,
                ATTEMPTS,
                defmt::Debug2Format(&e)
            ),
        }
        Timer::after(Duration::from_millis(100)).await;
    }
    error!("MAX17048 hibernate stays disabled until the next boot");
}

/// Waits for MODE.HibStat to clear. The datasheet does not say whether disabling hibernate takes
/// effect at once or at the next hibernate ADC sample.
async fn wait_for_max17048_active(i2c: &mut I2c<'static, Async>) -> bool {
    let deadline = Instant::now() + MAX17048_HIBERNATE_EXIT_TIMEOUT;
    loop {
        match read_max17048_register(i2c, MAX17048_MODE).await {
            Ok(mode) if mode & MAX17048_MODE_HIBSTAT == 0 => return true,
            Ok(_) if Instant::now() >= deadline => {
                warn!("MAX17048 still in hibernate after disabling it");
                return false;
            }
            Ok(_) => {}
            Err(e) => {
                warn!(
                    "Failed to read MAX17048 mode: {:?}",
                    defmt::Debug2Format(&e)
                );
                return false;
            }
        }
        Timer::after(Duration::from_millis(250)).await;
    }
}

async fn write_max17048_register(
    i2c: &mut I2c<'static, Async>,
    register: u8,
    value: u16,
) -> Result<(), esp_hal::i2c::master::Error> {
    let [high, low] = value.to_be_bytes();
    i2c.write_async(MAX17048_ADDRESS, &[register, high, low])
        .await
}

async fn read_max17048_register(
    i2c: &mut I2c<'static, Async>,
    register: u8,
) -> Result<u16, esp_hal::i2c::master::Error> {
    let mut value = [0; 2];
    i2c.write_read_async(MAX17048_ADDRESS, &[register], &mut value)
        .await?;
    Ok(u16::from_be_bytes(value))
}

fn enqueue_measurement_batch(
    channel: &'static MeasurementDataChannel,
    measurements: WeightMeasurementBatch,
    dropped_batches: &mut u32,
) {
    let data_point = DataPoint::weight_measurement(measurements);
    let Err(TrySendError::Full(data_point)) = channel.try_send(data_point) else {
        return;
    };

    // Overflow policy: discard the oldest batch and retain the newest samples.
    if channel.try_receive().is_ok() {
        *dropped_batches = dropped_batches.saturating_add(1);
        if *dropped_batches == 1 || (*dropped_batches).is_multiple_of(120) {
            warn!(
                "Measurement ring full; dropped {} oldest batches",
                *dropped_batches
            );
        }
    }

    if let Err(TrySendError::Full(_)) = channel.try_send(data_point) {
        error!("Failed to enqueue measurement after dropping oldest batch");
    }
}

#[embassy_executor::task]
async fn measurement_task(
    control_channel: &'static ControlResponseChannel,
    measurement_channel: &'static MeasurementDataChannel,
    spi: Spi<'static, Blocking>,
    chip_select: Output<'static>,
    data_ready: Input<'static>,
) {
    let mut load_cell = Ads1220::new(spi.into_async(), chip_select, data_ready);
    let mut initialized = false;
    let mut measurement_buffer = WeightMeasurementBatch::new();
    let mut dropped_measurement_batches = 0u32;
    // Start time of the measurement being sampled. A Start updates the state's start time, so a
    // mismatch is a new measurement, also when Stop and Start both arrive before this task runs.
    let mut streaming_start_time = None;
    let mut state_changes = STATE_CHANGED
        .receiver()
        .expect("Missing state change receiver for the measurement task");

    loop {
        // Get current device state
        let (sleep_state, status, start_time, measurement_batch_size, settings) =
            critical_section::with(|cs| {
                let state = DEVICE_STATE.borrow_ref(cs);
                (
                    state.sleep_state,
                    state.measurement_status,
                    state.start_time,
                    state.measurement_batch_size,
                    state.settings,
                )
            });

        match sleep_state {
            SleepState::Requested(reason) => {
                if !load_cell.is_powered_down() {
                    info!("Powering down ADS1220 before deep sleep: {:?}", reason);
                    if let Err(e) = load_cell.power_down().await {
                        error!("Failed to power down ADS1220: {:?}", e);
                    }
                }
                measurement_buffer.clear();
                measurement_channel.clear();
                streaming_start_time = None;
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

        // Retry inside the loop, so a missing or faulty ADS1220 cannot block sleep requests.
        if !initialized {
            if let Err(e) = load_cell.initialize(settings.sample_rate).await {
                error!("ADS1220 initialization failed: {:?}", e);
                select(
                    Timer::after(Duration::from_millis(250)),
                    state_changes.changed(),
                )
                .await;
                continue;
            }
            if let Err(e) = load_cell.tare().await {
                error!("Initial tare failed: {:?}", e);
            }
            initialized = true;
            continue;
        }

        if load_cell.sample_rate() != settings.sample_rate {
            match load_cell.set_sample_rate(settings.sample_rate).await {
                Ok(()) => info!(
                    "ADS1220 sample rate set to {} SPS",
                    settings.sample_rate.sps()
                ),
                Err(e) => {
                    error!("Failed to set ADS1220 sample rate: {:?}", e);
                    Timer::after(Duration::from_millis(250)).await;
                }
            }
            continue;
        }

        if let Ok(command) = LOAD_CELL_COMMANDS.try_receive() {
            // Send the pending samples before the command changes the tare or calibration.
            if !measurement_buffer.is_empty() {
                enqueue_measurement_batch(
                    measurement_channel,
                    core::mem::take(&mut measurement_buffer),
                    &mut dropped_measurement_batches,
                );
            }
            if command.uses_load_cell() && load_cell.is_powered_down() {
                info!("Waking ADS1220 for {:?}", command);
                if let Err(e) = load_cell.wake().await {
                    error!("ADS1220 wake failed, dropping {:?}: {:?}", command, e);
                    continue;
                }
            }
            run_load_cell_command(command, &mut load_cell, control_channel).await;
            continue;
        }

        match status {
            MeasurementTaskStatus::Disabled => {
                streaming_start_time = None;
                if !measurement_buffer.is_empty() {
                    enqueue_measurement_batch(
                        measurement_channel,
                        core::mem::take(&mut measurement_buffer),
                        &mut dropped_measurement_batches,
                    );
                }
                if !load_cell.is_powered_down() {
                    debug!("Powering down idle ADS1220");
                    if let Err(e) = load_cell.power_down().await {
                        error!("Failed to power down ADS1220: {:?}", e);
                    }
                }
                select(
                    state_changes.changed(),
                    LOAD_CELL_COMMANDS.ready_to_receive(),
                )
                .await;
            }
            MeasurementTaskStatus::Enabled => {
                if streaming_start_time != Some(start_time) {
                    // Samples of the previous measurement would join the new one with timestamps
                    // from its timeline.
                    measurement_buffer.clear();
                    measurement_channel.clear();
                    if load_cell.is_powered_down() {
                        info!("Waking ADS1220");
                    }
                    if let Err(e) = load_cell.wake().await {
                        // The ADS1220 is still powered down, so the next iteration retries.
                        error!("ADS1220 wake failed: {:?}", e);
                        continue;
                    }
                    // Start the timestamps once the ADS1220 converts, also when a queued tare or
                    // calibration woke it before the measurement started.
                    streaming_start_time = update_device_state(|state| {
                        (state.measurement_status == MeasurementTaskStatus::Enabled).then(|| {
                            state.start_time = (esp_hal::time::Instant::now()
                                .duration_since_epoch())
                            .as_micros() as u32;
                            state.start_time
                        })
                    });
                    // The state may have changed while the ADS1220 woke.
                    continue;
                }

                let weight = match load_cell.read_calibrated(settings.calibration_factor).await {
                    Ok(weight) => weight,
                    Err(e) => {
                        error!("Failed to read weight measurement: {:?}", e);
                        continue;
                    }
                };
                let now = (esp_hal::time::Instant::now().duration_since_epoch()).as_micros() as u32;
                let timestamp = now.wrapping_sub(start_time);
                measurement_buffer.push((weight, timestamp));
                let batch_size =
                    measurement_batch_size.min(settings.sample_rate.samples_in(MAX_BATCH_DURATION));
                if measurement_buffer.len() >= batch_size {
                    enqueue_measurement_batch(
                        measurement_channel,
                        core::mem::take(&mut measurement_buffer),
                        &mut dropped_measurement_batches,
                    );
                }
            }
        }
    }
}

/// Runs a one-shot load cell command.
async fn run_load_cell_command(
    command: LoadCellCommand,
    load_cell: &mut Ads1220<'static>,
    channel: &'static ControlResponseChannel,
) {
    match command {
        LoadCellCommand::Tare => {
            if let Err(e) = load_cell.tare().await {
                error!("Tare failed: {:?}", e);
            }
        }
        LoadCellCommand::Calibrate(weight) => {
            // Like the other calibration failures, a full buffer sends no reply, so the client
            // reads the unchanged points back with GetCalibration. Only this task changes the
            // points, so they are unchanged after sampling.
            let (mut calibration_points, stored_points) = critical_section::with(|cs| {
                let state = DEVICE_STATE.borrow_ref(cs);
                (state.calibration_points, state.calibration_point_count)
            });
            if stored_points >= MAX_CALIBRATION_POINTS {
                warn!(
                    "Calibration point buffer full (max {}), ignoring new point",
                    MAX_CALIBRATION_POINTS
                );
                return;
            }

            let raw_value = match load_cell.perform_calibration().await {
                Ok(raw_value) => raw_value,
                Err(e) => {
                    error!("Calibration sampling failed: {:?}", e);
                    return;
                }
            };

            calibration_points[stored_points] = (raw_value, weight);
            let calibration_point_count = stored_points + 1;
            let points = &calibration_points[..calibration_point_count];

            if calibration_point_count < 2 {
                update_device_state(|state| {
                    state.calibration_points = calibration_points;
                    state.calibration_point_count = calibration_point_count;
                });
                info!("Calibration needs at least two points before applying.");
                return;
            }

            // A point is stored only with an accepted fit, so a rejected one does not skew later
            // fits. A stored first point was never validated and may be the wrong one, so it is
            // dropped too, letting the calibration restart instead of rejecting every new point.
            let Some(factor) = fit_calibration_factor(points) else {
                error!("Rejected calibration point {:?}", (raw_value, weight));
                if stored_points == 1 {
                    warn!("Dropping the unvalidated first calibration point");
                    update_device_state(|state| state.calibration_point_count = 0);
                }
                return;
            };
            info!("Calibration factor applied: {}", factor);
            store_calibration(channel, calibration_points, calibration_point_count, factor).await;
        }
        LoadCellCommand::RemoveCalibrationPoint(index) => {
            // Calibration factors also answer other commands, so this reply tells the client
            // whether the point was removed, after the new calibration.
            let removed = remove_calibration_point(channel, index).await;
            send_load_cell_reply(
                channel,
                DataPoint::from(ResponseCode::CalibrationPointRemoved(removed)),
            )
            .await;
        }
        LoadCellCommand::DefaultCalibration => {
            debug!("Restoring default calibration factor");
            update_device_state(|state| {
                state.set_calibration_factor(DEFAULT_CALIBRATION_FACTOR);
                state.calibration_point_count = 0;
            });
            notify_calibration_factor(channel, DEFAULT_CALIBRATION_FACTOR).await;
        }
        LoadCellCommand::GetCalibration => {
            let (calibration_factor, calibration_points, calibration_point_count) =
                critical_section::with(|cs| {
                    let state = DEVICE_STATE.borrow_ref(cs);
                    (
                        state.settings.calibration_factor,
                        state.calibration_points,
                        state.calibration_point_count,
                    )
                });
            notify_calibration_factor(channel, calibration_factor).await;

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

/// Removes the calibration point at the index and refits the factor to the remaining points.
/// Returns false, keeping the calibration, for a missing point or remaining points whose fit is
/// rejected.
async fn remove_calibration_point(channel: &'static ControlResponseChannel, index: u8) -> bool {
    let (mut calibration_points, stored_points, factor) = critical_section::with(|cs| {
        let state = DEVICE_STATE.borrow_ref(cs);
        (
            state.calibration_points,
            state.calibration_point_count,
            state.settings.calibration_factor,
        )
    });
    let index = usize::from(index);
    if index >= stored_points {
        warn!(
            "No calibration point {} to remove ({} stored)",
            index, stored_points
        );
        return false;
    }

    calibration_points.copy_within(index + 1..stored_points, index);
    let calibration_point_count = stored_points - 1;
    // Fewer than two points do not define a factor, so the current one stays until the next
    // point is added.
    let factor = if calibration_point_count < 2 {
        factor
    } else {
        match fit_calibration_factor(&calibration_points[..calibration_point_count]) {
            Some(factor) => factor,
            None => {
                error!(
                    "Keeping calibration point {}: the remaining points do not fit",
                    index
                );
                return false;
            }
        }
    };
    info!("Removed calibration point {}, factor: {}", index, factor);
    store_calibration(channel, calibration_points, calibration_point_count, factor).await;
    true
}

/// Stores the calibration points with their factor and notifies the client of both.
async fn store_calibration(
    channel: &'static ControlResponseChannel,
    calibration_points: [CalibrationPoint; MAX_CALIBRATION_POINTS],
    calibration_point_count: usize,
    factor: f32,
) {
    update_device_state(|state| {
        state.calibration_points = calibration_points;
        state.calibration_point_count = calibration_point_count;
        state.set_calibration_factor(factor);
    });
    notify_calibration_factor(channel, factor).await;
    notify_calibration_points(channel, &calibration_points[..calibration_point_count]).await;
}

async fn notify_calibration_points(
    channel: &'static ControlResponseChannel,
    calibration_points: &[CalibrationPoint],
) {
    for (raw_value, weight) in calibration_points {
        debug!("Notifying calibration point: {:?}", (raw_value, weight));
        send_load_cell_reply(
            channel,
            DataPoint::from(ResponseCode::CalibrationPoint(*raw_value, *weight)),
        )
        .await;
    }
}

async fn notify_calibration_factor(
    channel: &'static ControlResponseChannel,
    calibration_factor: f32,
) {
    debug!("Notifying calibration factor: {:?}", calibration_factor);
    send_load_cell_reply(
        channel,
        DataPoint::from(ResponseCode::CalibrationFactor(calibration_factor)),
    )
    .await;
}

/// Queues a load cell command reply for the connected client. Replies are dropped once BLE
/// disconnects: nothing drains the channel then, and a full channel would block the measurement
/// task, and with it sleep, until the next connection.
async fn send_load_cell_reply(channel: &'static ControlResponseChannel, reply: DataPoint) {
    if critical_section::with(|cs| DEVICE_STATE.borrow_ref(cs).is_ble_connected()) {
        channel.send(reply).await;
    }
}

/// Connection parameters while measuring. The interval is meant to carry up to 1000 samples/s.
/// Apple's accessory guidelines require a minimum interval of at least 15 ms, and a maximum 15 ms
/// above it unless both are 15 ms.
const MEASURING_CONNECTION_PARAMS: RequestedConnParams = RequestedConnParams {
    min_connection_interval: Duration::from_millis(15),
    max_connection_interval: Duration::from_millis(15),
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

/// Requests connection parameters that match the measurement state, and disconnects once deep
/// sleep is requested so the client sees the link close instead of a supervision timeout.
async fn connection_state_task<C, P>(stack: &Stack<'_, C, P>, conn: &GattConnection<'_, '_, P>)
where
    C: Controller
        + ControllerCmdAsync<LeConnUpdate>
        + ControllerCmdSync<LeReadLocalSupportedFeatures>,
    P: PacketPool,
{
    let Some(mut state_changes) = STATE_CHANGED.receiver() else {
        error!("Missing state change receiver for the connection state task");
        return core::future::pending().await;
    };
    let mut measuring = None;

    loop {
        let (sleep_state, now_measuring) = critical_section::with(|cs| {
            let state = DEVICE_STATE.borrow_ref(cs);
            (
                state.sleep_state,
                state.measurement_status == MeasurementTaskStatus::Enabled,
            )
        });
        if let SleepState::Requested(reason) = sleep_state {
            info!("Disconnecting BLE link before deep sleep: {:?}", reason);
            conn.raw().disconnect();
            // gatt_events_task ends the connection once the disconnect completes.
            return core::future::pending().await;
        }
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
    control_channel: &'static ControlResponseChannel,
) -> Result<(), Error> {
    let control_point = server.progressor.control_point;
    loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => {
                info!("Device disconnected: {:?}", reason);
                break;
            }
            GattConnectionEvent::ConnectionParamsUpdated {
                conn_interval,
                peripheral_latency,
                supervision_timeout,
            } => {
                info!(
                    "BLE parameters: interval={} us, latency={}, timeout={} ms",
                    conn_interval.as_micros(),
                    peripheral_latency,
                    supervision_timeout.as_millis()
                );
                let (measuring, sample_rate) = critical_section::with(|cs| {
                    let state = DEVICE_STATE.borrow_ref(cs);
                    (
                        state.measurement_status == MeasurementTaskStatus::Enabled,
                        state.settings.sample_rate,
                    )
                });
                if measuring
                    && (conn_interval > MEASURING_CONNECTION_PARAMS.max_connection_interval
                        || peripheral_latency != 0)
                {
                    warn!(
                        "Negotiated BLE parameters may not sustain {} SPS: interval={} us, latency={}",
                        sample_rate.sps(),
                        conn_interval.as_micros(),
                        peripheral_latency
                    );
                }
            }
            GattConnectionEvent::Gatt { event } => {
                // Copy the command so it runs only once the write is accepted. Accepting a write
                // that does not fit the characteristic still succeeds, as the error goes in the
                // reply, so such writes are not commands.
                let command = match &event {
                    GattEvent::Write(write_event)
                        if write_event.handle() == control_point.handle =>
                    {
                        write_event.with_data(|offset, data| {
                            let command =
                                ArrayVec::<u8, CONTROL_POINT_MAX_PAYLOAD_SIZE>::try_from(data)
                                    .ok()
                                    .filter(|_| offset == 0);
                            if command.is_none() {
                                warn!(
                                    "Ignoring Control Point write of {} bytes at offset {}",
                                    data.len(),
                                    offset
                                );
                            }
                            command
                        })
                    }
                    _ => None,
                };

                match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(e) => {
                        warn!("Failed to accept GATT event: {:?}", defmt::Debug2Format(&e));
                        continue;
                    }
                }

                let Some(command) = command else {
                    continue;
                };
                let mut disconnect_after_response = false;
                let immediate_response = match command.first().copied() {
                    Some(op_code_byte) => match ControlOpCode::try_from(op_code_byte) {
                        Ok(op_code) => {
                            info!("Control Point Received: {:?}", op_code);
                            disconnect_after_response = matches!(op_code, ControlOpCode::Shutdown);
                            update_device_state(|device_state| {
                                if op_code.counts_as_activity() {
                                    device_state.record_activity();
                                }
                                op_code.process(&command, device_state)
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
                };

                // Waiting for queue space would stall this loop and leave incoming packets
                // holding the shared packet pool.
                if let Some(response) = immediate_response
                    && control_channel.try_send(response).is_err()
                {
                    warn!("Control response queue full, dropping reply");
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
    Ok(())
}

/// Process data and send notifications to the client
async fn data_processing_task<P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
    control_channel: &'static ControlResponseChannel,
    measurement_channel: &'static MeasurementDataChannel,
) {
    let data_point_handle = &server.progressor.data_point;
    let mut control_first = true;

    loop {
        // The peer may negotiate a larger MTU after the connection starts.
        update_measurement_batch_size(conn.raw().att_mtu());

        // Alternate priority when both queues have packets, so neither starves the other.
        let (data_point, is_control_response) = if control_first {
            match select(control_channel.receive(), measurement_channel.receive()).await {
                Either::First(control_response) => (control_response, true),
                Either::Second(measurement) => (measurement, false),
            }
        } else {
            match select(measurement_channel.receive(), control_channel.receive()).await {
                Either::First(measurement) => (measurement, false),
                Either::Second(control_response) => (control_response, true),
            }
        };
        control_first = !is_control_response;
        debug!("Sending Data Point: {:?}", data_point);

        // Send notification with the data packet. An exhausted packet pool drops this packet
        // instead of the connection.
        match data_point_handle.notify(conn, &data_point, false).await {
            Ok(()) => {}
            Err(Error::OutOfMemory) => warn!("No packet buffer, dropping Data Point"),
            Err(e) => {
                info!("Error sending Data Point: {:?}", defmt::Debug2Format(&e));
                break;
            }
        }
    }
}
