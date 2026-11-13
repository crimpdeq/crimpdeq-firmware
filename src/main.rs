#![no_std]
#![no_main]
// The mock build replaces the ADS1220 and MAX17048 code with simulations; the default build still
// checks that code for dead items and imports.
#![cfg_attr(feature = "mock", allow(dead_code, unused_imports))]

use core::cell::RefCell;

use arrayvec::ArrayVec;
use bt_hci::{
    cmd::le::{LeConnUpdate, LeReadLocalSupportedFeatures, LeSetDataLength, LeSetPhy},
    controller::{ControllerCmdAsync, ControllerCmdSync, ExternalController},
};
use crimpdeq_calibration::{
    CalibrationRecord,
    DEFAULT_FACTOR,
    Error as CalibrationError,
    Model,
    Outcome,
    Point,
    wire::{self, LastCommand},
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
    flash::{Config as FlashConfig, Flash},
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    i2c::master::{Config as I2cConfig, I2c},
    rmt::Rmt,
    rtc_cntl::{
        CpuRetentionStorage,
        WakeLock,
        sleep::{LowPower, RtcSleepConfig},
    },
    spi::{
        Mode as SpiMode,
        master::{Config as SpiConfig, Spi},
    },
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_hal_smartled::{RmtSmartLeds, Timing, buffer_size, color_order};
use esp_radio::ble::{RetentionStorage, controller::BleConnector, install_retention_memory};
use max170xx::Max17048;
use smart_leds::{RGB8, SmartLedsWriteAsync as _, brightness};
use trouble_host::prelude::*;

use crate::{
    ads1220::Ads1220,
    ble::{CONNECTIONS_MAX, DEVICE_NAME, L2CAP_CHANNELS_MAX, L2CAP_MTU, Server, advertise},
    progressor::{
        BatteryStatus,
        CONTROL_POINT_MAX_PAYLOAD_SIZE,
        ControlOpCode,
        ControlResponseChannel,
        DataPoint,
        DeviceState,
        LOAD_CELL_COMMANDS,
        LoadCellCommand,
        MAX_BATCH_DURATION,
        MAX_SAMPLES_PER_PACKET,
        MeasurementDataChannel,
        MeasurementTaskStatus,
        ResponseCode,
        SleepReadySource,
        SleepReason,
        SleepState,
        WeightMeasurementBatch,
        measurements_per_packet,
    },
    storage::{Settings, Storage},
};

mod ads1220;
mod ble;
#[cfg(feature = "mock")]
mod mock;
mod progressor;
mod storage;

#[cfg(not(feature = "mock"))]
type LoadCell = Ads1220<'static>;
#[cfg(feature = "mock")]
type LoadCell = mock::MockLoadCell;

const STATUS_LED_COUNT: usize = 1;
const STATUS_LED_RMT_BUFFER_SIZE: usize = buffer_size::<RGB8>(STATUS_LED_COUNT);
const STATUS_LED_BRIGHTNESS: u8 = 24;
/// D4 is a Xinglight XL-2121RGBC-2812B (LCSC C5349957), sent its typical timing: T0H 250 ns
/// (at most 300 ns), T1H 850 ns (620-1000 ns), T0L 1000 ns, T1L 400 ns, bit period of at least
/// 1.2 us and reset of at least 80 us. The crate's `WS2812B_TIMING` sends a 400 ns T0H, which
/// decodes as a one bit, so every color would show as white.
const STATUS_LED_TIMING: Timing = Timing {
    time_0_high: 250,
    time_0_low: 1000,
    time_1_high: 850,
    time_1_low: 400,
    reset_us: 100,
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

    // The deep sleep task arms its own wakeup deadline right before deep sleep, which replaces the
    // one that the light sleep idle hook arms on each idle pass.
    // SAFETY: `LowPower` keeps no state, and the idle hook only uses LPWR while no task runs.
    let low_power = LowPower::new(unsafe { peripherals.LPWR.clone_unchecked() });

    // Enter light sleep whenever all tasks wait and nothing holds a `WakeLock`. The BLE controller
    // refuses the sleeps that would overlap its radio events.
    let mut sleep = esp_rtos::sleep::configure(peripherals.LPWR);
    // The memory that the bootloader used is otherwise unused after boot.
    #[esp_hal::ram(reclaimed, unstable(zeroed))]
    static CPU_RETENTION_MEMORY: CpuRetentionStorage = CpuRetentionStorage::new();
    sleep
        .enable_cpu_powerdown(CPU_RETENTION_MEMORY.take())
        .expect("Failed to install the CPU retention memory");
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start_with_idle_hook(timg0.timer0, sleep.light_sleep_hook);

    // Read the persisted settings before BLE starts and before any task uses them.
    let flash = Flash::new(peripherals.FLASH, FlashConfig::default())
        .expect("Failed to identify the flash");
    let mut storage = Storage::new(flash);
    let stored_settings = storage.read();
    update_device_state(|state| state.settings = stored_settings.clone());

    // Initialize BLE
    let bluetooth = peripherals.BT;
    // On PCB V3, each transmission couples into the ADS1220 input and offsets the conversion it
    // overlaps: with no load and the default calibration, +20 dBm gave single-sample spikes up to
    // 1.4 kg, +9 dBm up to 0.12 kg, and 0 dBm stayed within the ADC noise. +9 dBm trades those
    // smaller spikes for BLE range.
    // Modem sleep turns the radio off between BLE events, and lets the chip light-sleep then. With
    // the retention memory, it also powers the BLE MAC and baseband down.
    #[esp_hal::ram(reclaimed, unstable(zeroed))]
    static BLE_RETENTION_MEMORY: RetentionStorage = RetentionStorage::new();
    install_retention_memory(BLE_RETENTION_MEMORY.take())
        .expect("Failed to install the BLE retention memory");
    let ble_config = esp_radio::ble::Config::default()
        .with_default_tx_power(esp_radio::ble::TxPower::P9)
        .with_modem_sleep(true);
    let connector = BleConnector::new(bluetooth, ble_config).unwrap();
    let controller: ExternalController<_, 1> = ExternalController::new(connector);

    // Initialize the ADS1220 load-cell front end.
    // GPIO5=SCLK, GPIO4=DIN/MOSI, GPIO3=/CS, GPIO1=DOUT/MISO, GPIO0=DRDY.
    // DRDY is a push-pull output, so it needs no pull.
    #[cfg(not(feature = "mock"))]
    let load_cell_data_ready = Input::new(peripherals.GPIO0, InputConfig::default());
    #[cfg(not(feature = "mock"))]
    let load_cell_spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(2))
            .with_mode(SpiMode::_1),
    )
    .expect("Failed to initialize ADS1220 SPI")
    .with_sck(peripherals.GPIO5)
    .with_mosi(peripherals.GPIO4)
    .with_miso(peripherals.GPIO1);
    // The mock build still drives /CS high, next to its 100k pull-up (R23), so the unused ADS1220
    // stays deselected. On the Rust board, GPIO3 is a free header pin.
    #[cfg_attr(feature = "mock", allow(unused_variables))]
    let load_cell_chip_select =
        Output::new(peripherals.GPIO3, Level::High, OutputConfig::default());

    // Initialize MAX17048 fuel gauge over I2C.
    // GPIO6=SDA (I2C_SDA, R20), GPIO7=SCL (I2C_SCL, R21), GPIO10=/ALRT (FG_ALRT).
    // CELL/VDD=VBAT, QSTRT=GND. R20/R21 are 4.7k pull-ups on SDA/SCL. The open-drain /ALRT has no
    // external pull-up, so the internal one keeps it from floating.
    #[cfg(not(feature = "mock"))]
    let _battery_alert_pin = Input::new(
        peripherals.GPIO10,
        InputConfig::default().with_pull(Pull::Up),
    );
    #[cfg(not(feature = "mock"))]
    let battery_i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .expect("Failed to initialize battery I2C")
        .with_sda(peripherals.GPIO6)
        .with_scl(peripherals.GPIO7)
        .into_async();

    // Initialize the XL-2121RGBC-2812B status LED on GPIO2 (LED_DATA, R13 0R to D4 DIN) using RMT.
    // The ESP32-C3 Rust board, which can run the mock build, also has a WS2812 on GPIO2.
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
            #[cfg(not(feature = "mock"))]
            load_cell_spi,
            #[cfg(not(feature = "mock"))]
            load_cell_chip_select,
            #[cfg(not(feature = "mock"))]
            load_cell_data_ready,
        )
        .unwrap(),
    );
    spawner.spawn(storage_task(storage, stored_settings).unwrap());
    #[cfg(not(feature = "mock"))]
    spawner.spawn(battery_gauge_task(battery_i2c, control_channel).unwrap());
    #[cfg(feature = "mock")]
    spawner.spawn(mock_battery_task().unwrap());
    spawner.spawn(status_led_task(status_led).unwrap());
    spawner.spawn(deep_sleep_task(low_power).unwrap());

    join(ble_task(runner), async {
        loop {
            match advertise(&mut peripheral, &server).await {
                Ok(conn) => {
                    info!("Initial ATT MTU: {}", conn.raw().att_mtu());

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
    // Light sleep would stop the RMT clock in the middle of the LED data.
    let _wake_lock = WakeLock::new();
    if let Err(e) = led
        .write(brightness([color].into_iter(), STATUS_LED_BRIGHTNESS))
        .await
    {
        warn!(
            "Failed to update the status LED: {:?}",
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
                        // The mock build stays awake, so it does not need a power cycle after
                        // idling.
                        if cfg!(feature = "mock") {
                            info!(
                                "Idle timeout reached after {} ms, skipping deep sleep in the mock build",
                                inactivity_ms
                            );
                            state_changes.changed().await;
                        } else {
                            update_device_state(|state| {
                                state.request_sleep(SleepReason::IdleTimeout)
                            });
                        }
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
                // The status LED and the ADS1220 stay on 3V3 while the digital domain is powered
                // off. Hold the LED data line low so the LED does not latch noise, and the
                // ADS1220 /CS high, so the pad cannot pull it below its 100k pull-up (R23) and
                // let noise clock in a command.
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

#[cfg(feature = "mock")]
#[embassy_executor::task]
async fn mock_battery_task() {
    let mut battery = mock::MockBattery::new();
    loop {
        let (voltage_mv, status) = battery.read();
        info!(
            "Mock battery: {} mV, SOC: {}%, charging: {}",
            voltage_mv, status.soc_percent, status.charging
        );
        update_device_state(|state| {
            state.battery_voltage = voltage_mv;
            state.battery_status = Some(status);
        });
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
                state.settings.clone(),
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
            if settings.calibration != stored.calibration {
                match storage.write_calibration(&settings.calibration) {
                    Ok(()) => {
                        info!("Calibration stored: {:?}", settings.calibration.active);
                        stored.calibration = settings.calibration.clone();
                    }
                    Err(e) => error!("Failed to store the calibration: {:?}", e),
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
    let Err(TrySendError::Full(measurements)) = channel.try_send(measurements) else {
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

    if let Err(TrySendError::Full(_)) = channel.try_send(measurements) {
        error!("Failed to enqueue measurement after dropping oldest batch");
    }
}

#[embassy_executor::task]
async fn measurement_task(
    control_channel: &'static ControlResponseChannel,
    measurement_channel: &'static MeasurementDataChannel,
    #[cfg(not(feature = "mock"))] spi: Spi<'static, Blocking>,
    #[cfg(not(feature = "mock"))] chip_select: Output<'static>,
    #[cfg(not(feature = "mock"))] data_ready: Input<'static>,
) {
    #[cfg(not(feature = "mock"))]
    let mut load_cell = Ads1220::new(spi.into_async(), chip_select, data_ready);
    #[cfg(feature = "mock")]
    let mut load_cell = mock::MockLoadCell::new();
    let mut initialized = false;
    let mut measurement_buffer = WeightMeasurementBatch::new();
    let mut dropped_measurement_batches = 0u32;
    // Start time of the measurement being sampled. A Start updates the state's start time, so a
    // mismatch is a new measurement, also when Stop and Start both arrive before this task runs.
    let mut streaming_start_time = None;
    let mut calibration_runtime = CalibrationRuntime::default();
    let mut state_changes = STATE_CHANGED
        .receiver()
        .expect("Missing state change receiver for the measurement task");

    loop {
        // Get current device state
        let (sleep_state, status, start_time, sample_rate, calibration) =
            critical_section::with(|cs| {
                let state = DEVICE_STATE.borrow_ref(cs);
                (
                    state.sleep_state,
                    state.measurement_status,
                    state.start_time,
                    state.settings.sample_rate,
                    state.settings.calibration.active,
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
            if let Err(e) = load_cell.initialize(sample_rate).await {
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

        if load_cell.sample_rate() != sample_rate {
            match load_cell.set_sample_rate(sample_rate).await {
                Ok(()) => info!("ADS1220 sample rate set to {} SPS", sample_rate.sps()),
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
            run_load_cell_command(
                command,
                &mut load_cell,
                control_channel,
                &mut calibration_runtime,
            )
            .await;
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

                let weight = match load_cell
                    .read_calibrated(&calibration, calibration_runtime.zero_shift)
                    .await
                {
                    Ok(weight) => weight,
                    Err(e) => {
                        error!("Failed to read weight measurement: {:?}", e);
                        continue;
                    }
                };
                let now = (esp_hal::time::Instant::now().duration_since_epoch()).as_micros() as u32;
                let timestamp = now.wrapping_sub(start_time);
                measurement_buffer.push((weight, timestamp));
                // Full batches regardless of the ATT MTU, so the ring holds the same samples at
                // any MTU; data_processing_task splits them into packets.
                let batch_size =
                    MAX_SAMPLES_PER_PACKET.min(sample_rate.samples_in(MAX_BATCH_DURATION));
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

/// Calibration state of the measurement task that is not persisted.
#[derive(Default)]
struct CalibrationRuntime {
    /// Drift of the unloaded baseline since the calibration, in raw counts, measured by the last
    /// zero update. A new calibration restarts from its own points, so it resets the shift.
    zero_shift: f32,
    /// Last calibration command and its outcome, for the calibration details.
    last: Option<LastCommand>,
}

impl CalibrationRuntime {
    /// Uses an edited calibration and records the outcome of the command that edited it.
    fn apply(&mut self, op_code: ControlOpCode, record: CalibrationRecord, outcome: Outcome) {
        update_device_state(|state| {
            if record.active != state.settings.calibration.active {
                self.zero_shift = 0.0;
            }
            state.set_calibration(record);
        });
        self.last = Some(LastCommand {
            op_code: op_code as u8,
            outcome,
        });
    }
}

fn calibration_record() -> CalibrationRecord {
    critical_section::with(|cs| DEVICE_STATE.borrow_ref(cs).settings.calibration.clone())
}

/// Runs a one-shot load cell command.
///
/// Commands of the linear-only protocol keep their replies: the calibration factor, NaN for a
/// nonlinear model, and the stored points. The calibration model commands reply with the
/// calibration details, which also report the outcome of the last calibration command.
async fn run_load_cell_command(
    command: LoadCellCommand,
    load_cell: &mut LoadCell,
    channel: &'static ControlResponseChannel,
    runtime: &mut CalibrationRuntime,
) {
    match command {
        LoadCellCommand::Tare => {
            if let Err(e) = load_cell.tare().await {
                error!("Tare failed: {:?}", e);
            }
        }
        LoadCellCommand::Calibrate(weight) => {
            // Only this task changes the calibration, so it is unchanged after sampling.
            let mut record = calibration_record();
            let outcome = if record.points.is_full() {
                Outcome::Rejected(CalibrationError::PointLimitReached)
            } else {
                #[cfg(not(feature = "mock"))]
                let raw_value = load_cell.perform_calibration().await;
                #[cfg(feature = "mock")]
                let raw_value = load_cell.perform_calibration(weight).await;
                match raw_value {
                    Ok(raw_value) => record.add_point((raw_value, weight)),
                    Err(e) => {
                        error!("Calibration sampling failed: {:?}", e);
                        Outcome::Rejected(CalibrationError::SamplingFailed)
                    }
                }
            };
            info!("Calibration point of {} kg: {:?}", weight, outcome);
            let stored = !matches!(outcome, Outcome::Rejected(_));
            runtime.apply(ControlOpCode::AddCalibrationPoint, record.clone(), outcome);
            // A rejected point sends nothing, so linear-only clients read the calibration back.
            if stored {
                notify_legacy_calibration(channel, &record).await;
            }
        }
        LoadCellCommand::RemoveCalibrationPoint(index) => {
            let mut record = calibration_record();
            let outcome = record.remove_point(usize::from(index));
            info!("Removing calibration point {}: {:?}", index, outcome);
            let removed = !matches!(outcome, Outcome::Rejected(_));
            runtime.apply(
                ControlOpCode::RemoveCalibrationPoint,
                record.clone(),
                outcome,
            );
            // Calibration factors also answer other commands, so this reply tells the client
            // whether the point was removed, after the new calibration.
            if removed {
                notify_legacy_calibration(channel, &record).await;
            }
            send_load_cell_reply(
                channel,
                DataPoint::from(ResponseCode::CalibrationPointRemoved(removed)),
            )
            .await;
        }
        LoadCellCommand::DefaultCalibration => {
            debug!("Restoring default calibration");
            runtime.apply(
                ControlOpCode::DefaultCalibration,
                CalibrationRecord::DEFAULT,
                Outcome::Applied,
            );
            notify_calibration_factor(channel, DEFAULT_FACTOR).await;
        }
        LoadCellCommand::GetCalibration => {
            let record = calibration_record();
            info!(
                "Calibration: {:?}, points: {:?}",
                record.active,
                record.points.as_slice()
            );
            notify_legacy_calibration(channel, &record).await;
        }
        LoadCellCommand::GetCalibrationDetails => {
            notify_calibration_details(channel, load_cell.tare_value(), runtime).await;
        }
        LoadCellCommand::SetCalibrationModel(code) => {
            let mut record = calibration_record();
            let outcome = match Model::from_code(code) {
                Some(model) => record.select_model(model),
                None => Outcome::Rejected(CalibrationError::UnsupportedModel),
            };
            info!("Selecting calibration model {}: {:?}", code, outcome);
            runtime.apply(ControlOpCode::SetCalibrationModel, record, outcome);
            notify_calibration_details(channel, load_cell.tare_value(), runtime).await;
        }
        LoadCellCommand::SetZero => {
            let calibration = calibration_record().active;
            // A rejected zero update keeps the previous tare too.
            let outcome = match load_cell.read_tare_average().await {
                Ok(unloaded_raw) => match calibration.zero_shift(unloaded_raw as f32) {
                    Ok(zero_shift) => {
                        load_cell.set_tare_value(unloaded_raw);
                        runtime.zero_shift = zero_shift;
                        Outcome::Applied
                    }
                    Err(e) => Outcome::Rejected(e),
                },
                Err(e) => {
                    error!("Zero sampling failed: {:?}", e);
                    Outcome::Rejected(CalibrationError::SamplingFailed)
                }
            };
            info!("Zero update: {:?}, shift: {}", outcome, runtime.zero_shift);
            runtime.last = Some(LastCommand {
                op_code: ControlOpCode::SetZero as u8,
                outcome,
            });
            notify_calibration_details(channel, load_cell.tare_value(), runtime).await;
        }
    }
}

/// Sends the calibration factor and points of the linear-only protocol.
async fn notify_legacy_calibration(
    channel: &'static ControlResponseChannel,
    record: &CalibrationRecord,
) {
    notify_calibration_factor(channel, record.active.legacy_factor()).await;
    notify_calibration_points(channel, &record.points).await;
}

async fn notify_calibration_details(
    channel: &'static ControlResponseChannel,
    tare_raw: i32,
    runtime: &CalibrationRuntime,
) {
    let record = calibration_record();
    for detail in wire::details(&record, tare_raw, runtime.zero_shift, runtime.last) {
        send_load_cell_reply(
            channel,
            DataPoint::from(ResponseCode::CalibrationDetail(detail.payload())),
        )
        .await;
    }
}

async fn notify_calibration_points(
    channel: &'static ControlResponseChannel,
    calibration_points: &[Point],
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

/// Largest link layer payload, and the time it takes on the 1M PHY. A measurement packet then
/// fits one link layer packet instead of four 27-byte ones, so a connection event carries more of
/// them.
const MAX_DATA_LENGTH_OCTETS: u16 = 251;
const MAX_DATA_LENGTH_TIME_US: u16 = 2120;

/// Asks for the longest packets and the 2M PHY, which halves their airtime. Each side keeps its
/// own limits, and a client that supports neither keeps the defaults.
async fn request_fast_link<C, P>(stack: &Stack<'_, C, P>, conn: &GattConnection<'_, '_, P>)
where
    C: Controller
        + ControllerCmdSync<LeSetDataLength>
        + ControllerCmdAsync<LeSetPhy>
        + ControllerCmdSync<LeReadLocalSupportedFeatures>,
    P: PacketPool,
{
    if let Err(e) = conn
        .raw()
        .update_data_length(stack, MAX_DATA_LENGTH_OCTETS, MAX_DATA_LENGTH_TIME_US)
        .await
    {
        warn!(
            "Failed to request the data length: {:?}",
            defmt::Debug2Format(&e)
        );
    }
    if let Err(e) = conn.raw().set_phy(stack, PhyKind::Le2M).await {
        warn!(
            "Failed to request the 2M PHY: {:?}",
            defmt::Debug2Format(&e)
        );
    }
}

/// Requests connection parameters that match the measurement state, and disconnects once deep
/// sleep is requested so the client sees the link close instead of a supervision timeout.
async fn connection_state_task<C, P>(stack: &Stack<'_, C, P>, conn: &GattConnection<'_, '_, P>)
where
    C: Controller
        + ControllerCmdAsync<LeConnUpdate>
        + ControllerCmdSync<LeSetDataLength>
        + ControllerCmdAsync<LeSetPhy>
        + ControllerCmdSync<LeReadLocalSupportedFeatures>,
    P: PacketPool,
{
    request_fast_link(stack, conn).await;
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
            GattConnectionEvent::DataLengthUpdated {
                max_tx_octets,
                max_rx_octets,
                ..
            } => info!(
                "BLE data length: tx={} octets, rx={} octets",
                max_tx_octets, max_rx_octets
            ),
            GattConnectionEvent::PhyUpdated { tx_phy, rx_phy } => {
                info!("BLE PHY: tx={:?}, rx={:?}", tx_phy, rx_phy)
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
    let mut control_first = true;
    let mut att_mtu = 0;

    loop {
        // Alternate priority when both queues have data, so neither starves the other.
        let next = if control_first {
            select(control_channel.receive(), measurement_channel.receive()).await
        } else {
            match select(measurement_channel.receive(), control_channel.receive()).await {
                Either::First(measurements) => Either::Second(measurements),
                Either::Second(control_response) => Either::First(control_response),
            }
        };
        control_first = matches!(next, Either::Second(_));

        let sent = match next {
            Either::First(control_response) => {
                notify_data_point(server, conn, &control_response).await
            }
            Either::Second(measurements) => {
                // The peer may negotiate a larger MTU after the connection starts.
                let current_att_mtu = conn.raw().att_mtu();
                if current_att_mtu != att_mtu {
                    att_mtu = current_att_mtu;
                    info!(
                        "ATT MTU {} selects {} measurements per packet",
                        att_mtu,
                        measurements_per_packet(att_mtu)
                    );
                }
                notify_measurements(
                    server,
                    conn,
                    &measurements,
                    measurements_per_packet(att_mtu),
                )
                .await
            }
        };
        if let Err(e) = sent {
            info!("Error sending Data Point: {:?}", defmt::Debug2Format(&e));
            break;
        }
    }
}

/// Sends a measurement batch in packets of up to `packet_size` measurements.
async fn notify_measurements<P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
    measurements: &WeightMeasurementBatch,
    packet_size: usize,
) -> Result<(), Error> {
    for packet in measurements.chunks(packet_size) {
        notify_data_point(server, conn, &DataPoint::weight_measurement(packet)).await?;
    }
    Ok(())
}

/// Sends a Data Point notification. An exhausted packet pool drops this packet instead of the
/// connection.
async fn notify_data_point<P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
    data_point: &DataPoint,
) -> Result<(), Error> {
    debug!("Sending Data Point: {:?}", data_point);
    match server
        .progressor
        .data_point
        .notify(conn, data_point, false)
        .await
    {
        Err(Error::OutOfMemory) => {
            warn!("No packet buffer, dropping Data Point");
            Ok(())
        }
        result => result,
    }
}
