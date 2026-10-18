//! Progressor data types
//!
//! See [Tindeq API documentation] for more information
//!
//! [Tindeq API documentation]: https://tindeq.com/progressor_api/
use arrayvec::ArrayVec;
use defmt::{Format, error, info, warn};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use esp_hal::time;
use trouble_host::types::gatt_traits::AsGatt;

use crate::{ads1220::SampleRate, storage::Settings};

/// Number of high-priority control responses that can be queued.
const CONTROL_RESPONSE_CHANNEL_SIZE: usize = 16;
/// Number of measurement packets retained by the BLE transport ring buffer.
const MEASUREMENT_CHANNEL_SIZE: usize = 80;
/// Channel reserved for calibration, status, and other control responses.
pub type ControlResponseChannel =
    Channel<CriticalSectionRawMutex, DataPoint, CONTROL_RESPONSE_CHANNEL_SIZE>;
/// Ring buffer used to decouple measurement acquisition from BLE transmission.
pub type MeasurementDataChannel =
    Channel<CriticalSectionRawMutex, DataPoint, MEASUREMENT_CHANNEL_SIZE>;
/// Number of load cell commands that can wait for the measurement task.
const LOAD_CELL_COMMAND_QUEUE_SIZE: usize = 4;
/// Queue of one-shot commands for the measurement task.
pub static LOAD_CELL_COMMANDS: Channel<
    CriticalSectionRawMutex,
    LoadCellCommand,
    LOAD_CELL_COMMAND_QUEUE_SIZE,
> = Channel::new();

/// Maximum number of samples packed into one BLE measurement packet.
pub const MAX_SAMPLES_PER_PACKET: usize = 10;
/// Longest time a batch collects samples, so low sample rates still update the client often.
pub const MAX_BATCH_DURATION: embassy_time::Duration = embassy_time::Duration::from_millis(50);
/// Size in bytes of one packed weight measurement (f32 weight + u32 timestamp).
const WEIGHT_MEASUREMENT_SIZE: usize = 8;
/// ATT notification opcode/handle plus the Progressor response code/length.
const MEASUREMENT_PACKET_OVERHEAD: usize = 5;
/// Maximum size of the payload written to the Control Point characteristic.
pub const CONTROL_POINT_MAX_PAYLOAD_SIZE: usize = 10;
/// Batch of weight measurements sent in a single BLE packet.
pub type WeightMeasurementBatch = ArrayVec<(f32, u32), MAX_SAMPLES_PER_PACKET>;
/// Size of the Data Point response code and payload length.
const DATA_POINT_HEADER_SIZE: usize = 2;
/// Maximum size of the payload sent in a Data Point notification.
pub const DATA_POINT_MAX_PAYLOAD_SIZE: usize = MAX_SAMPLES_PER_PACKET * WEIGHT_MEASUREMENT_SIZE;

/// Choose the largest batch that fits the negotiated ATT MTU.
pub const fn measurement_batch_size_for_att_mtu(att_mtu: u16) -> usize {
    let available = (att_mtu as usize).saturating_sub(MEASUREMENT_PACKET_OVERHEAD);
    let batch_size = available / WEIGHT_MEASUREMENT_SIZE;
    if batch_size == 0 {
        1
    } else if batch_size > MAX_SAMPLES_PER_PACKET {
        MAX_SAMPLES_PER_PACKET
    } else {
        batch_size
    }
}

/// Number of bytes in the device ID
const DEVICE_ID_SIZE: usize = 6;
/// Progressor ID from the `DEVICE_ID` build variable, 12 hex digits with the most significant
/// byte first. A malformed ID fails the build.
const DEVICE_ID: [u8; DEVICE_ID_SIZE] = parse_device_id(env!("DEVICE_ID"));

const fn parse_device_id(id: &str) -> [u8; DEVICE_ID_SIZE] {
    let digits = id.as_bytes();
    assert!(
        digits.len() == 2 * DEVICE_ID_SIZE,
        "DEVICE_ID must be 12 hex digits"
    );
    let mut bytes = [0; DEVICE_ID_SIZE];
    let mut i = 0;
    while i < DEVICE_ID_SIZE {
        bytes[i] = (hex_digit(digits[2 * i]) << 4) | hex_digit(digits[2 * i + 1]);
        i += 1;
    }
    bytes
}

const fn hex_digit(digit: u8) -> u8 {
    match digit {
        b'0'..=b'9' => digit - b'0',
        b'a'..=b'f' => digit - b'a' + 10,
        b'A'..=b'F' => digit - b'A' + 10,
        _ => panic!("DEVICE_ID must be 12 hex digits"),
    }
}

/// Maximum number of calibration points to store
pub const MAX_CALIBRATION_POINTS: usize = 20;

/// Calibration point storing raw value and known weight
pub type CalibrationPoint = (f32, f32);

/// Fuel gauge reading reported to clients.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct BatteryStatus {
    /// State of charge from the MAX17048 model, 0-100 %.
    pub soc_percent: u8,
    pub charging: bool,
}

/// Status of the weight measurement task
#[derive(Copy, Debug, Clone, PartialEq)]
pub enum MeasurementTaskStatus {
    /// Measurements are enabled
    Enabled,
    /// Measurements are disabled
    Disabled,
}

/// One-shot command handled by the measurement task.
#[derive(Copy, Clone, Debug, PartialEq, Format)]
pub enum LoadCellCommand {
    /// Taring the scale (used in ClimbHarder App)
    Tare,
    /// Adds a calibration point with the known weight in kg
    Calibrate(f32),
    /// Removes the stored calibration point at the index
    RemoveCalibrationPoint(u8),
    /// Restores default calibration values
    DefaultCalibration,
    /// Get the calibration values
    GetCalibration,
}

impl LoadCellCommand {
    /// Returns whether this command needs readings from the load cell.
    pub fn uses_load_cell(self) -> bool {
        matches!(self, Self::Tare | Self::Calibrate(_))
    }
}

/// Reason why the device is transitioning to sleep.
#[derive(Copy, Clone, Debug, PartialEq, Format)]
pub enum SleepReason {
    /// No user or BLE activity has been observed for too long.
    IdleTimeout,
    /// The battery is too low to keep the 3V3 rail regulated.
    LowBattery,
}

/// Sleep transition state.
#[derive(Copy, Clone, Debug, PartialEq, Format)]
pub enum SleepState {
    /// Device is fully awake.
    Awake,
    /// Deep sleep has been requested and peripherals are shutting down.
    Requested(SleepReason),
    /// Peripherals are powered down and deep sleep can be entered.
    Ready(SleepReason),
}

/// Peripheral that must acknowledge shutdown before deep sleep starts.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum SleepReadySource {
    Measurement,
    StatusLed,
}

/// Device state management
#[derive(Debug, Clone, PartialEq)]
pub struct DeviceState {
    /// Measurement status
    pub measurement_status: MeasurementTaskStatus,
    /// Start time of the measurement in microseconds
    pub start_time: u32,
    /// Calibration points (raw value, weight)
    pub calibration_points: [CalibrationPoint; MAX_CALIBRATION_POINTS],
    /// Number of calibration points currently stored
    pub calibration_point_count: usize,
    /// Battery voltage in millivolts
    pub battery_voltage: u32,
    /// Fuel gauge reading, `None` until the first successful read.
    pub battery_status: Option<BatteryStatus>,
    /// Timestamp of the last user-visible activity.
    pub last_activity_time_ms: u32,
    /// Whether BLE is currently connected.
    pub ble_connected: bool,
    /// Number of measurements to pack according to the negotiated ATT MTU.
    pub measurement_batch_size: usize,
    /// Current sleep transition state.
    pub sleep_state: SleepState,
    /// Whether the measurement task has powered down its peripherals for sleep.
    pub sleep_measurement_ready: bool,
    /// Whether the status LED has been explicitly turned off for sleep.
    pub sleep_status_led_ready: bool,
    /// Whether a low battery check has disabled the battery gauge's hibernate. The gauge stays
    /// powered from the battery, so deep sleep waits for the check to restore it.
    pub battery_gauge_check: bool,
    /// Settings in use. They apply to every client and the storage task persists them.
    pub settings: Settings,
    /// Whether the settings changed since the storage task last persisted them. Deep sleep waits
    /// for the write.
    pub settings_write_pending: bool,
}

impl Default for DeviceState {
    fn default() -> Self {
        Self::new()
    }
}

impl DeviceState {
    /// Create the state used after boot.
    pub const fn new() -> Self {
        Self {
            measurement_status: MeasurementTaskStatus::Disabled,
            start_time: 0,
            calibration_points: [(0.0, 0.0); MAX_CALIBRATION_POINTS],
            calibration_point_count: 0,
            battery_voltage: 4300,
            battery_status: None,
            last_activity_time_ms: 0,
            ble_connected: false,
            measurement_batch_size: measurement_batch_size_for_att_mtu(23),
            sleep_state: SleepState::Awake,
            sleep_measurement_ready: false,
            sleep_status_led_ready: false,
            battery_gauge_check: false,
            settings: Settings::DEFAULT,
            settings_write_pending: false,
        }
    }

    fn now_ms() -> u32 {
        (time::Instant::now().duration_since_epoch()).as_millis() as u32
    }

    fn cancel_idle_sleep_request(&mut self) {
        match self.sleep_state {
            SleepState::Requested(SleepReason::IdleTimeout)
            | SleepState::Ready(SleepReason::IdleTimeout) => {
                self.sleep_state = SleepState::Awake;
                self.sleep_measurement_ready = false;
                self.sleep_status_led_ready = false;
            }
            // Activity does not cancel a low battery sleep.
            SleepState::Requested(_) | SleepState::Ready(_) | SleepState::Awake => {}
        }
    }

    /// Record user-visible activity so idle sleep can be postponed.
    pub fn record_activity(&mut self) {
        self.cancel_idle_sleep_request();
        self.last_activity_time_ms = Self::now_ms();
    }

    /// Start a measurement
    pub fn start_measurement(&mut self) {
        self.record_activity();
        self.start_time = (time::Instant::now().duration_since_epoch()).as_micros() as u32;
        self.measurement_status = MeasurementTaskStatus::Enabled;
    }

    /// Stop the current measurement
    pub fn stop_measurement(&mut self) {
        self.record_activity();
        self.measurement_status = MeasurementTaskStatus::Disabled;
    }

    /// Queue a command for the measurement task.
    fn queue_load_cell_command(&mut self, command: LoadCellCommand) {
        self.record_activity();
        if LOAD_CELL_COMMANDS.try_send(command).is_err() {
            warn!("Load cell command queue full, dropping {:?}", command);
        }
    }

    /// Start taring process
    pub fn tare(&mut self) {
        self.queue_load_cell_command(LoadCellCommand::Tare);
    }

    /// Add a calibration point with the given weight
    pub fn calibrate(&mut self, weight: f32) {
        self.queue_load_cell_command(LoadCellCommand::Calibrate(weight));
    }

    /// Remove the calibration point at the given index
    pub fn remove_calibration_point(&mut self, index: u8) {
        self.queue_load_cell_command(LoadCellCommand::RemoveCalibrationPoint(index));
    }

    pub fn get_calibration(&mut self) {
        self.queue_load_cell_command(LoadCellCommand::GetCalibration);
    }

    /// Reset to default calibration
    pub fn reset_calibration(&mut self) {
        self.queue_load_cell_command(LoadCellCommand::DefaultCalibration);
    }

    /// Mark BLE as connected.
    pub fn on_ble_connected(&mut self) {
        self.ble_connected = true;
        self.record_activity();
        // A connection that arrives while a low battery sleep is ready must close before the
        // device sleeps. Activity already cancelled an idle sleep.
        if let SleepState::Ready(reason) = self.sleep_state {
            self.sleep_state = SleepState::Requested(reason);
        }
    }

    /// Mark BLE as disconnected and drop the measurement and commands of the closed connection.
    pub fn on_ble_disconnected(&mut self) {
        LOAD_CELL_COMMANDS.clear();
        self.ble_connected = false;
        self.measurement_status = MeasurementTaskStatus::Disabled;
        // Restart the idle timeout without cancelling a sleep request, which disconnects BLE.
        self.last_activity_time_ms = Self::now_ms();
        self.enter_sleep_when_ready();
    }

    /// Returns true when BLE is currently connected.
    pub fn is_ble_connected(&self) -> bool {
        self.ble_connected
    }

    /// Get elapsed inactivity time in milliseconds.
    pub fn get_inactivity_elapsed_ms(&self) -> u32 {
        Self::now_ms().wrapping_sub(self.last_activity_time_ms)
    }

    /// Request deep sleep. The measurement and LED tasks power down peripherals and the connection
    /// task disconnects BLE first.
    pub fn request_sleep(&mut self, reason: SleepReason) {
        if self.sleep_state == SleepState::Awake {
            LOAD_CELL_COMMANDS.clear();
            self.measurement_status = MeasurementTaskStatus::Disabled;
            self.sleep_measurement_ready = false;
            self.sleep_status_led_ready = false;
            self.sleep_state = SleepState::Requested(reason);
        }
    }

    /// Mark that one peripheral is powered down and deep sleep can start when all are ready.
    pub fn mark_sleep_ready(&mut self, source: SleepReadySource) {
        if let SleepState::Requested(_) = self.sleep_state {
            match source {
                SleepReadySource::Measurement => self.sleep_measurement_ready = true,
                SleepReadySource::StatusLed => self.sleep_status_led_ready = true,
            }
            self.enter_sleep_when_ready();
        }
    }

    /// Start a low battery check, which holds off deep sleep until it ends. Returns false once
    /// sleep is requested, as the device then sleeps without the check.
    pub fn begin_battery_gauge_check(&mut self) -> bool {
        if self.sleep_state != SleepState::Awake {
            return false;
        }
        self.battery_gauge_check = true;
        true
    }

    /// End the low battery check and allow a pending deep sleep.
    pub fn end_battery_gauge_check(&mut self) {
        self.battery_gauge_check = false;
        self.enter_sleep_when_ready();
    }

    /// Apply a sample rate now and persist it later.
    pub fn set_sample_rate(&mut self, sample_rate: SampleRate) {
        self.update_settings(Settings {
            sample_rate,
            ..self.settings
        });
    }

    /// Apply a calibration factor now and persist it later.
    pub fn set_calibration_factor(&mut self, calibration_factor: f32) {
        self.update_settings(Settings {
            calibration_factor,
            ..self.settings
        });
    }

    fn update_settings(&mut self, settings: Settings) {
        if settings != self.settings {
            self.settings = settings;
            self.settings_write_pending = true;
        }
    }

    /// End a settings write and allow a pending deep sleep, unless the settings changed during
    /// the write.
    pub fn end_settings_write(&mut self, written: Settings) {
        if written == self.settings {
            self.settings_write_pending = false;
            self.enter_sleep_when_ready();
        }
    }

    /// Allow deep sleep once the peripherals are powered down, BLE is disconnected and no low
    /// battery check or settings write is pending.
    fn enter_sleep_when_ready(&mut self) {
        if let SleepState::Requested(reason) = self.sleep_state
            && self.sleep_measurement_ready
            && self.sleep_status_led_ready
            && !self.ble_connected
            && !self.battery_gauge_check
            && !self.settings_write_pending
        {
            self.sleep_state = SleepState::Ready(reason);
        }
    }
}

/// Progressor Commands
#[derive(Debug, Clone, Copy, Format)]
/// Commands that Tindeq app can send to the device
// Source: Tindeq API documentation and https://github.com/blims/Tindeq-Progressor-API/blob/78a0bd244303589d0c773ee15ede53e0299712ee/progressor_client.py#L21-L33
pub enum ControlOpCode {
    /// Command used to zero weight when no load is applied
    TareScale = 0x64,
    /// Start continuous measurement at the configured firmware sample rate.
    StartMeasurement = 0x65,
    /// Stop weight measurement. This should be done before sampling the battery voltage
    StopMeasurement = 0x66,
    /// Start peak RFD measurement
    // TODO: Implement it
    StartPeakRFDMeasurement = 0x67,
    /// Start peak RFD measurement series
    // TODO: Implement it
    StartPeakRFDMeasurementSeries = 0x68,
    /// Adds a calibration point
    AddCalibrationPoint = 0x69,
    /// Save calibration
    // TODO: Implement it
    SaveCalibration = 0x6A,
    /// Get the error information
    // TODO: Implement it
    GetErrorInformation = 0x6C,
    /// Clear the error information
    // TODO: Implement it
    ClearErrorInformation = 0x6D,
    /// Close the BLE link. The device keeps advertising so another client can connect, and
    /// sleeps after the disconnected idle timeout.
    Shutdown = 0x6E,
    /// Measures the battery voltage in millivolts
    SampleBattery = 0x6F,
    /// Get the Progressor ID
    GetProgressorId = 0x70,
    /// Get the application version
    GetAppVersion = 0x6B,
    /// Get the calibration values
    // Custom command, no part of Tindeq API
    GetCalibration = 0x72,
    /// Default calibration
    // Custom command, no part of Tindeq API
    DefaultCalibration = 0x74,
    /// Set the ADS1220 sample rate (u16 LE samples per second), persisted once measurement stops,
    /// and reply with the rate in use, which stays unchanged for unsupported rates
    // Custom command, no part of Tindeq API
    SetSampleRate = 0x75,
    /// Reply with the ADS1220 sample rate in use
    // Custom command, no part of Tindeq API
    GetSampleRate = 0x76,
    /// Reply with the fuel gauge state of charge and whether the battery is charging
    // Custom command, no part of Tindeq API
    GetBatteryStatus = 0x77,
    /// Remove the calibration point at an index (u8) and refit the factor to the remaining points
    // Custom command, no part of Tindeq API
    RemoveCalibrationPoint = 0x78,
}

impl ControlOpCode {
    /// Returns whether this command should reset the idle timer. Status polls do not, so an open
    /// client still lets the device sleep.
    pub fn counts_as_activity(self) -> bool {
        !matches!(
            self,
            ControlOpCode::SampleBattery
                | ControlOpCode::GetBatteryStatus
                | ControlOpCode::GetSampleRate
        )
    }

    /// Process the control operation.
    ///
    /// Returns an immediate response packet when the command requires one.
    pub fn process(self, data: &[u8], device_state: &mut DeviceState) -> Option<DataPoint> {
        match self {
            ControlOpCode::TareScale => {
                device_state.tare();
                None
            }
            ControlOpCode::StartMeasurement => {
                device_state.start_measurement();
                None
            }
            ControlOpCode::StopMeasurement => {
                device_state.stop_measurement();
                None
            }
            ControlOpCode::GetAppVersion => {
                let response = ResponseCode::AppVersion(env!("DEVICE_VERSION_NUMBER").as_bytes());
                info!("AppVersion: {:#x}", response);
                Some(DataPoint::from(response))
            }
            ControlOpCode::GetProgressorId => {
                let response = ResponseCode::ProgressorId(DEVICE_ID);
                info!("ProgressorId: {:?}", response);
                Some(DataPoint::from(response))
            }
            ControlOpCode::GetCalibration => {
                info!("GetCalibration requested");
                device_state.get_calibration();
                None
            }
            ControlOpCode::AddCalibrationPoint => {
                if data.len() < 5 {
                    error!("AddCalibrationPoint: Invalid data length");
                    return None;
                }

                let weight = match data[1..5].try_into() {
                    Ok(bytes) => f32::from_le_bytes(bytes),
                    Err(e) => {
                        error!("Failed to parse calibration point data: {:?}", e);
                        return None;
                    }
                };

                if !weight.is_finite() || weight < 0.0 {
                    error!("AddCalibrationPoint: Invalid weight {}", weight);
                    return None;
                }

                device_state.calibrate(weight);
                info!(
                    "Received AddCalibrationPoint command with measurement: {}",
                    weight
                );
                None
            }
            ControlOpCode::DefaultCalibration => {
                device_state.reset_calibration();
                None
            }
            ControlOpCode::RemoveCalibrationPoint => {
                let Some(&index) = data.get(1) else {
                    error!("RemoveCalibrationPoint: Invalid data length");
                    return None;
                };
                info!(
                    "Received RemoveCalibrationPoint command for index {}",
                    index
                );
                device_state.remove_calibration_point(index);
                None
            }
            ControlOpCode::SampleBattery => {
                let voltage = device_state.battery_voltage;
                let response = ResponseCode::SampleBatteryVoltage(voltage);
                info!("SampleBattery: {:?}", response);
                Some(DataPoint::from(response))
            }
            ControlOpCode::Shutdown => {
                info!("Shutdown command received, disconnecting");
                None
            }
            ControlOpCode::SetSampleRate => {
                if data.len() < 3 {
                    error!("SetSampleRate: Invalid data length");
                    return None;
                }

                let sps = u16::from_le_bytes([data[1], data[2]]);
                match SampleRate::from_sps(sps) {
                    Some(sample_rate) => device_state.set_sample_rate(sample_rate),
                    None => warn!("SetSampleRate: Unsupported rate {} SPS", sps),
                }
                Some(DataPoint::from(ResponseCode::SampleRate(
                    device_state.settings.sample_rate,
                )))
            }
            ControlOpCode::GetSampleRate => Some(DataPoint::from(ResponseCode::SampleRate(
                device_state.settings.sample_rate,
            ))),
            ControlOpCode::GetBatteryStatus => match device_state.battery_status {
                Some(status) => Some(DataPoint::from(ResponseCode::BatteryStatus(status))),
                None => {
                    warn!("GetBatteryStatus: no fuel gauge reading yet");
                    None
                }
            },
            // Currently unimplemented operations
            ControlOpCode::StartPeakRFDMeasurement
            | ControlOpCode::StartPeakRFDMeasurementSeries
            | ControlOpCode::SaveCalibration
            | ControlOpCode::ClearErrorInformation
            | ControlOpCode::GetErrorInformation => None,
        }
    }
}

impl TryFrom<u8> for ControlOpCode {
    type Error = ();

    fn try_from(op_code: u8) -> Result<Self, Self::Error> {
        match op_code {
            0x64 => Ok(ControlOpCode::TareScale),
            0x65 => Ok(ControlOpCode::StartMeasurement),
            0x66 => Ok(ControlOpCode::StopMeasurement),
            0x69 => Ok(ControlOpCode::AddCalibrationPoint),
            0x6E => Ok(ControlOpCode::Shutdown),
            0x6F => Ok(ControlOpCode::SampleBattery),
            0x70 => Ok(ControlOpCode::GetProgressorId),
            0x6B => Ok(ControlOpCode::GetAppVersion),
            0x72 => Ok(ControlOpCode::GetCalibration),
            0x74 => Ok(ControlOpCode::DefaultCalibration),
            0x75 => Ok(ControlOpCode::SetSampleRate),
            0x76 => Ok(ControlOpCode::GetSampleRate),
            0x77 => Ok(ControlOpCode::GetBatteryStatus),
            0x78 => Ok(ControlOpCode::RemoveCalibrationPoint),
            0x6C => Ok(ControlOpCode::GetErrorInformation),
            0x6D => Ok(ControlOpCode::ClearErrorInformation),
            0x67 => Ok(ControlOpCode::StartPeakRFDMeasurement),
            0x68 => Ok(ControlOpCode::StartPeakRFDMeasurementSeries),
            0x6A => Ok(ControlOpCode::SaveCalibration),
            _ => Err(()),
        }
    }
}

/// Notification on the Data Point characteristic: response code, payload length and payload.
#[derive(Debug, Clone)]
pub struct DataPoint(ArrayVec<u8, { DATA_POINT_HEADER_SIZE + DATA_POINT_MAX_PAYLOAD_SIZE }>);

impl AsGatt for DataPoint {
    const MIN_SIZE: usize = DATA_POINT_HEADER_SIZE;
    const MAX_SIZE: usize = DATA_POINT_HEADER_SIZE + DATA_POINT_MAX_PAYLOAD_SIZE;

    fn as_gatt(&self) -> &[u8] {
        &self.0
    }
}

/// Initial characteristic value: response code 0 with an empty payload.
impl Default for DataPoint {
    fn default() -> Self {
        Self(ArrayVec::from_iter([0; DATA_POINT_HEADER_SIZE]))
    }
}

impl DataPoint {
    /// Create a weight measurement data point
    pub fn weight_measurement(measurements: WeightMeasurementBatch) -> Self {
        Self::from(ResponseCode::WeightMeasurement(measurements))
    }
}

impl Format for DataPoint {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "Code: {}, Length: {}, Data: {:x}",
            self.0[0],
            self.0[1],
            &self.0[DATA_POINT_HEADER_SIZE..]
        );
    }
}

impl From<ResponseCode> for DataPoint {
    fn from(response_code: ResponseCode) -> Self {
        let payload = response_code.payload();
        let mut bytes = ArrayVec::new();
        bytes.push(response_code.op_code());
        bytes.push(payload.len() as u8);
        bytes.extend(payload);
        Self(bytes)
    }
}

/// Data point response code
#[derive(Clone, Debug)]
#[repr(u8)]
/// Response codes that the device can send to the Tindeq app
// Source: Tindeq API documentation and https://github.com/blims/Tindeq-Progressor-API/blob/78a0bd244303589d0c773ee15ede53e0299712ee/progressor_client.py#L36-L40
pub enum ResponseCode {
    /// Response to battery voltage sampling command
    SampleBatteryVoltage(u32),
    /// Each measurement is sent together with a timestamp where the timestamp is the number of microseconds since the measurement was started
    WeightMeasurement(WeightMeasurementBatch),
    /// Calibration factor response
    CalibrationFactor(f32),
    /// Calibration point response (raw value, weight)
    CalibrationPoint(f32, f32),
    /// Sample rate in use, in samples per second
    SampleRate(SampleRate),
    /// Fuel gauge state of charge (u8 %) and charging flag (u8)
    BatteryStatus(BatteryStatus),
    /// Low power warning indicating that the battery is empty. The Progressor will turn itself off after sending this warning
    LowPowerWarning,
    /// Response to app version request command
    AppVersion(&'static [u8]),
    /// Response to progressor ID request command
    ProgressorId([u8; DEVICE_ID_SIZE]),
}

impl Format for ResponseCode {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            ResponseCode::SampleBatteryVoltage(voltage) => {
                defmt::write!(fmt, "SampleBatteryVoltage: {}", voltage)
            }
            ResponseCode::WeightMeasurement(measurements) => {
                defmt::write!(fmt, "WeightMeasurement: {} points", measurements.len())
            }
            ResponseCode::CalibrationFactor(factor) => {
                defmt::write!(fmt, "CalibrationFactor: {}", factor)
            }
            ResponseCode::CalibrationPoint(raw, weight) => {
                defmt::write!(fmt, "CalibrationPoint: Raw: {}, Weight: {}", raw, weight)
            }
            ResponseCode::SampleRate(rate) => defmt::write!(fmt, "SampleRate: {}", rate.sps()),
            ResponseCode::BatteryStatus(status) => defmt::write!(
                fmt,
                "BatteryStatus: {}%, charging: {}",
                status.soc_percent,
                status.charging
            ),
            ResponseCode::LowPowerWarning => defmt::write!(fmt, "LowPowerWarning"),
            ResponseCode::AppVersion(version) => defmt::write!(fmt, "AppVersion: {:x}", version),
            ResponseCode::ProgressorId(id) => defmt::write!(fmt, "ProgressorId: {:x}", id),
        }
    }
}

impl ResponseCode {
    /// Get the operation code for this response
    fn op_code(&self) -> u8 {
        match self {
            ResponseCode::SampleBatteryVoltage(..)
            | ResponseCode::AppVersion(..)
            | ResponseCode::ProgressorId(..) => 0x00,
            ResponseCode::WeightMeasurement(..) => 0x01,
            ResponseCode::LowPowerWarning => 0x04,
            ResponseCode::CalibrationFactor(..) => 0x05,
            ResponseCode::CalibrationPoint(..) => 0x06,
            ResponseCode::SampleRate(..) => 0x07,
            ResponseCode::BatteryStatus(..) => 0x08,
        }
    }

    /// Get the payload bytes for this response. Every payload fits the data point: a batch holds
    /// at most `MAX_SAMPLES_PER_PACKET` measurements and the app version is truncated.
    fn payload(&self) -> ArrayVec<u8, DATA_POINT_MAX_PAYLOAD_SIZE> {
        let mut payload = ArrayVec::new();
        match self {
            ResponseCode::SampleBatteryVoltage(voltage) => payload.extend(voltage.to_le_bytes()),
            ResponseCode::WeightMeasurement(measurements) => {
                for (weight, timestamp) in measurements {
                    payload.extend(weight.to_le_bytes());
                    payload.extend(timestamp.to_le_bytes());
                }
            }
            ResponseCode::CalibrationFactor(factor) => payload.extend(factor.to_le_bytes()),
            ResponseCode::CalibrationPoint(raw_value, weight) => {
                payload.extend(raw_value.to_le_bytes());
                payload.extend(weight.to_le_bytes());
            }
            ResponseCode::SampleRate(rate) => payload.extend(rate.sps().to_le_bytes()),
            ResponseCode::BatteryStatus(status) => {
                payload.extend([status.soc_percent, u8::from(status.charging)]);
            }
            ResponseCode::LowPowerWarning => (),
            // Reverse the bytes as they are LE
            ResponseCode::ProgressorId(id) => payload.extend(id.iter().rev().copied()),
            ResponseCode::AppVersion(version) => {
                payload.extend(version.iter().copied().take(DATA_POINT_MAX_PAYLOAD_SIZE));
            }
        }
        payload
    }
}
