/// Progressor data types
///
/// See [Tindeq API documentation] for more information
///
/// [Tindeq API documentation]: https://tindeq.com/progressor_api/
use defmt::{Format, error, info, trace, warn};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use esp_hal::time;
use trouble_host::types::gatt_traits::{AsGatt, FromGatt, FromGattError};

/// Size of the channel used to send data points
const DATA_POINT_COMMAND_CHANNEL_SIZE: usize = 80;
/// Channel used to send data points
pub type DataPointChannel = Channel<NoopRawMutex, DataPoint, DATA_POINT_COMMAND_CHANNEL_SIZE>;

/// Maximum size of the data payload in bytes for any data point
pub const MAX_PAYLOAD_SIZE: usize = 10;

/// Number of bytes in the device ID
const DEVICE_ID_SIZE: usize = 6;
/// Maximum number of calibration points to store
pub const MAX_CALIBRATION_POINTS: usize = 20;

/// Calibration point storing raw value and known weight
pub type CalibrationPoint = (f32, f32);

/// Status of the weight measurement task
#[derive(Copy, Debug, Clone, PartialEq)]
pub enum MeasurementTaskStatus {
    /// Measurements are enabled
    Enabled,
    /// Measurements are disabled
    Disabled,
    /// Device is in calibration mode with target weight
    Calibration(f32),
    /// Taring the scale (used in ClimbHarder App)
    Tare,
    /// Restores default calibration values
    DefaultCalibration,
    /// Get the calibration values
    GetCalibration,
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
    /// BLE disconnection time in milliseconds (None when connected)
    pub ble_disconnection_time: Option<u32>,
}

impl Default for DeviceState {
    fn default() -> Self {
        Self {
            measurement_status: MeasurementTaskStatus::Disabled,
            start_time: 0,
            calibration_points: [(0.0, 0.0); MAX_CALIBRATION_POINTS],
            calibration_point_count: 0,
            battery_voltage: 4300,
            ble_disconnection_time: None,
        }
    }
}

impl DeviceState {
    /// Start a measurement
    pub fn start_measurement(&mut self) {
        self.start_time = (time::Instant::now().duration_since_epoch()).as_micros() as u32;
        self.measurement_status = MeasurementTaskStatus::Enabled;
    }

    /// Stop the current measurement
    pub fn stop_measurement(&mut self) {
        self.measurement_status = MeasurementTaskStatus::Disabled;
    }

    /// Start taring process
    pub fn tare(&mut self) {
        self.measurement_status = MeasurementTaskStatus::Tare;
    }

    /// Set calibration mode with the given weight
    pub fn calibrate(&mut self, weight: f32) {
        self.measurement_status = MeasurementTaskStatus::Calibration(weight);
    }

    pub fn get_calibration(&mut self) {
        self.measurement_status = MeasurementTaskStatus::GetCalibration;
    }

    /// Reset to default calibration
    pub fn reset_calibration(&mut self) {
        self.measurement_status = MeasurementTaskStatus::DefaultCalibration;
    }

    /// Mark BLE as connected (clear disconnection time)
    pub fn on_ble_connected(&mut self) {
        self.ble_disconnection_time = None;
    }

    /// Mark BLE as disconnected (record current time)
    pub fn on_ble_disconnected(&mut self) {
        self.ble_disconnection_time =
            Some((time::Instant::now().duration_since_epoch()).as_millis() as u32);
    }

    /// Get elapsed time since BLE disconnection in milliseconds
    /// Returns None if BLE is currently connected
    pub fn get_ble_disconnection_elapsed_ms(&self) -> Option<u32> {
        self.ble_disconnection_time.map(|disconnect_time| {
            let current_time = (time::Instant::now().duration_since_epoch()).as_millis() as u32;
            current_time.saturating_sub(disconnect_time)
        })
    }
}

/// Progressor Commands
#[derive(Debug, Clone, Copy)]
/// Commands that Tindeq app can send to the device
// Source: Tindeq API documentation and https://github.com/blims/Tindeq-Progressor-API/blob/78a0bd244303589d0c773ee15ede53e0299712ee/progressor_client.py#L21-L33
pub enum ControlOpCode {
    /// Command used to zero weight when no load is applied
    TareScale = 0x64,
    /// Start continuous measurement. Sample rate is 80Hz
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
    /// Turn the Progressor off (enter sleep mode)
    // TODO: Implement it
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
}

impl ControlOpCode {
    /// Process the control operation
    pub fn process(
        self,
        data: &[u8],
        channel: &'static DataPointChannel,
        device_state: &mut DeviceState,
    ) {
        match self {
            ControlOpCode::TareScale => {
                device_state.tare();
            }
            ControlOpCode::StartMeasurement => {
                device_state.start_measurement();
            }
            ControlOpCode::StopMeasurement => {
                device_state.stop_measurement();
            }
            ControlOpCode::GetAppVersion => {
                let response = ResponseCode::AppVersion(env!("DEVICE_VERSION_NUMBER").as_bytes());
                info!("AppVersion: {:#x}", response);
                DataPoint::from(response).send(channel);
            }
            ControlOpCode::GetProgressorId => {
                /// Number of hex characters needed per byte (2 hex chars = 1 byte)
                const HEX_CHARS_PER_BYTE: usize = 2;
                /// Hex radix for parsing hex strings
                const HEX_RADIX: u32 = 16;

                let device_id = env!("DEVICE_ID");
                let mut bytes = [0u8; DEVICE_ID_SIZE];
                for (i, byte) in bytes.iter_mut().enumerate() {
                    let char_pos = i * HEX_CHARS_PER_BYTE;
                    let next_char_pos = char_pos + HEX_CHARS_PER_BYTE;
                    if next_char_pos <= device_id.len()
                        && let Ok(parsed_byte) =
                            u8::from_str_radix(&device_id[char_pos..next_char_pos], HEX_RADIX)
                    {
                        *byte = parsed_byte;
                    }
                }
                let response = ResponseCode::ProgressorId(bytes);
                info!("ProgressorId: {:?}", response);
                DataPoint::from(response).send(channel);
            }
            ControlOpCode::GetCalibration => {
                info!("GetCalibration requested");
                device_state.get_calibration();
            }
            ControlOpCode::AddCalibrationPoint => {
                if data.len() < 5 {
                    error!("AddCalibrationPoint: Invalid data length");
                    return;
                }

                let weight = match data[1..5].try_into() {
                    Ok(bytes) => f32::from_le_bytes(bytes),
                    Err(e) => {
                        error!("Failed to parse calibration point data: {:?}", e);
                        return;
                    }
                };

                if !weight.is_finite() || weight < 0.0 {
                    error!("AddCalibrationPoint: Invalid weight {}", weight);
                    return;
                }

                device_state.calibrate(weight);
                info!(
                    "Received AddCalibrationPoint command with measurement: {}",
                    weight
                );
            }
            ControlOpCode::DefaultCalibration => {
                device_state.reset_calibration();
            }
            ControlOpCode::SampleBattery => {
                let voltage = device_state.battery_voltage;
                let response = ResponseCode::SampleBatteryVoltage(voltage);
                info!("SampleBattery: {:?}", response);
                DataPoint::from(response).send(channel);
            }
            // Currently unimplemented operations
            ControlOpCode::Shutdown => {}
            ControlOpCode::StartPeakRFDMeasurement => {}
            ControlOpCode::StartPeakRFDMeasurementSeries => {}
            ControlOpCode::SaveCalibration => {}
            ControlOpCode::ClearErrorInformation => {}
            ControlOpCode::GetErrorInformation => {}
        }
    }
}

impl From<u8> for ControlOpCode {
    fn from(op_code: u8) -> Self {
        match op_code {
            0x64 => ControlOpCode::TareScale,
            0x65 => ControlOpCode::StartMeasurement,
            0x66 => ControlOpCode::StopMeasurement,
            0x69 => ControlOpCode::AddCalibrationPoint,
            0x6E => ControlOpCode::Shutdown,
            0x6F => ControlOpCode::SampleBattery,
            0x70 => ControlOpCode::GetProgressorId,
            0x6B => ControlOpCode::GetAppVersion,
            0x72 => ControlOpCode::GetCalibration,
            0x74 => ControlOpCode::DefaultCalibration,
            0x6C => ControlOpCode::GetErrorInformation,
            0x6D => ControlOpCode::ClearErrorInformation,
            0x67 => ControlOpCode::StartPeakRFDMeasurement,
            0x68 => ControlOpCode::StartPeakRFDMeasurementSeries,
            0x6A => ControlOpCode::SaveCalibration,
            _ => {
                error!("Invalid OpCode received: {:#x}", op_code);
                ControlOpCode::StopMeasurement
            }
        }
    }
}

impl Format for ControlOpCode {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            ControlOpCode::TareScale => defmt::write!(fmt, "TareScale"),
            ControlOpCode::StartMeasurement => defmt::write!(fmt, "StartMeasurement"),
            ControlOpCode::StopMeasurement => defmt::write!(fmt, "StopMeasurement"),
            ControlOpCode::GetAppVersion => defmt::write!(fmt, "GetAppVersion"),
            ControlOpCode::Shutdown => defmt::write!(fmt, "Shutdown"),
            ControlOpCode::SampleBattery => defmt::write!(fmt, "SampleBattery"),
            ControlOpCode::GetProgressorId => defmt::write!(fmt, "GetProgressorId"),
            ControlOpCode::GetCalibration => defmt::write!(fmt, "GetCalibration"),
            ControlOpCode::AddCalibrationPoint => defmt::write!(fmt, "AddCalibrationPoint"),
            ControlOpCode::DefaultCalibration => defmt::write!(fmt, "DefaultCalibration"),
            ControlOpCode::StartPeakRFDMeasurement => defmt::write!(fmt, "StartPeakRFDMeasurement"),
            ControlOpCode::StartPeakRFDMeasurementSeries => {
                defmt::write!(fmt, "StartPeakRFDMeasurementSeries")
            }
            ControlOpCode::SaveCalibration => defmt::write!(fmt, "SaveCalibration"),
            ControlOpCode::GetErrorInformation => defmt::write!(fmt, "GetErrorInformation"),
            ControlOpCode::ClearErrorInformation => defmt::write!(fmt, "ClearErrorInformation"),
        }
    }
}

/// Data point characteristic is where we receive data from the Progressor
#[derive(Copy, Debug, Clone)]
#[repr(C, packed)]
pub struct DataPoint {
    /// Response code
    pub(crate) response_code: u8,
    /// Length of the data
    pub(crate) length: u8,
    /// Data
    pub(crate) value: [u8; MAX_PAYLOAD_SIZE],
}

impl AsGatt for DataPoint {
    const MIN_SIZE: usize = 2;
    const MAX_SIZE: usize = MAX_PAYLOAD_SIZE + 2; // +2 for response_code and length

    fn as_gatt(&self) -> &[u8] {
        let len = (self.length as usize).min(MAX_PAYLOAD_SIZE);
        unsafe { core::slice::from_raw_parts(self as *const DataPoint as *const u8, 2 + len) }
    }
}

impl FromGatt for DataPoint {
    fn from_gatt(data: &[u8]) -> Result<Self, FromGattError> {
        if data.len() < 2 || data.len() > Self::MAX_SIZE {
            return Err(FromGattError::InvalidLength);
        }

        let response_code = data[0];
        let length = data[1] as usize;
        if length > MAX_PAYLOAD_SIZE || data.len() != 2 + length {
            return Err(FromGattError::InvalidLength);
        }

        Ok(DataPoint::new(response_code, data[1], &data[2..]))
    }
}

impl Default for DataPoint {
    fn default() -> Self {
        Self {
            response_code: 0,
            length: 0,
            value: [0; MAX_PAYLOAD_SIZE],
        }
    }
}

impl DataPoint {
    /// Create a new data point with specified response code, length and data
    pub fn new(response_code: u8, length: u8, data: &[u8]) -> Self {
        let mut value = [0; MAX_PAYLOAD_SIZE];
        let max_len = length.min(MAX_PAYLOAD_SIZE as u8) as usize;
        let copy_len = max_len.min(data.len());
        if copy_len > 0 {
            value[..copy_len].copy_from_slice(&data[..copy_len]);
        }

        Self {
            response_code,
            length: copy_len as u8,
            value,
        }
    }

    /// Send data point to the channel
    pub fn send(&self, channel: &'static DataPointChannel) {
        if channel.try_send(*self).is_err() {
            error!("Failed to send data point: channel full or receiver dropped");
        } else {
            trace!("Sent data point successfully");
        }
    }

    /// Create a weight measurement data point
    pub fn weight_measurement(weight: f32, timestamp: u32) -> Self {
        Self::from(ResponseCode::WeightMeasurement(weight, timestamp))
    }
}

impl Format for DataPoint {
    fn format(&self, fmt: defmt::Formatter) {
        let len = (self.length as usize).min(MAX_PAYLOAD_SIZE);
        defmt::write!(
            fmt,
            "Code: {}, Length: {}, Data: {:x}",
            self.response_code,
            self.length,
            &self.value[0..len]
        );
    }
}

impl From<ResponseCode> for DataPoint {
    fn from(response_code: ResponseCode) -> Self {
        Self {
            response_code: response_code.op_code(),
            length: response_code.length(),
            value: response_code.value(),
        }
    }
}

/// Data point response code
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
/// Response codes that the device can send to the Tindeq app
// Source: Tindeq API documentation and https://github.com/blims/Tindeq-Progressor-API/blob/78a0bd244303589d0c773ee15ede53e0299712ee/progressor_client.py#L36-L40
pub enum ResponseCode {
    /// Response to battery voltage sampling command
    SampleBatteryVoltage(u32),
    /// Each measurement is sent together with a timestamp where the timestamp is the number of microseconds since the measurement was started
    WeightMeasurement(f32, u32),
    /// Calibration factor response
    CalibrationFactor(f32),
    /// Calibration point response (raw value, weight)
    CalibrationPoint(f32, f32),
    /// Low power warning indicating that the battery is empty. The Progressor will turn itself off after sending this warning
    LowPowerWarning,
    /// Response to app version request command
    AppVersion(&'static [u8]),
    /// Response to progressor ID request command
    ProgressorId([u8; DEVICE_ID_SIZE]),
    /// RFD peak response.
    // TODO: Implement it
    RfdPeak,
    /// RFD peak series response.
    // TODO: Implement it
    RfdPeakSeries,
}

impl Format for ResponseCode {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            ResponseCode::SampleBatteryVoltage(voltage) => {
                defmt::write!(fmt, "SampleBatteryVoltage: {}", voltage)
            }
            ResponseCode::WeightMeasurement(weight, timestamp) => {
                defmt::write!(
                    fmt,
                    "WeightMeasurement: Weight: {}, Timestamp: {}",
                    weight,
                    timestamp
                )
            }
            ResponseCode::CalibrationFactor(factor) => {
                defmt::write!(fmt, "CalibrationFactor: {}", factor)
            }
            ResponseCode::CalibrationPoint(raw, weight) => {
                defmt::write!(fmt, "CalibrationPoint: Raw: {}, Weight: {}", raw, weight)
            }
            ResponseCode::LowPowerWarning => defmt::write!(fmt, "LowPowerWarning"),
            ResponseCode::AppVersion(version) => defmt::write!(fmt, "AppVersion: {:x}", version),
            ResponseCode::ProgressorId(id) => defmt::write!(fmt, "ProgressorId: {:x}", id),
            ResponseCode::RfdPeak => defmt::write!(fmt, "RfdPeak"),
            ResponseCode::RfdPeakSeries => defmt::write!(fmt, "RfdPeakSeries"),
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
            ResponseCode::RfdPeak => 0x02,
            ResponseCode::RfdPeakSeries => 0x03,
            ResponseCode::LowPowerWarning => 0x04,
            ResponseCode::CalibrationFactor(..) => 0x05,
            ResponseCode::CalibrationPoint(..) => 0x06,
        }
    }

    /// Get the length of the data for this response
    fn length(&self) -> u8 {
        match self {
            ResponseCode::SampleBatteryVoltage(..) => 4,
            ResponseCode::WeightMeasurement(..) => 8,
            ResponseCode::CalibrationFactor(..) => 4,
            ResponseCode::CalibrationPoint(..) => 8,
            ResponseCode::LowPowerWarning => 0,
            ResponseCode::AppVersion(version) => version.len().min(MAX_PAYLOAD_SIZE) as u8,
            ResponseCode::ProgressorId(..) => DEVICE_ID_SIZE as u8,
            ResponseCode::RfdPeak => 0,
            ResponseCode::RfdPeakSeries => 0,
        }
    }

    /// Get the value bytes for this response
    fn value(&self) -> [u8; MAX_PAYLOAD_SIZE] {
        let mut value = [0; MAX_PAYLOAD_SIZE];
        match self {
            ResponseCode::SampleBatteryVoltage(voltage) => {
                value[0..4].copy_from_slice(&voltage.to_le_bytes());
            }
            ResponseCode::WeightMeasurement(weight, timestamp) => {
                value[0..4].copy_from_slice(&weight.to_le_bytes());
                value[4..8].copy_from_slice(&timestamp.to_le_bytes());
            }
            ResponseCode::CalibrationFactor(factor) => {
                value[0..4].copy_from_slice(&factor.to_le_bytes());
            }
            ResponseCode::CalibrationPoint(raw_value, weight) => {
                value[0..4].copy_from_slice(&raw_value.to_le_bytes());
                value[4..8].copy_from_slice(&weight.to_le_bytes());
            }
            ResponseCode::LowPowerWarning => (),
            ResponseCode::ProgressorId(id) => {
                // Reverse the bytes as they are LE
                let mut reversed = *id;
                reversed.reverse();
                value[..DEVICE_ID_SIZE].copy_from_slice(&reversed);
            }
            ResponseCode::AppVersion(version) => {
                let len = version.len().min(MAX_PAYLOAD_SIZE);
                value[0..len].copy_from_slice(&version[0..len]);
            }
            ResponseCode::RfdPeak => {
                warn!("RfdPeak response not implemented");
            }
            ResponseCode::RfdPeakSeries => {
                warn!("RfdPeakSeries response not implemented");
            }
        };
        value
    }
}
