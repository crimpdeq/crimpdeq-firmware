/// Progressor data types
///
/// See [Tindeq API documentation] for more information
///
/// [Tindeq API documentation]: https://tindeq.com/progressor_api/
use arrayvec::ArrayVec;
use defmt::{Format, error, info, warn};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use esp_hal::time;
use trouble_host::types::gatt_traits::{AsGatt, FromGatt, FromGattError};

/// Size of the channel used to send data points
const DATA_POINT_COMMAND_CHANNEL_SIZE: usize = 80;
/// Max number of immediate responses produced by a single control command
const CONTROL_RESPONSES_MAX: usize = 2;
/// Channel used to send data points
pub type DataPointChannel = Channel<NoopRawMutex, DataPoint, DATA_POINT_COMMAND_CHANNEL_SIZE>;
/// Buffered immediate responses produced by control command processing
pub type ControlResponses = ArrayVec<DataPoint, CONTROL_RESPONSES_MAX>;

/// Maximum size of control-point payload in bytes.
pub const MAX_CONTROL_PAYLOAD_SIZE: usize = 10;
/// Number of weight samples batched in one BLE notification.
pub const WEIGHT_SAMPLES_PER_PACKET: usize = 16;
/// Bytes per packed weight sample (f32 weight + u32 timestamp).
const WEIGHT_SAMPLE_BYTES: usize = 8;
/// Maximum size of data-point payload in bytes.
pub const MAX_DATA_PAYLOAD_SIZE: usize = WEIGHT_SAMPLES_PER_PACKET * WEIGHT_SAMPLE_BYTES;

/// Number of bytes in the device ID
pub const DEVICE_ID_SIZE: usize = 6;
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
        Self::new()
    }
}

impl DeviceState {
    /// Create a default-initialized device state.
    pub const fn new() -> Self {
        Self {
            measurement_status: MeasurementTaskStatus::Disabled,
            start_time: 0,
            calibration_points: [(0.0, 0.0); MAX_CALIBRATION_POINTS],
            calibration_point_count: 0,
            battery_voltage: 4300,
            ble_disconnection_time: None,
        }
    }

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
    fn enqueue_response(responses: &mut ControlResponses, response: ResponseCode) {
        if responses.try_push(DataPoint::from(response)).is_err() {
            warn!("Dropping control response: response buffer full");
        }
    }

    /// Process the control operation
    pub fn process(
        self,
        data: &[u8],
        device_state: &mut DeviceState,
        device_id: [u8; DEVICE_ID_SIZE],
        responses: &mut ControlResponses,
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
                Self::enqueue_response(responses, response);
            }
            ControlOpCode::GetProgressorId => {
                let response = ResponseCode::ProgressorId(device_id);
                info!("ProgressorId: {:?}", response);
                Self::enqueue_response(responses, response);
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
                Self::enqueue_response(responses, response);
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
            0x6C => Ok(ControlOpCode::GetErrorInformation),
            0x6D => Ok(ControlOpCode::ClearErrorInformation),
            0x67 => Ok(ControlOpCode::StartPeakRFDMeasurement),
            0x68 => Ok(ControlOpCode::StartPeakRFDMeasurementSeries),
            0x6A => Ok(ControlOpCode::SaveCalibration),
            _ => Err(()),
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
    pub(crate) value: [u8; MAX_DATA_PAYLOAD_SIZE],
}

impl AsGatt for DataPoint {
    const MIN_SIZE: usize = 2;
    const MAX_SIZE: usize = MAX_DATA_PAYLOAD_SIZE + 2; // +2 for response_code and length

    fn as_gatt(&self) -> &[u8] {
        let len = (self.length as usize).min(MAX_DATA_PAYLOAD_SIZE);
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
        if length > MAX_DATA_PAYLOAD_SIZE || data.len() != 2 + length {
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
            value: [0; MAX_DATA_PAYLOAD_SIZE],
        }
    }
}

impl DataPoint {
    /// Create a new data point with specified response code, length and data
    pub fn new(response_code: u8, length: u8, data: &[u8]) -> Self {
        let mut value = [0; MAX_DATA_PAYLOAD_SIZE];
        let max_len = length.min(MAX_DATA_PAYLOAD_SIZE as u8) as usize;
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

    /// Create a weight measurement data point
    pub fn weight_measurement(weight: f32, timestamp: u32) -> Self {
        Self::from(ResponseCode::WeightMeasurement(weight, timestamp))
    }

    /// Create a batched weight measurement data point with up to 16 samples.
    pub fn weight_measurements(samples: &[(f32, u32)]) -> Self {
        const WEIGHT_MEASUREMENT_RESPONSE_CODE: u8 = 0x01;

        let sample_count = samples.len().min(WEIGHT_SAMPLES_PER_PACKET);
        let mut payload = [0u8; MAX_DATA_PAYLOAD_SIZE];
        for (i, (weight, timestamp)) in samples[..sample_count].iter().enumerate() {
            let offset = i * WEIGHT_SAMPLE_BYTES;
            payload[offset..offset + 4].copy_from_slice(&weight.to_le_bytes());
            payload[offset + 4..offset + WEIGHT_SAMPLE_BYTES]
                .copy_from_slice(&timestamp.to_le_bytes());
        }

        Self::new(
            WEIGHT_MEASUREMENT_RESPONSE_CODE,
            (sample_count * WEIGHT_SAMPLE_BYTES) as u8,
            &payload[..sample_count * WEIGHT_SAMPLE_BYTES],
        )
    }
}

impl Format for DataPoint {
    fn format(&self, fmt: defmt::Formatter) {
        let len = (self.length as usize).min(MAX_DATA_PAYLOAD_SIZE);
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
            ResponseCode::AppVersion(version) => version.len().min(MAX_DATA_PAYLOAD_SIZE) as u8,
            ResponseCode::ProgressorId(..) => DEVICE_ID_SIZE as u8,
            ResponseCode::RfdPeak => 0,
            ResponseCode::RfdPeakSeries => 0,
        }
    }

    /// Get the value bytes for this response
    fn value(&self) -> [u8; MAX_DATA_PAYLOAD_SIZE] {
        let mut value = [0; MAX_DATA_PAYLOAD_SIZE];
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
                let len = version.len().min(MAX_DATA_PAYLOAD_SIZE);
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
