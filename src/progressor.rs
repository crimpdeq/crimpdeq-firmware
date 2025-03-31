use core::cell::UnsafeCell;

/// Progressor data types
///
/// See [Tindeq API documentation] for more information
///
/// [Tindeq API documentation]: https://tindeq.com/progressor_api/
use arrayvec::ArrayVec;
use bytemuck_derive::{Pod, Zeroable};
use defmt::{debug, error, info, trace, Format};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use esp_hal::time;
use trouble_host::types::gatt_traits::{AsGatt, FromGatt, FromGattError};

use crate::hx711::Hx711;

/// Size of the channel used to send data points
const DATA_POINT_COMMAND_CHANNEL_SIZE: usize = 80;
/// Channel used to send data points
pub type DataPointChannel = Channel<NoopRawMutex, DataPoint, DATA_POINT_COMMAND_CHANNEL_SIZE>;

/// Maximum size of the data payload in bytes for any data point
pub const MAX_PAYLOAD_SIZE: usize = 10;

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
    /// Soft taring the scale - subtract the current weight (used in Tindeq App)
    SoftTare,
    /// Restores default calibration values
    DefaultCalibration,
}

/// Device state management
#[derive(Copy, Debug, Clone, PartialEq)]
pub struct DeviceState {
    /// Measurement status
    pub measurement_status: MeasurementTaskStatus,
    /// Tared status
    pub tared: bool,
    /// Start time of the measurement in microseconds
    pub start_time: u32,
    /// Calibration points [point1, point2]
    pub calibration_points: [f32; 2],
}

impl Default for DeviceState {
    fn default() -> Self {
        Self {
            measurement_status: MeasurementTaskStatus::Disabled,
            tared: false,
            start_time: 0,
            calibration_points: [-1.0, -1.0],
        }
    }
}

impl DeviceState {
    /// Create a new device state with default values
    pub fn new() -> Self {
        Self::default()
    }

    /// Start a measurement
    pub fn start_measurement(&mut self) {
        self.start_time = (time::Instant::now().duration_since_epoch()).as_micros() as u32;
        if self.tared {
            self.measurement_status = MeasurementTaskStatus::Enabled;
        } else {
            self.measurement_status = MeasurementTaskStatus::SoftTare;
        }
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

    /// Reset to default calibration
    pub fn reset_calibration(&mut self) {
        self.measurement_status = MeasurementTaskStatus::DefaultCalibration;
    }
}

/// Progressor Commands
#[derive(Debug, Clone, Copy)]
pub enum ControlOpCode {
    /// Command used to zero weight when no load is applied
    TareScale = 0x64,
    /// Start continuous measurement. Sample rate is 80Hz
    StartMeasurement = 0x65,
    /// Stop weight measurement. This should be done before sampling the battery voltage
    StopMeasurement = 0x66,
    /// Turn the Progressor off
    Shutdown = 0x6E,
    /// Measures the battery voltage in millivolts
    SampleBattery = 0x6F,
    /// Get the Progressor ID
    GetProgressorId = 0x70,
    /// Get the application version
    GetAppVersion = 0x6B,
    /// Get the calibration values
    GetCalibration = 0x72,
    /// Adds a calibration point
    AddCalibrationPoint = 0x73,
    /// Default calibration
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
                debug!("AppVersion: {:#x}", response);
                DataPoint::from(response).send(channel);
            }
            ControlOpCode::GetProgressorId => {
                let device_id = env!("DEVICE_ID");
                let mut id = 0;
                for (i, c) in device_id.chars().enumerate() {
                    id |= (c as u64) << (i * 8);
                }
                let response = ResponseCode::ProgressorId(id);
                debug!("ProgressorId: {:?}", response);
                DataPoint::from(response).send(channel);
            }
            ControlOpCode::GetCalibration => {
                info!("GetCalibration: {:?}", Hx711::get_calibration().unwrap());
            }
            ControlOpCode::AddCalibrationPoint => {
                if data.len() < 5 {
                    error!("AddCalibrationPoint: Invalid data length");
                    return;
                }

                let weight = match data[1..5].try_into() {
                    Ok(bytes) => f32::from_be_bytes(bytes),
                    Err(e) => {
                        error!("Failed to parse calibration point data: {:?}", e);
                        return;
                    }
                };

                device_state.calibrate(weight);
                debug!(
                    "Received AddCalibrationPoint command with measurement: {}",
                    weight
                );
            }
            ControlOpCode::DefaultCalibration => {
                device_state.reset_calibration();
            }
            ControlOpCode::SampleBattery => {
                // Hardcoded for now
                let voltage = 3300;
                let response = ResponseCode::SampleBatteryVoltage(voltage);
                debug!("SampleBattery: {:?}", response);
                DataPoint::from(response).send(channel);
            }
            // Currently unimplemented operations
            ControlOpCode::Shutdown => {}
        }
    }
}

impl From<u8> for ControlOpCode {
    fn from(op_code: u8) -> Self {
        match op_code {
            0x64 => ControlOpCode::TareScale,
            0x65 => ControlOpCode::StartMeasurement,
            0x66 => ControlOpCode::StopMeasurement,
            0x6E => ControlOpCode::Shutdown,
            0x6F => ControlOpCode::SampleBattery,
            0x70 => ControlOpCode::GetProgressorId,
            0x6B => ControlOpCode::GetAppVersion,
            0x72 => ControlOpCode::GetCalibration,
            0x73 => ControlOpCode::AddCalibrationPoint,
            0x74 => ControlOpCode::DefaultCalibration,
            _ => {
                error!("Invalid OpCode received: {:#x}", op_code);
                ControlOpCode::Shutdown
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
        }
    }
}

/// Data point characteristic is where we receive data from the Progressor
#[derive(Copy, Debug, Clone, Pod, Zeroable)]
#[repr(C, packed)]
pub struct DataPoint {
    /// Response code
    pub(crate) response_code: u8,
    /// Length of the data
    pub(crate) length: u8,
    /// Data
    pub(crate) value: [u8; MAX_PAYLOAD_SIZE],
}

// Thread-local buffer for preparing GATT data
struct SyncUnsafeCell<T>(UnsafeCell<T>);

unsafe impl<T> Sync for SyncUnsafeCell<T> {}

static GATT_BUFFER: SyncUnsafeCell<[u8; MAX_PAYLOAD_SIZE + 2]> =
    SyncUnsafeCell(UnsafeCell::new([0; MAX_PAYLOAD_SIZE + 2]));

impl AsGatt for DataPoint {
    const MIN_SIZE: usize = 3;
    const MAX_SIZE: usize = MAX_PAYLOAD_SIZE + 2; // +2 for response_code and length

    fn as_gatt(&self) -> &[u8] {
        // SAFETY: We're using an UnsafeCell to provide interior mutability.
        // This is safe as long as we ensure this function is not called concurrently from multiple threads.
        // In our embedded context with no preemptive threading, this should be fine.
        let buffer = unsafe { &mut *GATT_BUFFER.0.get() };

        // Populate the buffer with our data
        buffer[0] = self.response_code;
        buffer[1] = self.length;

        // Copy the value bytes
        if self.length > 0 {
            buffer[2..2 + self.length as usize]
                .copy_from_slice(&self.value[..self.length as usize]);
        }
        let result =
            unsafe { core::slice::from_raw_parts(buffer.as_ptr(), 2 + self.length as usize) };
        trace!("AsGatt: {:?}", result);
        result
    }
}

impl FromGatt for DataPoint {
    fn from_gatt(data: &[u8]) -> Result<Self, FromGattError> {
        Ok(DataPoint::new(data[0], data[1], &data[2..]))
    }
}

// // Implement FixedGattValue for DataPoint
// impl FixedGattValue for DataPoint {
//     const SIZE: usize = 10;

//     fn from_gatt(data: &[u8]) -> Result<Self, FromGattError> {
//         Ok(DataPoint::new(data[0], data[1], &data[2..]))
//     }

//     fn as_gatt(&self) -> &[u8] {
//         &self.value[..self.length as usize]
//     }
// }

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
        let len = length.min(MAX_PAYLOAD_SIZE as u8) as usize;
        if len > 0 && !data.is_empty() {
            value[..len.min(data.len())].copy_from_slice(&data[..len.min(data.len())]);
        }

        Self {
            response_code,
            length,
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
        defmt::write!(
            fmt,
            "Code: {}, Length: {}, Data: {:x}",
            self.response_code,
            self.length,
            &self.value[0..self.length as usize]
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
pub enum ResponseCode {
    /// Response to battery voltage sampling command
    SampleBatteryVoltage(u32),
    /// Each measurement is sent together with a timestamp where the timestamp is the number of microseconds since the measurement was started
    WeightMeasurement(f32, u32),
    /// Low power warning indicating that the battery is empty. The Progressor will turn itself off after sending this warning
    LowPowerWarning,
    /// Response to app version request command
    AppVersion(&'static [u8]),
    /// Response to progressor ID request command
    ProgressorId(u64),
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
            ResponseCode::LowPowerWarning => defmt::write!(fmt, "LowPowerWarning"),
            ResponseCode::AppVersion(version) => defmt::write!(fmt, "AppVersion: {:?}", version),
            ResponseCode::ProgressorId(id) => defmt::write!(fmt, "ProgressorId({})", id),
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
        }
    }

    /// Get the length of the data for this response
    fn length(&self) -> u8 {
        match self {
            ResponseCode::SampleBatteryVoltage(..) => 4,
            ResponseCode::WeightMeasurement(..) => 8,
            ResponseCode::LowPowerWarning => 0,
            ResponseCode::AppVersion(version) => version.len() as u8,
            ResponseCode::ProgressorId(..) => 6,
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
            ResponseCode::LowPowerWarning => (),
            ResponseCode::ProgressorId(id) => {
                let bytes = to_le_bytes_without_trailing_zeros(*id);
                value[0..bytes.len()].copy_from_slice(&bytes);
            }
            ResponseCode::AppVersion(version) => {
                value[0..version.len()].copy_from_slice(version);
            }
        };
        value
    }
}

/// Convert an integer into an array of bytes with any zeros on the MSB side trimmed
fn to_le_bytes_without_trailing_zeros<T: Into<u64>>(input: T) -> ArrayVec<u8, 8> {
    let input = input.into();
    if input == 0 {
        return ArrayVec::try_from([0_u8].as_slice()).unwrap();
    }

    let mut out: ArrayVec<u8, 8> = input
        .to_le_bytes()
        .into_iter()
        .rev()
        .skip_while(|&i| i == 0)
        .collect();
    out.reverse();
    out
}
