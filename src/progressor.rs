use arrayvec::ArrayVec;
use bytemuck_derive::{Pod, Zeroable};
use defmt::Format;

pub const MAX_PAYLOAD_SIZE: usize = 12;
pub(crate) type CalibrationCurve = [u8; 12];

// TODO: Can we avoid ArrayVec and rewrite this fn
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

/// Progressor Commands
pub enum ControlOpCode {
    /// Command used to zero weight when no load is applied
    TareScale = 0x64,
    /// Start continuous measurement. Sample rate is 80Hz
    StartMeasurement = 0x65,
    /// Stop weight measurement. This should be done before sampling the battery voltage
    StopMeasurement = 0x66,
    /// Turn the Progressor off
    Shutdown = 0x6E,
    /// Measures the battery voltage in milivolts
    SampleBattery = 0x6F,
    GetProgressorId = 0x70,
    GetAppVersion = 0x6B,
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
            _ => panic!("Invalid OpCode"),
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
        }
    }
}

#[derive(Copy, Debug, Clone, Pod, Zeroable)]
#[repr(C, packed)]
/// Data point characteristic is where we receive data from the Progressor
pub struct DataPoint {
    /// Response code
    response_code: u8,
    /// Length of the data
    length: u8,
    /// Data
    value: [u8; MAX_PAYLOAD_SIZE],
}

impl DataPoint {
    pub fn new(response_code: ResponseCode) -> Self {
        DataPoint {
            length: response_code.length(),
            value: response_code.value(),
            response_code: response_code.op_code(),
        }
    }
}

impl Format for DataPoint {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "Code: {:?}, Length: {}, Data: {:?}",
            self.response_code,
            self.length,
            &self.value[0..self.length as usize]
        );
    }
}

impl From<ResponseCode> for DataPoint {
    fn from(response_code: ResponseCode) -> Self {
        Self {
            length: response_code.length(),
            value: response_code.value(),
            response_code: response_code.op_code(),
        }
    }
}

#[derive(Copy, Clone, Debug)]
#[repr(u8)]
/// Data point resposne code
pub enum ResponseCode {
    /// Response to [OpCode::SampleBattery] command
    SampleBatteryVoltage(u32),
    /// Each measurement is sent together with a timestam where the timestam is the number of microseconds since the measurement was started
    WeigthtMeasurement(f32, u32),
    /// Low power warning indicating that the battery is empty. The Progressor will turn itself off after sending this warning
    LowPowerWarning,
    AppVersion(&'static [u8]),
    ProgressorId(u64),
    CalibrationCurve(CalibrationCurve),
}

impl Format for ResponseCode {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            ResponseCode::SampleBatteryVoltage(voltage) => {
                defmt::write!(fmt, "SampleBatteryVoltage: {}", voltage)
            }
            ResponseCode::WeigthtMeasurement(weigth, timestamp) => {
                defmt::write!(
                    fmt,
                    "WeigthtMeasurement: Weigth: {}, Timestamp: {}",
                    weigth,
                    timestamp
                )
            }
            ResponseCode::LowPowerWarning => defmt::write!(fmt, "LowPowerWarning"),
            ResponseCode::AppVersion(version) => defmt::write!(fmt, "AppVersion: {:?}", version),
            ResponseCode::ProgressorId(id) => defmt::write!(fmt, "ProgressorId({})", id),
            ResponseCode::CalibrationCurve(curve) => {
                defmt::write!(fmt, "CalibrationCurve: {:?}", curve)
            }
        }
    }
}

impl ResponseCode {
    fn op_code(&self) -> u8 {
        match self {
            ResponseCode::SampleBatteryVoltage(..)
            | ResponseCode::AppVersion(..)
            | ResponseCode::ProgressorId(..)
            | ResponseCode::CalibrationCurve(..) => 0x00,
            ResponseCode::WeigthtMeasurement(..) => 0x01,
            ResponseCode::LowPowerWarning => 0x04,
        }
    }

    fn length(&self) -> u8 {
        match self {
            ResponseCode::SampleBatteryVoltage(..) => 4,
            ResponseCode::WeigthtMeasurement(..) => 8,
            ResponseCode::LowPowerWarning => 0,
            ResponseCode::AppVersion(version) => version.len() as u8,
            ResponseCode::CalibrationCurve(curve) => curve.len() as u8,
            ResponseCode::ProgressorId(id) => to_le_bytes_without_trailing_zeros(*id).len() as u8,
        }
    }

    fn value(&self) -> [u8; MAX_PAYLOAD_SIZE] {
        let mut value = [0; MAX_PAYLOAD_SIZE];
        match self {
            ResponseCode::SampleBatteryVoltage(voltage) => {
                value[0..4].copy_from_slice(&voltage.to_le_bytes());
            }
            ResponseCode::WeigthtMeasurement(weight, timestamp) => {
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
            ResponseCode::CalibrationCurve(curve) => value = *curve,
        };
        value
    }
}
