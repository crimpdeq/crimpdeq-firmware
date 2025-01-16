pub const MAX_PAYLOAD_SIZE: usize = 12;

pub struct ControlPoint {
    op_code: OpCode,
    length: u8,
    // TODO: Update the length (n)
    value: [u8; 20],
}

/// Progressor Commands
pub enum OpCode {
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
}

/// Data point characteristic is where we receive data from the Progressor
pub struct DataPoint {
    /// Response code
    response_code: ResponseCode,
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
            response_code,
        }
    }

    /// Converts the DataPoint to a byte slice (`&[u8]`).
    pub fn as_bytes(&self) -> &[u8] {
        &self.value[..self.length as usize]
    }
}

impl From<ResponseCode> for DataPoint {
    fn from(response_code: ResponseCode) -> Self {
        Self {
            length: response_code.length(),
            value: response_code.value(),
            response_code,
        }
    }
}

#[repr(u8)]
/// Data point resposne code
pub enum ResponseCode {
    /// Response to [OpCode::SampleBattery] command
    SampleBatteryVoltage(u32) = 0x00,
    /// Each measurement is sent together with a timestam where the timestam is the number of microseconds since the measurement was started
    WeigthtMeasurement(f32, u32) = 0x01,
    /// Low power warning indicating that the battery is empty. The Progressor will turn itself off after sending this warning
    LowPowerWarning = 0x04,
}

impl ResponseCode {
    fn length(&self) -> u8 {
        match self {
            ResponseCode::SampleBatteryVoltage(..) => 4,
            ResponseCode::WeigthtMeasurement(..) => 8,
            ResponseCode::LowPowerWarning => 0,
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
        };
        value
    }
}
