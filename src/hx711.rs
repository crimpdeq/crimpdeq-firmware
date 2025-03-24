/// HX711 driver
///
/// Based on [loadcell] crate.
///
/// [loadcell]: https://crates.io/crates/loadcell
use defmt::{debug, info, Format};
use embedded_hal::delay::DelayNs;
use embedded_storage::{ReadStorage, Storage};
use esp_hal::{
    delay::Delay,
    gpio::{Input, Output},
};
use esp_storage::FlashStorage;

/// The absolute minimum readings. A smaller value should be clamped.
const HX711_MINIMUM: i32 = -(2i32.saturating_pow(24 - 1));
/// The absolute maximum readings. A greater value should be clamped.
const HX711_MAXIMUM: i32 = 2i32.saturating_pow(24 - 1) - 1;
/// The default delay time in microseconds for the HX711.
const HX711_DELAY_TIME_US: u32 = 1;

/// The address of the NVS flash storage.
const NVS_ADDR: u32 = 0x110000;

/// The HX711 has different amplifier gain settings.
/// The choice of gain settings is controlled by writing a fixed number of
/// extra pulses after a read.
#[repr(u8)]
#[derive(Clone, Copy, Debug)]
pub enum GainMode {
    /// Amplification gain of 128 on channel A.
    A128 = 1,
    /// Amplification gain of 32 on channel B.
    B32 = 2,
    /// Amplification gain of 64 on channel A.
    A64 = 3,
}

/// Calibration values
#[derive(Debug, Clone, Copy)]
pub struct Calibration {
    /// Calibration offset
    offset: f32,
    /// Calibration factor
    factor: f32,
}

impl Format for Calibration {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "Calibration {{
                    - Offset: {}
                    - Factor: {}
                }}",
            self.offset,
            self.factor
        );
    }
}

/// HX711 24-bit ADC driver
pub struct Hx711<'d> {
    /// Data pin
    data: Input<'d>,
    /// Clock pin
    clock: Output<'d>,
    /// Delay instance
    delay: Delay,
    /// Gain mode
    gain_mode: GainMode,
    /// Tare value
    tare_value: i32,
    /// Calibration
    calibration: Calibration,
}

impl<'d> Hx711<'d> {
    /// Create a new HX711 driver.
    pub fn new(data: Input<'d>, mut clock: Output<'d>, delay: Delay) -> Self {
        info!("HX711 initialized");
        clock.set_low();
        Self {
            data,
            clock,
            delay,
            gain_mode: GainMode::A64,
            tare_value: 0,
            calibration: Self::get_calibration(),
        }
    }

    /// Update the calibration values.
    pub fn update_calibration(&mut self, offset: f32, factor: f32) {
        debug!(
            "Updating calibration: offset: {}, factor: {}",
            offset, factor
        );

        let mut flash = FlashStorage::new();
        let mut bytes = [0u8; 8];
        bytes[0..4].copy_from_slice(&offset.to_le_bytes());
        bytes[4..8].copy_from_slice(&factor.to_le_bytes());
        flash.write(NVS_ADDR, &bytes).unwrap();

        self.calibration.offset = offset;
        self.calibration.factor = factor;
    }

    /// Get the calibration values from the NVS flash storage.
    pub fn get_calibration() -> Calibration {
        let mut flash = FlashStorage::new();
        let mut bytes = [0u8; 8];
        flash.read(NVS_ADDR, &mut bytes).unwrap();
        let offset = f32::from_le_bytes(bytes[0..4].try_into().unwrap());
        let factor = f32::from_le_bytes(bytes[4..8].try_into().unwrap());
        Calibration { offset, factor }
    }

    /// Reads a single bit from the data pin.
    fn read_data_bit(&mut self) -> bool {
        self.clock.set_high();
        self.delay.delay_us(HX711_DELAY_TIME_US);

        let bit = self.data.is_high();

        self.clock.set_low();
        self.delay.delay_us(HX711_DELAY_TIME_US);

        bit
    }

    /// Toggles the clock pin to prepare for the next gain mode.
    fn send_gain_pulses(&mut self) {
        critical_section::with(|_| {
            for _ in 0..(self.gain_mode as u8) {
                self.clock.set_high();
                self.delay.delay_us(HX711_DELAY_TIME_US);
                self.clock.set_low();
                self.delay.delay_us(HX711_DELAY_TIME_US);
            }
        });
    }

    /// Sets the gain mode for the next reading.
    pub fn set_gain_mode(&mut self, gain_mode: GainMode) {
        self.gain_mode = gain_mode;
    }

    /// Reads 24 bits from the HX711 within a critical section.
    fn read_raw(&mut self) -> i32 {
        let value = critical_section::with(|_| {
            let mut result: u32 = 0;
            for _ in 0..24 {
                let bit = self.read_data_bit() as u32;
                result = (result << 1) | bit;
            }
            result
        });

        self.send_gain_pulses();

        // Handle sign extension for 24-bit signed values
        let extended_value = if value & 0x800000 != 0 {
            value | 0xFF000000 // Negative value, extend the sign bit
        } else {
            value // Positive value, no change
        };

        // Clamp to valid range and return as signed 32-bit
        (extended_value as i32).clamp(HX711_MINIMUM, HX711_MAXIMUM)
    }

    /// Waits until the data is ready to be read.
    async fn wait_for_ready(&mut self) {
        self.data.wait_for_low().await;
    }

    /// Tares the sensor by measuring the average of `num_samples` readings.
    pub async fn tare(&mut self) {
        if self.calibration.offset == 0.0 && self.calibration.factor == 1.0 {
            info!("Calibration values not set, skipping tare");
            return;
        }

        const TARING_SAMPLES: usize = 16;
        let mut total: f32 = 0.0;

        for _ in 0..TARING_SAMPLES {
            self.wait_for_ready().await;
            total += self.read_raw() as f32;
        }
        let average = total / TARING_SAMPLES as f32;
        self.tare_value = average as i32;
    }

    /// Reads a calibrated value, in kg.
    pub async fn read_calibrated(&mut self) -> f32 {
        self.wait_for_ready().await;
        let raw_value = self.read_raw() - self.tare_value;
        let calibrated_value = raw_value as f32 * self.calibration.factor - self.calibration.offset;
        // Convert to kg
        calibrated_value / 1000.0
    }
}
