/// HX711 driver
///
/// Based on [loadcell] crate.
///
/// [loadcell]: https://crates.io/crates/loadcell
use defmt::info;
use embedded_hal::delay::DelayNs;
use esp_hal::{
    delay::Delay,
    gpio::{Input, Output},
};

/// Obtained calibration factor
const CALIBRATION_FACTOR: f32 = 0.06672;
/// Obtained calibration offset
const CALIBRATION_OFFSET: f32 = 52.8916;

/// The absolute minimum readings. A smaller value should be clamped.
const HX711_MINIMUM: i32 = -(2i32.saturating_pow(24 - 1));
/// The absolute maximum readings. A greater value should be clamped.
const HX711_MAXIMUM: i32 = 2i32.saturating_pow(24 - 1) - 1;
/// The default delay time in microseconds for the HX711.
const HX711_DELAY_TIME_US: u32 = 1;

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
struct Calibration {
    /// Calibration offset
    offset: f32,
    /// Calibration factor
    factor: f32,
}

/// A driver for the HX711 24-bit ADC.
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
            calibration: Calibration {
                offset: CALIBRATION_OFFSET,
                factor: CALIBRATION_FACTOR,
            },
        }
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

        for _ in 0..=TARING_SAMPLES {
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
        calibrated_value / 1000.0
    }
}
