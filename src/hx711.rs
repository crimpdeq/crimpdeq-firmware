/// HX711 driver
///
/// Based on [loadcell] crate.
///
/// [loadcell]: https://crates.io/crates/loadcell
use defmt::info;
use embedded_hal::delay::DelayNs;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Input, Output};

/// The HX711 has different amplifier gain settings.
/// The choice of gain settings is controlled by writing a fixed number of
/// extra pulses after a read.
#[repr(u8)]
#[derive(Clone, Copy, Debug)]
pub enum GainMode {
    /// Amplification gain of 128 on channel A.
    A128 = 1, // extra pulses
    /// Amplification gain of 32 on channel B.
    B32 = 2,
    /// Amplification gain of 64 on channel A.
    A64 = 3,
}

/// The absolute minimum readings. A smaller value should be clamped.
const HX711_MINIMUM: i32 = -(2i32.saturating_pow(24 - 1));
/// The absolute maximum readings. A greater value should be clamped.
const HX711_MAXIMUM: i32 = 2i32.saturating_pow(24 - 1) - 1;
/// The default delay time in microseconds for the HX711.
const HX711_DELAY_TIME_US: u32 = 1;
/// The delay time in microseconds for the HX711 tare function.
const HX711_TARE_SLEEP_TIME_US: u32 = 10_000;

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
    offset: i32,
    /// Calibration value
    scale: f32,
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
            offset: 0,
            scale: 1.0,
        }
    }

    /// Returns true if the load cell amplifier has a value ready to be read.
    pub fn is_ready(&mut self) -> bool {
        self.data.is_low()
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
        for _ in 0..(self.gain_mode as u8) {
            self.clock.set_high();
            self.delay.delay_us(HX711_DELAY_TIME_US);
            self.clock.set_low();
            self.delay.delay_us(HX711_DELAY_TIME_US);
        }
    }

    /// Sets the gain mode for the next reading.
    pub fn set_gain_mode(&mut self, gain_mode: GainMode) {
        self.gain_mode = gain_mode;
    }

    /// Sets the offset value.
    pub fn set_offset(&mut self, offset: i32) {
        self.offset = offset;
    }

    /// Sets the calibration scale value.
    pub fn set_scale(&mut self, scale: f32) {
        self.scale = scale;
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
    pub async fn tare(&mut self, num_samples: usize) {
        let mut total: f32 = 0.0;

        for n in 1..=num_samples {
            self.wait_for_ready().await;
            let current = self.read_raw() as f32;
            total += (current - total) / n as f32;
            self.delay.delay_us(HX711_TARE_SLEEP_TIME_US);
        }

        self.offset = total as i32;
    }

    /// Reads a raw value from the HX711, subtracting the tare offset.
    pub fn read(&mut self) -> Option<i32> {
        if !self.is_ready() {
            return None;
        }

        Some(self.read_raw() - self.offset)
    }

    /// Reads a scaled value from the HX711.
    pub fn read_scaled(&mut self) -> Option<f32> {
        self.read().map(|raw| raw as f32 * self.scale)
    }

    /// Get the average of 20 readings in kgs.
    // TODO: Improve this function
    pub async fn get_measurement(&mut self) -> f32 {
        self.wait_for_ready().await;

        let mut weight = 0.0;
        for _ in 0..20 {
            if let Some(x) = self.read_scaled() {
                // Don't take absolute value - keep the sign
                weight += x;
            }
        }
        // Divide by 20000.0 to convert to kg
        weight / 20000.0
    }
}
