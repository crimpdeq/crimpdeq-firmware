/// HX711 driver
///
/// A driver for the HX711 24-bit ADC commonly used with load cells.
/// This driver provides functions for reading data, calibration, and taring.
///
/// Based on [loadcell] crate.
///
/// [loadcell]: https://crates.io/crates/loadcell
use core::fmt;

use defmt::{debug, error, info};
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
/// The number of bits in the HX711 reading
const HX711_DATA_BITS: usize = 24;
/// The sign bit position in the HX711 reading
const HX711_SIGN_BIT: u32 = 0x800000;

/// The default address of the NVS flash storage.
const NVS_ADDR: u32 = 0x9000;
/// The default number of samples for taring
const DEFAULT_TARING_SAMPLES: usize = 16;
/// The default number of samples for calibration
const DEFAULT_CALIBRATION_SAMPLES: usize = 100;
/// The default calibration value.
const DEFAULT_CALIBRATION_FACTOR: f32 = 0.066;

/// Custom error type for HX711 operations
#[derive(Debug)]
pub enum Hx711Error {
    /// Flash storage error
    FlashError,
    /// Invalid calibration value
    InvalidCalibration,
}

impl fmt::Display for Hx711Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Hx711Error::FlashError => write!(f, "Flash storage error"),
            Hx711Error::InvalidCalibration => write!(f, "Invalid calibration value"),
        }
    }
}

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

/// HX711 24-bit ADC driver
pub struct Hx711<'d> {
    /// Data pin
    data: Input<'d>,
    /// Clock pin
    clock: Output<'d>,
    /// Delay instance
    delay: Delay,
    /// Flash storage
    flash: FlashStorage<'d>,
    /// Gain mode
    gain_mode: GainMode,
    /// Tare value
    tare_value: i32,
    /// Calibration
    calibration_factor: f32,
}

impl<'d> Hx711<'d> {
    /// Create a new HX711 driver.
    pub fn new(
        data: Input<'d>,
        mut clock: Output<'d>,
        delay: Delay,
        flash: FlashStorage<'d>,
    ) -> Self {
        info!("HX711 initialized");
        clock.set_low();

        let mut hx711 = Self {
            data,
            clock,
            delay,
            flash,
            gain_mode: GainMode::A64,
            tare_value: 0,
            calibration_factor: 0.0,
        };

        hx711.calibration_factor = hx711
            .get_calibration_factor()
            .unwrap_or(DEFAULT_CALIBRATION_FACTOR);

        hx711
    }

    /// Read calibration factor from flash
    fn read_from_flash(&mut self) -> Result<f32, Hx711Error> {
        let mut bytes = [0u8; 4];

        self.flash.read(NVS_ADDR, &mut bytes).map_err(|_| {
            error!("Failed to read calibration factor from flash");
            Hx711Error::FlashError
        })?;

        let factor = f32::from_le_bytes(bytes[0..4].try_into().unwrap());

        if !Self::is_valid_calibration_factor(factor) {
            info!("Invalid calibration factor read from flash");
            return Err(Hx711Error::InvalidCalibration);
        }

        Ok(factor)
    }

    /// Check if the calibration factor is valid
    pub fn is_valid_calibration_factor(factor: f32) -> bool {
        !factor.is_nan() && factor != 0.0
    }

    /// Write calibration factor to flash
    fn write_to_flash(&mut self, calibration_factor: f32) -> Result<(), Hx711Error> {
        if !Self::is_valid_calibration_factor(calibration_factor) {
            return Err(Hx711Error::InvalidCalibration);
        }

        let mut bytes = [0u8; 4];

        bytes[0..4].copy_from_slice(&calibration_factor.to_le_bytes());

        self.flash.write(NVS_ADDR, &bytes).map_err(|_| {
            error!("Failed to write calibration factor to flash");
            Hx711Error::FlashError
        })?;

        Ok(())
    }

    /// Update the calibration factor in memory and flash.
    pub fn update_calibration_factor(&mut self, factor: f32) -> Result<(), Hx711Error> {
        if !Self::is_valid_calibration_factor(factor) {
            error!("Invalid calibration factor: {}", factor);
            return Err(Hx711Error::InvalidCalibration);
        }

        debug!("Updating calibration factor: {}", factor);
        self.write_to_flash(factor)?;

        self.calibration_factor = factor;
        Ok(())
    }

    pub fn get_calibration_factor(&mut self) -> Result<f32, Hx711Error> {
        // Get the calibration factor from the NVS flash storage.
        match self.read_from_flash() {
            Ok(factor) => {
                info!("Read calibration factor: {:?}", factor);
                Ok(factor)
            }
            Err(Hx711Error::InvalidCalibration) => {
                info!("Using default calibration factor");
                Ok(DEFAULT_CALIBRATION_FACTOR)
            }
            Err(e) => Err(e),
        }
    }

    /// Get the current calibration factor.
    pub fn current_calibration_factor(&self) -> f32 {
        self.calibration_factor
    }

    /// Set the default calibration factor.
    pub fn default_calibration_factor(&mut self) -> Result<(), Hx711Error> {
        debug!("Restoring default calibration factor");
        self.write_to_flash(DEFAULT_CALIBRATION_FACTOR)?;
        self.calibration_factor = DEFAULT_CALIBRATION_FACTOR;
        Ok(())
    }

    /// Reads a single bit from the data pin.
    #[inline]
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
            let pulses = self.gain_mode as u8;
            for _ in 0..pulses {
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

    /// Gets the current gain mode.
    pub fn gain_mode(&self) -> GainMode {
        self.gain_mode
    }

    /// Reads 24 bits from the HX711 within a critical section.
    fn read_raw(&mut self) -> i32 {
        let value = critical_section::with(|_| {
            let mut result: u32 = 0;
            for _ in 0..HX711_DATA_BITS {
                result = (result << 1) | (self.read_data_bit() as u32);
            }
            result
        });

        self.send_gain_pulses();

        // Handle sign extension for 24-bit signed values
        let extended_value = if value & HX711_SIGN_BIT != 0 {
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

    /// Takes multiple samples and returns the average
    async fn take_samples(&mut self, num_samples: usize) -> f32 {
        let mut total: f32 = 0.0;

        for _ in 0..num_samples {
            self.wait_for_ready().await;
            total += self.read_raw() as f32;
        }

        total / num_samples as f32
    }

    /// Tares the sensor by measuring the average of several readings.
    pub async fn tare(&mut self) {
        debug!("Taring the scale");
        if !Self::is_valid_calibration_factor(self.calibration_factor) {
            info!("Invalid calibration factor, skipping tare");
            return;
        }

        let average = self.take_samples(DEFAULT_TARING_SAMPLES).await;
        self.tare_value = average as i32;
        debug!("Tare value set to: {}", self.tare_value);
    }

    /// Reads a raw value without calibration
    pub async fn read_raw_value(&mut self) -> i32 {
        self.wait_for_ready().await;
        self.read_raw()
    }

    /// Reads a tared raw value (raw value minus tare value)
    pub async fn read_tared(&mut self) -> i32 {
        self.wait_for_ready().await;
        self.read_raw() - self.tare_value
    }

    /// Reads a calibrated value, in kg.
    pub async fn read_calibrated(&mut self) -> f32 {
        let raw_tared = self.read_tared().await;
        let calibrated_value = raw_tared as f32 * self.calibration_factor;
        // Convert to kg
        calibrated_value / 1000.0
    }

    /// Perform two-point calibration with a known target weight
    ///
    /// This method collects raw values for calibration by taking multiple samples
    /// and averaging them for stability.
    ///
    /// Returns the average raw value for the calibration point.
    pub async fn perform_calibration(&mut self, _target_weight: f32) -> f32 {
        // Reset calibration to raw values first
        let _ = self.update_calibration_factor(1.0);

        // Take multiple readings and average them for stability
        let average_value = self.take_samples(DEFAULT_CALIBRATION_SAMPLES).await;
        debug!("Calibration point collected: {}", average_value);

        average_value
    }

    /// Apply two-point calibration using the collected calibration points
    ///
    /// This method calculates and applies calibration parameters based on
    /// two previously measured calibration points and a target weight.
    ///
    /// Returns true if calibration was successfully applied, false otherwise.
    pub fn apply_two_point_calibration(
        &mut self,
        calibration_points: [f32; 2],
        target_weight: f32,
    ) -> bool {
        debug!("Calibration points: {:?}", calibration_points);

        let (point1, point2) = (calibration_points[0], calibration_points[1]);

        // Check for invalid calibration points
        if (point2 - point1).abs() < f32::EPSILON {
            error!("Invalid calibration - points are too close together");
            return false;
        }

        if target_weight <= 0.0 {
            error!("Invalid target weight: {}", target_weight);
            return false;
        }

        // Calculate calibration factor (scale factor)
        let scale_factor = target_weight / (point2 - point1);

        // Apply the calibration factor
        match self.update_calibration_factor(scale_factor) {
            Ok(_) => {
                debug!("Calibration factor successfully applied");
                true
            }
            Err(e) => {
                error!(
                    "Failed to apply calibration factor: {:?}",
                    defmt::Debug2Format(&e)
                );
                false
            }
        }
    }
}
