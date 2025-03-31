/// HX711 driver
///
/// A driver for the HX711 24-bit ADC commonly used with load cells.
/// This driver provides functions for reading data, calibration, and taring.
///
/// Based on [loadcell] crate.
///
/// [loadcell]: https://crates.io/crates/loadcell
use core::fmt;

use defmt::{debug, error, info, Format};
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
/// The default calibration values.
const DEFAULT_CALIBRATION: Calibration = Calibration {
    offset: 0.0,
    factor: 0.066,
};

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
            "{{
                    - Offset: {}
                    - Factor: {}
                }}",
            self.offset,
            self.factor
        );
    }
}

impl Calibration {
    /// Check if the calibration values are valid
    pub fn is_valid(&self) -> bool {
        !self.offset.is_nan() && !self.factor.is_nan() && self.factor != 0.0
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
            calibration: Self::get_calibration().unwrap_or(DEFAULT_CALIBRATION),
        }
    }

    /// Read calibration values from flash
    fn read_from_flash() -> Result<Calibration, Hx711Error> {
        let mut flash = FlashStorage::new();
        let mut bytes = [0u8; 8];

        flash.read(NVS_ADDR, &mut bytes).map_err(|_| {
            error!("Failed to read calibration from flash");
            Hx711Error::FlashError
        })?;

        let offset = f32::from_le_bytes(bytes[0..4].try_into().unwrap());
        let factor = f32::from_le_bytes(bytes[4..8].try_into().unwrap());
        let calibration = Calibration { offset, factor };

        if !calibration.is_valid() {
            info!("Invalid calibration values read from flash");
            return Err(Hx711Error::InvalidCalibration);
        }

        Ok(calibration)
    }

    /// Write calibration values to flash
    fn write_to_flash(calibration: Calibration) -> Result<(), Hx711Error> {
        if !calibration.is_valid() {
            return Err(Hx711Error::InvalidCalibration);
        }

        let mut flash = FlashStorage::new();
        let mut bytes = [0u8; 8];

        bytes[0..4].copy_from_slice(&calibration.offset.to_le_bytes());
        bytes[4..8].copy_from_slice(&calibration.factor.to_le_bytes());

        flash.write(NVS_ADDR, &bytes).map_err(|_| {
            error!("Failed to write calibration to flash");
            Hx711Error::FlashError
        })?;

        Ok(())
    }

    /// Update the calibration values in memory and flash.
    pub fn update_calibration(&mut self, offset: f32, factor: f32) -> Result<(), Hx711Error> {
        let calibration = Calibration { offset, factor };

        if !calibration.is_valid() {
            error!(
                "Invalid calibration values: offset={}, factor={}",
                offset, factor
            );
            return Err(Hx711Error::InvalidCalibration);
        }

        debug!(
            "Updating calibration: offset: {}, factor: {}",
            offset, factor
        );
        Self::write_to_flash(calibration)?;

        self.calibration = calibration;
        Ok(())
    }

    pub fn get_calibration() -> Result<Calibration, Hx711Error> {
        // Get the calibration values from the NVS flash storage.
        match Self::read_from_flash() {
            Ok(calibration) => {
                info!("Read Calibration values: {:?}", calibration);
                Ok(calibration)
            }
            Err(Hx711Error::InvalidCalibration) => {
                info!("Using default calibration values");
                Ok(DEFAULT_CALIBRATION)
            }
            Err(e) => Err(e),
        }
    }

    /// Get the current calibration values.
    pub fn current_calibration(&self) -> Calibration {
        self.calibration
    }

    /// Set the default calibration values.
    pub fn default_calibration(&mut self) -> Result<(), Hx711Error> {
        debug!("Restoring default calibration");
        Self::write_to_flash(DEFAULT_CALIBRATION)?;
        self.calibration = DEFAULT_CALIBRATION;
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
        if !self.calibration.is_valid() {
            info!("Invalid calibration values, skipping tare");
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
        let calibrated_value = raw_tared as f32 * self.calibration.factor - self.calibration.offset;
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
        let _ = self.update_calibration(0.0, 1.0);

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

        // Calculate calibration parameters (convert weight to raw value range)
        let scale_factor = target_weight / (point2 - point1);
        let offset = scale_factor * point1;

        // Apply the calibration
        match self.update_calibration(offset, scale_factor) {
            Ok(_) => {
                debug!("Calibration successfully applied");
                true
            }
            Err(e) => {
                error!("Failed to apply calibration: {:?}", defmt::Debug2Format(&e));
                false
            }
        }
    }
}
