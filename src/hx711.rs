/// HX711 driver
///
/// A driver for the HX711 24-bit ADC commonly used with load cells.
/// This driver provides functions for reading data, calibration, and taring.
///
/// Based on [loadcell] crate.
///
/// [loadcell]: https://crates.io/crates/loadcell
use core::{fmt, mem::size_of};

use defmt::{debug, error, info, warn};
use embassy_time::{Duration, Timer, with_timeout};
use embedded_hal::delay::DelayNs;
use embedded_storage::{ReadStorage, Storage};
use esp_bootloader_esp_idf::partitions::{self, PartitionType};
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
/// Timeout waiting for HX711 data-ready signal (DOUT low).
const HX711_READY_TIMEOUT_MS: u64 = 250;
/// Number of retries when waiting for HX711 data-ready signal.
const HX711_READY_MAX_RETRIES: usize = 3;
/// Delay between readiness retries.
const HX711_READY_RETRY_DELAY_MS: u64 = 5;
/// Minimum high pulse width on PD_SCK to enter power-down mode.
const HX711_POWER_DOWN_PULSE_US: u32 = 80;

/// Label of the dedicated data partition used to persist calibration data.
const CALIBRATION_PARTITION_LABEL: &str = env!("CALIBRATION_PARTITION_LABEL");
/// Number of bytes used to persist calibration factor.
const CALIBRATION_FACTOR_STORAGE_LEN: u32 = size_of::<f32>() as u32;
/// The default number of samples for taring
const DEFAULT_TARING_SAMPLES: usize = 16;
/// The default number of samples for calibration
const DEFAULT_CALIBRATION_SAMPLES: usize = 100;
/// The default calibration value.
const DEFAULT_CALIBRATION_FACTOR: f32 = 0.0639;

/// Custom error type for HX711 operations
#[derive(Debug)]
pub enum Hx711Error {
    /// Flash storage error
    FlashError,
    /// No valid calibration storage partition is available
    StorageUnavailable,
    /// Invalid calibration value
    InvalidCalibration,
    /// Timed out waiting for HX711 data-ready signal
    ReadyTimeout,
}

impl fmt::Display for Hx711Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Hx711Error::FlashError => write!(f, "Flash storage error"),
            Hx711Error::StorageUnavailable => write!(f, "Calibration storage unavailable"),
            Hx711Error::InvalidCalibration => write!(f, "Invalid calibration value"),
            Hx711Error::ReadyTimeout => write!(f, "Timed out waiting for HX711 data-ready"),
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
    /// Calibration factor storage offset resolved from partition table
    calibration_storage_offset: Option<u32>,
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
            calibration_storage_offset: None,
            calibration_factor: 0.0,
        };

        hx711.calibration_storage_offset = hx711.find_calibration_storage_offset();
        hx711.calibration_factor = hx711
            .get_calibration_factor()
            .unwrap_or(DEFAULT_CALIBRATION_FACTOR);

        hx711
    }

    /// Resolve calibration storage from the partition table to avoid raw flash offsets.
    fn find_calibration_storage_offset(&mut self) -> Option<u32> {
        let mut partition_table_buffer = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
        let partition_table =
            match partitions::read_partition_table(&mut self.flash, &mut partition_table_buffer) {
                Ok(table) => table,
                Err(_) => {
                    error!("Failed to read partition table for calibration storage");
                    return None;
                }
            };

        let Some(partition) = partition_table.iter().find(|entry| {
            entry.label_as_str() == CALIBRATION_PARTITION_LABEL
                && matches!(entry.partition_type(), PartitionType::Data(_))
        }) else {
            warn!(
                "Calibration partition '{}' not found; persistence disabled",
                CALIBRATION_PARTITION_LABEL
            );
            return None;
        };

        if partition.is_read_only() {
            warn!(
                "Calibration partition '{}' is read-only; persistence disabled",
                CALIBRATION_PARTITION_LABEL
            );
            return None;
        }

        if partition.len() < CALIBRATION_FACTOR_STORAGE_LEN {
            warn!(
                "Calibration partition '{}' too small ({} bytes); persistence disabled",
                CALIBRATION_PARTITION_LABEL,
                partition.len()
            );
            return None;
        }

        let storage_offset = partition.offset();
        info!(
            "Calibration storage resolved to partition '{}' at offset 0x{:x}",
            CALIBRATION_PARTITION_LABEL, storage_offset
        );
        Some(storage_offset)
    }

    fn read_factor_at(&mut self, offset: u32) -> Result<f32, Hx711Error> {
        let mut bytes = [0u8; 4];

        self.flash.read(offset, &mut bytes).map_err(|_| {
            error!("Failed to read calibration factor from flash");
            Hx711Error::FlashError
        })?;

        let factor = f32::from_le_bytes(bytes);

        if !Self::is_valid_calibration_factor(factor) {
            info!("Invalid calibration factor read from flash");
            return Err(Hx711Error::InvalidCalibration);
        }

        Ok(factor)
    }

    /// Read calibration factor from dedicated partition storage.
    fn read_from_flash(&mut self) -> Result<f32, Hx711Error> {
        let storage_offset = self
            .calibration_storage_offset
            .ok_or(Hx711Error::StorageUnavailable)?;
        self.read_factor_at(storage_offset)
    }

    /// Check if the calibration factor is valid
    pub fn is_valid_calibration_factor(factor: f32) -> bool {
        factor.is_finite() && factor != 0.0
    }

    /// Write calibration factor to flash
    fn write_to_flash(&mut self, calibration_factor: f32) -> Result<(), Hx711Error> {
        if !Self::is_valid_calibration_factor(calibration_factor) {
            return Err(Hx711Error::InvalidCalibration);
        }

        let Some(storage_offset) = self.calibration_storage_offset else {
            warn!("Calibration storage unavailable; using RAM-only calibration");
            return Ok(());
        };

        let bytes = calibration_factor.to_le_bytes();

        self.flash.write(storage_offset, &bytes).map_err(|_| {
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

        info!("Updating calibration factor: {}", factor);
        self.write_to_flash(factor)?;

        self.calibration_factor = factor;
        Ok(())
    }

    pub fn get_calibration_factor(&mut self) -> Result<f32, Hx711Error> {
        // Get calibration factor from resolved partition storage.
        match self.read_from_flash() {
            Ok(factor) => {
                info!("Calibration factor read from flash: {:?}", factor);
                Ok(factor)
            }
            Err(Hx711Error::InvalidCalibration) | Err(Hx711Error::StorageUnavailable) => {
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

    /// Put HX711 into low-power mode.
    ///
    /// The HX711 enters power-down when PD_SCK is held high for more than 60 µs.
    pub fn power_down(&mut self) {
        debug!("Powering down HX711");
        critical_section::with(|_| {
            self.clock.set_low();
            self.delay.delay_us(HX711_DELAY_TIME_US);
            self.clock.set_high();
            self.delay.delay_us(HX711_POWER_DOWN_PULSE_US);
        });
    }

    /// Wake HX711 from low-power mode.
    ///
    /// Subsequent reads will wait for DOUT-ready as usual.
    pub fn power_up(&mut self) {
        debug!("Powering up HX711");
        self.clock.set_low();
        self.delay.delay_us(HX711_DELAY_TIME_US);
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
    async fn wait_for_ready(&mut self) -> Result<(), Hx711Error> {
        for attempt in 1..=HX711_READY_MAX_RETRIES {
            if with_timeout(
                Duration::from_millis(HX711_READY_TIMEOUT_MS),
                self.data.wait_for_low(),
            )
            .await
            .is_ok()
            {
                return Ok(());
            }

            warn!(
                "HX711 not ready (attempt {}/{}), retrying",
                attempt, HX711_READY_MAX_RETRIES
            );
            Timer::after(Duration::from_millis(HX711_READY_RETRY_DELAY_MS)).await;
        }

        error!(
            "HX711 wait-for-ready timed out after {} attempts ({} ms each)",
            HX711_READY_MAX_RETRIES, HX711_READY_TIMEOUT_MS
        );
        Err(Hx711Error::ReadyTimeout)
    }

    /// Takes multiple samples and returns the average.
    async fn take_samples(&mut self, num_samples: usize) -> Result<f32, Hx711Error> {
        let mut total: f32 = 0.0;

        for _ in 0..num_samples {
            self.wait_for_ready().await?;
            total += self.read_raw() as f32;
        }

        Ok(total / num_samples as f32)
    }

    /// Tares the sensor by measuring the average of several readings.
    pub async fn tare(&mut self) -> Result<(), Hx711Error> {
        debug!("Taring the scale");
        if !Self::is_valid_calibration_factor(self.calibration_factor) {
            info!("Invalid calibration factor, skipping tare");
            return Ok(());
        }

        let average = self.take_samples(DEFAULT_TARING_SAMPLES).await?;
        self.tare_value = average as i32;
        debug!("Tare value set to: {}", self.tare_value);
        Ok(())
    }

    /// Reads a raw value without calibration.
    pub async fn read_raw_value(&mut self) -> Result<i32, Hx711Error> {
        self.wait_for_ready().await?;
        Ok(self.read_raw())
    }

    /// Reads a tared raw value (raw value minus tare value).
    pub async fn read_tared(&mut self) -> Result<i32, Hx711Error> {
        self.wait_for_ready().await?;
        Ok(self.read_raw() - self.tare_value)
    }

    /// Reads a calibrated value, in kg.
    pub async fn read_calibrated(&mut self) -> Result<f32, Hx711Error> {
        let raw_tared = self.read_tared().await?;
        let calibrated_value = (raw_tared as f32) * self.calibration_factor;
        // Convert to kg
        Ok(calibrated_value / 1000.0)
    }

    /// Perform two-point calibration with a known target weight.
    ///
    /// This method collects **raw** values for calibration by taking multiple
    /// samples and averaging them for stability.
    ///
    /// NOTE: This does not modify or persist the current calibration factor.
    /// Flash is only written once a final factor is computed and applied.
    ///
    /// Returns the average raw value for the calibration point.
    pub async fn perform_calibration(&mut self) -> Result<f32, Hx711Error> {
        // Take multiple readings and average them for stability.
        let average_value = self.take_samples(DEFAULT_CALIBRATION_SAMPLES).await?;
        debug!("Calibration point collected: {}", average_value);

        Ok(average_value)
    }

    /// Apply multi-point calibration using the collected calibration points.
    ///
    /// This method calculates and applies a best-fit calibration factor
    /// based on the provided (raw_value, weight) pairs.
    ///
    /// Returns true if calibration was successfully applied, false otherwise.
    pub fn apply_multi_point_calibration(&mut self, calibration_points: &[(f32, f32)]) -> bool {
        if calibration_points.len() < 2 {
            error!("Calibration requires at least two points");
            return false;
        }

        let mut valid_count = 0usize;
        let mut base_point: Option<(f32, f32)> = None;
        let mut sum_delta_raw_weight = 0.0;
        let mut sum_delta_raw_sq = 0.0;

        for (raw_value, weight) in calibration_points {
            if !raw_value.is_finite() || !weight.is_finite() || *weight < 0.0 {
                error!(
                    "Skipping invalid calibration point raw={}, weight={}",
                    raw_value, weight
                );
                continue;
            }

            valid_count += 1;
            if let Some((base_raw, base_weight)) = base_point {
                let delta_raw = raw_value - base_raw;
                // Incoming calibration weights are expressed in kg, while
                // calibration_factor operates on grams before read_calibrated()
                // converts back to kg.
                let delta_weight = (weight - base_weight) * 1000.0;
                sum_delta_raw_weight += delta_raw * delta_weight;
                sum_delta_raw_sq += delta_raw * delta_raw;
            } else {
                base_point = Some((*raw_value, *weight));
            }
        }

        if valid_count < 2 {
            error!("Calibration requires at least two valid points");
            return false;
        }

        if sum_delta_raw_sq.abs() < f32::EPSILON {
            error!("Invalid calibration - points are too close together");
            return false;
        }

        let scale_factor = sum_delta_raw_weight / sum_delta_raw_sq;
        match self.update_calibration_factor(scale_factor) {
            Ok(_) => {
                info!(
                    "Calibration factor successfully applied: {:?}",
                    scale_factor
                );
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
