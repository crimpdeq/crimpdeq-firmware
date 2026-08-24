//! ADS1220 load-cell ADC driver.
//!
//! The revision-3 PCB connects the ADS1220 over SPI and uses the bridge
//! excitation voltage as the ADC reference. The dedicated DRDY pin is not
//! connected, so reads use the RDATA command at the protocol's nominal 80 Hz
//! sample cadence.

use core::fmt;

use defmt::{debug, error, info};
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::spi::SpiBus;
use embedded_storage::{ReadStorage, Storage};
use esp_hal::{Async, gpio::Output, spi::master::Spi};
use esp_storage::FlashStorage;

const COMMAND_POWERDOWN: u8 = 0x02;
const COMMAND_RESET: u8 = 0x06;
const COMMAND_START_SYNC: u8 = 0x08;
const COMMAND_RDATA: u8 = 0x10;
const COMMAND_RREG: u8 = 0x20;
const COMMAND_WREG: u8 = 0x40;

const REGISTER_COUNT: usize = 4;
const CONFIGURATION_REGISTERS: [u8; REGISTER_COUNT] = [
    0x0E, // AIN0-AIN1, gain 128, PGA enabled
    0x44, // 90 SPS, normal mode, continuous conversion
    0x40, // External reference on REFP0/REFN0
    0x00, // IDACs disabled; dedicated DRDY mode
];

const RESET_DELAY: Duration = Duration::from_micros(100);
const SAMPLE_INTERVAL: Duration = Duration::from_micros(12_500);

/// Magic value used to validate stored calibration data.
const CALIBRATION_STORAGE_MAGIC: u32 = 0x4344_5146;
/// Version 2 invalidates calibration factors saved for the old HX711 front end.
const CALIBRATION_STORAGE_VERSION: u32 = 2;
const CALIBRATION_STORAGE_SIZE: usize = 16;
const DEFAULT_TARING_SAMPLES: usize = 16;
const DEFAULT_CALIBRATION_SAMPLES: usize = 100;
const DEFAULT_CALIBRATION_FACTOR: f32 = 0.08;

/// Errors produced by the ADS1220 load-cell driver.
#[derive(Debug)]
pub enum Ads1220Error {
    /// Flash storage access failed.
    FlashError,
    /// A calibration value is absent or invalid.
    InvalidCalibration,
    /// SPI communication failed.
    Spi,
    /// ADS1220 configuration register readback did not match.
    ConfigurationMismatch {
        expected: [u8; REGISTER_COUNT],
        actual: [u8; REGISTER_COUNT],
    },
}

impl fmt::Display for Ads1220Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Ads1220Error::FlashError => write!(f, "Flash storage error"),
            Ads1220Error::InvalidCalibration => write!(f, "Invalid calibration value"),
            Ads1220Error::Spi => write!(f, "ADS1220 SPI error"),
            Ads1220Error::ConfigurationMismatch { .. } => {
                write!(f, "ADS1220 configuration readback mismatch")
            }
        }
    }
}

/// ADS1220 load-cell front end.
pub struct Ads1220<'d> {
    spi: Spi<'d, Async>,
    chip_select: Output<'d>,
    flash: FlashStorage<'d>,
    tare_value: i32,
    calibration_factor: f32,
    next_sample_at: Instant,
}

impl<'d> Ads1220<'d> {
    /// Create a driver. Call [`Self::initialize`] before taking measurements.
    pub fn new(spi: Spi<'d, Async>, mut chip_select: Output<'d>, flash: FlashStorage<'d>) -> Self {
        chip_select.set_high();

        let mut ads1220 = Self {
            spi,
            chip_select,
            flash,
            tare_value: 0,
            calibration_factor: 0.0,
            next_sample_at: Instant::now() + SAMPLE_INTERVAL,
        };

        ads1220.calibration_factor = ads1220
            .get_calibration_factor()
            .unwrap_or(DEFAULT_CALIBRATION_FACTOR);
        ads1220
    }

    /// Reset and configure the ADC for the revision-3 ratiometric load cell.
    pub async fn initialize(&mut self) -> Result<(), Ads1220Error> {
        self.send_command(COMMAND_RESET).await?;
        Timer::after(RESET_DELAY).await;

        self.write_configuration(CONFIGURATION_REGISTERS).await?;
        let actual = self.read_configuration().await?;
        if actual != CONFIGURATION_REGISTERS {
            return Err(Ads1220Error::ConfigurationMismatch {
                expected: CONFIGURATION_REGISTERS,
                actual,
            });
        }

        self.send_command(COMMAND_START_SYNC).await?;
        self.reset_sample_cadence();
        info!("ADS1220 initialized at 90 SPS with 80 Hz read cadence");
        Ok(())
    }

    fn reset_sample_cadence(&mut self) {
        self.next_sample_at = Instant::now() + SAMPLE_INTERVAL;
    }

    async fn send_command(&mut self, command: u8) -> Result<(), Ads1220Error> {
        self.chip_select.set_low();
        let result = SpiBus::write(&mut self.spi, &[command]).await;
        self.chip_select.set_high();
        result.map_err(|_| Ads1220Error::Spi)
    }

    async fn write_configuration(
        &mut self,
        registers: [u8; REGISTER_COUNT],
    ) -> Result<(), Ads1220Error> {
        let command = COMMAND_WREG | (REGISTER_COUNT as u8 - 1);
        let bytes = [
            command,
            registers[0],
            registers[1],
            registers[2],
            registers[3],
        ];

        self.chip_select.set_low();
        let result = SpiBus::write(&mut self.spi, &bytes).await;
        self.chip_select.set_high();
        result.map_err(|_| Ads1220Error::Spi)
    }

    async fn read_configuration(&mut self) -> Result<[u8; REGISTER_COUNT], Ads1220Error> {
        let command = COMMAND_RREG | (REGISTER_COUNT as u8 - 1);
        let mut registers = [0; REGISTER_COUNT];

        self.chip_select.set_low();
        let result = async {
            SpiBus::write(&mut self.spi, &[command])
                .await
                .map_err(|_| Ads1220Error::Spi)?;
            SpiBus::read(&mut self.spi, &mut registers)
                .await
                .map_err(|_| Ads1220Error::Spi)
        }
        .await;
        self.chip_select.set_high();
        result?;

        Ok(registers)
    }

    async fn wait_for_sample_cadence(&mut self) {
        let now = Instant::now();
        if self.next_sample_at <= now {
            self.next_sample_at = now + SAMPLE_INTERVAL;
        }

        Timer::at(self.next_sample_at).await;
        self.next_sample_at += SAMPLE_INTERVAL;
    }

    async fn read_raw(&mut self) -> Result<i32, Ads1220Error> {
        self.wait_for_sample_cadence().await;

        let mut bytes = [0; 3];
        self.chip_select.set_low();
        let result = async {
            SpiBus::write(&mut self.spi, &[COMMAND_RDATA])
                .await
                .map_err(|_| Ads1220Error::Spi)?;
            SpiBus::read(&mut self.spi, &mut bytes)
                .await
                .map_err(|_| Ads1220Error::Spi)
        }
        .await;
        self.chip_select.set_high();
        result?;

        let raw = u32::from_be_bytes([0, bytes[0], bytes[1], bytes[2]]);
        Ok(if raw & 0x0080_0000 != 0 {
            (raw | 0xFF00_0000) as i32
        } else {
            raw as i32
        })
    }

    async fn take_samples(&mut self, num_samples: usize) -> Result<f32, Ads1220Error> {
        let mut total = 0.0;
        for _ in 0..num_samples {
            total += self.read_raw().await? as f32;
        }
        Ok(total / num_samples as f32)
    }

    /// Put the ADC into its low-power mode after the current conversion.
    pub async fn power_down(&mut self) -> Result<(), Ads1220Error> {
        debug!("Powering down ADS1220");
        self.send_command(COMMAND_POWERDOWN).await
    }

    /// Wake the ADC and restart continuous conversions.
    pub async fn power_up(&mut self) -> Result<(), Ads1220Error> {
        debug!("Powering up ADS1220");
        self.send_command(COMMAND_START_SYNC).await?;
        self.reset_sample_cadence();
        Ok(())
    }

    /// Tare the load cell with an average of multiple readings.
    pub async fn tare(&mut self) -> Result<(), Ads1220Error> {
        debug!("Taring the scale");
        let average = self.take_samples(DEFAULT_TARING_SAMPLES).await?;
        self.tare_value = average as i32;
        debug!("Tare value set to: {}", self.tare_value);
        Ok(())
    }

    /// Read an uncalibrated ADC code.
    pub async fn read_raw_value(&mut self) -> Result<i32, Ads1220Error> {
        self.read_raw().await
    }

    /// Read an ADC code with the tare offset removed.
    pub async fn read_tared(&mut self) -> Result<i32, Ads1220Error> {
        Ok(self.read_raw().await? - self.tare_value)
    }

    /// Read a calibrated load-cell value in kilograms.
    pub async fn read_calibrated(&mut self) -> Result<f32, Ads1220Error> {
        let raw_tared = self.read_tared().await?;
        Ok((raw_tared as f32) * self.calibration_factor / 1000.0)
    }

    /// Collect an averaged raw calibration point.
    pub async fn perform_calibration(&mut self) -> Result<f32, Ads1220Error> {
        let average = self.take_samples(DEFAULT_CALIBRATION_SAMPLES).await?;
        debug!("Calibration point collected: {}", average);
        Ok(average)
    }

    fn calibration_storage_offset(&self) -> Result<u32, Ads1220Error> {
        let capacity = self.flash.capacity();
        let sector_size = FlashStorage::SECTOR_SIZE as usize;

        if capacity < sector_size {
            error!("Flash capacity too small for calibration storage");
            return Err(Ads1220Error::FlashError);
        }

        Ok((capacity - sector_size) as u32)
    }

    fn calibration_checksum(factor_bits: u32) -> u32 {
        CALIBRATION_STORAGE_MAGIC ^ CALIBRATION_STORAGE_VERSION ^ factor_bits ^ 0xA5A5_5A5A
    }

    fn read_from_flash(&mut self) -> Result<f32, Ads1220Error> {
        let mut bytes = [0; CALIBRATION_STORAGE_SIZE];
        let offset = self.calibration_storage_offset()?;

        self.flash.read(offset, &mut bytes).map_err(|_| {
            error!("Failed to read calibration factor from flash");
            Ads1220Error::FlashError
        })?;

        let magic = u32::from_le_bytes(bytes[0..4].try_into().unwrap());
        let version = u32::from_le_bytes(bytes[4..8].try_into().unwrap());
        let factor_bits = u32::from_le_bytes(bytes[8..12].try_into().unwrap());
        let checksum = u32::from_le_bytes(bytes[12..16].try_into().unwrap());

        if magic != CALIBRATION_STORAGE_MAGIC || version != CALIBRATION_STORAGE_VERSION {
            info!("Calibration storage missing or version mismatch");
            return Err(Ads1220Error::InvalidCalibration);
        }
        if checksum != Self::calibration_checksum(factor_bits) {
            error!("Calibration storage checksum mismatch");
            return Err(Ads1220Error::InvalidCalibration);
        }

        let factor = f32::from_bits(factor_bits);
        if !Self::is_valid_calibration_factor(factor) {
            info!("Invalid calibration factor read from flash");
            return Err(Ads1220Error::InvalidCalibration);
        }
        Ok(factor)
    }

    /// Check whether a calibration factor can safely be used.
    pub fn is_valid_calibration_factor(factor: f32) -> bool {
        factor.is_finite() && factor != 0.0
    }

    fn write_to_flash(&mut self, calibration_factor: f32) -> Result<(), Ads1220Error> {
        if !Self::is_valid_calibration_factor(calibration_factor) {
            return Err(Ads1220Error::InvalidCalibration);
        }

        let factor_bits = calibration_factor.to_bits();
        let checksum = Self::calibration_checksum(factor_bits);
        let mut bytes = [0; CALIBRATION_STORAGE_SIZE];
        bytes[0..4].copy_from_slice(&CALIBRATION_STORAGE_MAGIC.to_le_bytes());
        bytes[4..8].copy_from_slice(&CALIBRATION_STORAGE_VERSION.to_le_bytes());
        bytes[8..12].copy_from_slice(&factor_bits.to_le_bytes());
        bytes[12..16].copy_from_slice(&checksum.to_le_bytes());

        let offset = self.calibration_storage_offset()?;
        self.flash.write(offset, &bytes).map_err(|_| {
            error!("Failed to write calibration factor to flash");
            Ads1220Error::FlashError
        })
    }

    /// Update the calibration factor in memory and flash.
    pub fn update_calibration_factor(&mut self, factor: f32) -> Result<(), Ads1220Error> {
        if !Self::is_valid_calibration_factor(factor) {
            error!("Invalid calibration factor: {}", factor);
            return Err(Ads1220Error::InvalidCalibration);
        }

        info!("Updating calibration factor: {}", factor);
        self.write_to_flash(factor)?;
        self.calibration_factor = factor;
        Ok(())
    }

    /// Load the persisted calibration factor, or the default if none is valid.
    pub fn get_calibration_factor(&mut self) -> Result<f32, Ads1220Error> {
        match self.read_from_flash() {
            Ok(factor) => {
                info!("Calibration factor read from flash: {:?}", factor);
                Ok(factor)
            }
            Err(Ads1220Error::InvalidCalibration) => {
                info!("Using default calibration factor");
                Ok(DEFAULT_CALIBRATION_FACTOR)
            }
            Err(error) => Err(error),
        }
    }

    /// Return the active calibration factor.
    pub fn current_calibration_factor(&self) -> f32 {
        self.calibration_factor
    }

    /// Restore and persist the default calibration factor.
    pub fn default_calibration_factor(&mut self) -> Result<(), Ads1220Error> {
        debug!("Restoring default calibration factor");
        self.write_to_flash(DEFAULT_CALIBRATION_FACTOR)?;
        self.calibration_factor = DEFAULT_CALIBRATION_FACTOR;
        Ok(())
    }

    /// Apply a best-fit factor to raw-code/weight calibration points.
    pub fn apply_multi_point_calibration(&mut self, calibration_points: &[(f32, f32)]) -> bool {
        if calibration_points.len() < 2 {
            error!("Calibration requires at least two points");
            return false;
        }

        let mut valid_count = 0;
        let mut base_point = None;
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
            Ok(()) => {
                info!(
                    "Calibration factor successfully applied: {:?}",
                    scale_factor
                );
                true
            }
            Err(error) => {
                error!(
                    "Failed to apply calibration factor: {:?}",
                    defmt::Debug2Format(&error)
                );
                false
            }
        }
    }
}
