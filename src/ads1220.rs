//! ADS1220 load-cell ADC driver.
//!
//! The revision-3 PCB connects the ADS1220 over SPI and uses the bridge
//! excitation voltage as the ADC reference. The dedicated DRDY pin is not
//! connected, so the ADC also signals data ready on DOUT/DRDY, which the
//! driver watches while chip select is low (datasheet section 8.5.6).

use core::fmt;

use defmt::{Format, debug, error, info, warn};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_hal_async::spi::SpiBus;
use esp_bootloader_esp_idf::partitions::{PARTITION_TABLE_MAX_LEN, read_partition_table};
use esp_hal::{
    Async,
    gpio::{Input, Output},
    spi::master::Spi,
};
use esp_storage::FlashStorage;

const COMMAND_POWERDOWN: u8 = 0x02;
const COMMAND_RESET: u8 = 0x06;
const COMMAND_START_SYNC: u8 = 0x08;
const COMMAND_RREG: u8 = 0x20;
const COMMAND_WREG: u8 = 0x40;

const REGISTER_COUNT: usize = 4;

const fn configuration_registers(sample_rate: SampleRate) -> [u8; REGISTER_COUNT] {
    [
        0x0E,                                       // AIN0-AIN1, gain 128, PGA enabled
        (sample_rate.data_rate_bits() << 5) | 0x04, // Normal mode, continuous conversion
        0x40,                                       // External reference on REFP0/REFN0
        0x02,                                       // IDACs disabled; data ready also on DOUT/DRDY
    ]
}

const RESET_DELAY: Duration = Duration::from_micros(100);
/// Internal oscillator frequency; tCLK = 1 / CLOCK_HZ.
const CLOCK_HZ: u64 = 4_096_000;
/// Internal oscillator wake-up time after START/SYNC (datasheet section 8.3.7).
const OSCILLATOR_WAKE_UP_TIME: Duration = Duration::from_micros(50);
/// Delay from START/SYNC until the first conversion starts in normal mode (datasheet section
/// 8.3.7).
const START_DELAY_CLOCKS: u32 = 210;
/// Extra wait for a conversion beyond its nominal conversion time.
const DATA_READY_TIMEOUT_MARGIN: Duration = Duration::from_millis(8);

/// Label of the data partition that stores the calibration and the sample rate.
const CALIBRATION_PARTITION_LABEL: &str = "calib";
/// Size of a stored setting: magic, version, value and checksum, as little-endian u32 words.
const STORAGE_RECORD_SIZE: usize = 16;
/// The calibration record stays at the partition start, where earlier firmware stored it.
const CALIBRATION_RECORD: StorageRecord = StorageRecord {
    offset: 0,
    magic: 0x4344_5146,
    // Version 2 invalidates calibration factors saved for the old HX711 front end.
    version: 2,
};
const SAMPLE_RATE_RECORD: StorageRecord = StorageRecord {
    offset: STORAGE_RECORD_SIZE as u32,
    magic: 0x4344_5352,
    version: 1,
};
const STORAGE_SIZE: usize = 2 * STORAGE_RECORD_SIZE;
const TARING_TIME: Duration = Duration::from_millis(200);
const CALIBRATION_TIME: Duration = Duration::from_millis(1250);
/// Calibration points are averaged at a low rate, which has less noise per conversion.
const CALIBRATION_SAMPLE_RATE: SampleRate = SampleRate::Sps90;
const DEFAULT_CALIBRATION_FACTOR: f32 = 0.08;

/// Location and validation constants of one stored setting.
struct StorageRecord {
    /// Offset within the storage partition.
    offset: u32,
    magic: u32,
    version: u32,
}

impl StorageRecord {
    fn checksum(&self, value: u32) -> u32 {
        self.magic ^ self.version ^ value ^ 0xA5A5_5A5A
    }
}

/// Normal-mode data rates (datasheet table 8-12). The discriminant is the DR[2:0] value.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Format)]
pub enum SampleRate {
    Sps20 = 0,
    Sps45 = 1,
    Sps90 = 2,
    Sps175 = 3,
    Sps330 = 4,
    Sps600 = 5,
    Sps1000 = 6,
}

impl SampleRate {
    pub const DEFAULT: Self = Self::Sps175;
    const ALL: [Self; 7] = [
        Self::Sps20,
        Self::Sps45,
        Self::Sps90,
        Self::Sps175,
        Self::Sps330,
        Self::Sps600,
        Self::Sps1000,
    ];

    /// Returns the rate with the given samples per second, if the ADS1220 supports it.
    pub fn from_sps(sps: u16) -> Option<Self> {
        Self::ALL.into_iter().find(|rate| rate.sps() == sps)
    }

    pub const fn sps(self) -> u16 {
        match self {
            Self::Sps20 => 20,
            Self::Sps45 => 45,
            Self::Sps90 => 90,
            Self::Sps175 => 175,
            Self::Sps330 => 330,
            Self::Sps600 => 600,
            Self::Sps1000 => 1000,
        }
    }

    const fn data_rate_bits(self) -> u8 {
        self as u8
    }

    /// Continuous and single-shot conversion times in tCLK (datasheet table 8-4).
    const fn conversion_clocks(self) -> (u32, u32) {
        match self {
            Self::Sps20 => (204_768, 204_850),
            Self::Sps45 => (91_120, 91_218),
            Self::Sps90 => (46_128, 46_226),
            Self::Sps175 => (23_664, 23_762),
            Self::Sps330 => (12_464, 12_562),
            Self::Sps600 => (6_896, 6_994),
            Self::Sps1000 => (4_144, 4_242),
        }
    }

    /// Longest wait for a conversion while converting continuously.
    fn data_ready_timeout(self) -> Duration {
        clocks_to_duration(self.conversion_clocks().0) + DATA_READY_TIMEOUT_MARGIN
    }

    /// Time from START/SYNC until the first conversion completes. Until then, DOUT/DRDY can still
    /// flag the conversion that completed before POWERDOWN or before the rate changed.
    fn first_conversion_time(self) -> Duration {
        OSCILLATOR_WAKE_UP_TIME
            + clocks_to_duration(START_DELAY_CLOCKS + self.conversion_clocks().1)
    }

    /// Number of conversions that cover `window`, at least one.
    pub fn samples_in(self, window: Duration) -> usize {
        (u64::from(self.sps()) * window.as_millis() / 1000).max(1) as usize
    }
}

/// Converts tCLK periods to time, with margin for the 2% internal oscillator tolerance.
fn clocks_to_duration(clocks: u32) -> Duration {
    Duration::from_micros((u64::from(clocks) * 1_000_000 * 102).div_ceil(CLOCK_HZ * 100))
}

/// Errors produced by the ADS1220 load-cell driver.
#[derive(Debug)]
pub enum Ads1220Error {
    /// Flash storage access failed.
    FlashError,
    /// A calibration value is absent or invalid.
    InvalidCalibration,
    /// SPI communication failed.
    Spi,
    /// No conversion became ready on DOUT/DRDY.
    DataReadyTimeout,
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
            Ads1220Error::DataReadyTimeout => write!(f, "ADS1220 data ready timeout"),
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
    /// DOUT/DRDY, which is also the SPI MISO input.
    data_ready: Input<'d>,
    flash: FlashStorage<'d>,
    tare_value: i32,
    calibration_factor: f32,
    first_conversion_at: Instant,
    powered_down: bool,
    sample_rate: SampleRate,
}

impl<'d> Ads1220<'d> {
    /// Create a driver. Call [`Self::initialize`] before taking measurements.
    pub fn new(
        spi: Spi<'d, Async>,
        mut chip_select: Output<'d>,
        data_ready: Input<'d>,
        flash: FlashStorage<'d>,
    ) -> Self {
        chip_select.set_high();

        let mut ads1220 = Self {
            spi,
            chip_select,
            data_ready,
            flash,
            tare_value: 0,
            calibration_factor: 0.0,
            first_conversion_at: Instant::now(),
            powered_down: false,
            sample_rate: SampleRate::DEFAULT,
        };

        ads1220.calibration_factor = ads1220
            .get_calibration_factor()
            .unwrap_or(DEFAULT_CALIBRATION_FACTOR);
        ads1220.sample_rate = ads1220.stored_sample_rate();
        ads1220
    }

    /// Reset and configure the ADC for the revision-3 ratiometric load cell.
    pub async fn initialize(&mut self) -> Result<(), Ads1220Error> {
        self.send_command(COMMAND_RESET).await?;
        Timer::after(RESET_DELAY).await;

        self.configure(self.sample_rate).await?;
        self.start_conversions().await?;
        info!("ADS1220 initialized at {} SPS", self.sample_rate.sps());
        Ok(())
    }

    /// Returns the configured data rate.
    pub fn sample_rate(&self) -> SampleRate {
        self.sample_rate
    }

    /// Persist the data rate, so it also applies after a reboot, then switch to it. The rate in use
    /// only changes once both steps succeed, so the caller retries until it does.
    pub async fn set_sample_rate(&mut self, sample_rate: SampleRate) -> Result<(), Ads1220Error> {
        let sps = u32::from(sample_rate.sps());
        // Skip the write when an earlier attempt stored the rate, to spare the flash while the ADC
        // keeps failing.
        if self.read_record(&SAMPLE_RATE_RECORD)? != Some(sps) {
            self.write_record(&SAMPLE_RATE_RECORD, sps)?;
        }
        self.switch_sample_rate(sample_rate).await?;
        info!("ADS1220 sample rate set to {} SPS", sample_rate.sps());
        Ok(())
    }

    /// Change the data rate without persisting it. Restarts conversions unless the ADC is powered
    /// down, which keeps the registers until the next START/SYNC.
    async fn switch_sample_rate(&mut self, sample_rate: SampleRate) -> Result<(), Ads1220Error> {
        self.configure(sample_rate).await?;
        if !self.powered_down {
            self.start_conversions().await?;
        }
        Ok(())
    }

    fn stored_sample_rate(&mut self) -> SampleRate {
        let stored = match self.read_record(&SAMPLE_RATE_RECORD) {
            Ok(stored) => stored,
            Err(error) => {
                error!(
                    "Failed to read the stored sample rate: {:?}",
                    defmt::Debug2Format(&error)
                );
                None
            }
        };
        match stored.and_then(|sps| SampleRate::from_sps(u16::try_from(sps).ok()?)) {
            Some(sample_rate) => sample_rate,
            None => {
                info!("Using default sample rate");
                SampleRate::DEFAULT
            }
        }
    }

    async fn configure(&mut self, sample_rate: SampleRate) -> Result<(), Ads1220Error> {
        let expected = configuration_registers(sample_rate);
        self.write_configuration(expected).await?;
        let actual = self.read_configuration().await?;
        if actual != expected {
            return Err(Ads1220Error::ConfigurationMismatch { expected, actual });
        }
        self.sample_rate = sample_rate;
        Ok(())
    }

    async fn start_conversions(&mut self) -> Result<(), Ads1220Error> {
        self.send_command(COMMAND_START_SYNC).await?;
        self.first_conversion_at = Instant::now() + self.sample_rate.first_conversion_time();
        Ok(())
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

    /// Wait for the next conversion and read it.
    async fn read_raw(&mut self) -> Result<i32, Ads1220Error> {
        Timer::at(self.first_conversion_at).await;

        // With chip select low, DOUT/DRDY drives low once a conversion is ready. Clocking out the
        // three data bytes plus one byte with DIN low leaves DOUT/DRDY high until the next
        // conversion, so every conversion is read exactly once.
        let mut transaction = [0; 4];
        let timeout = self.sample_rate.data_ready_timeout();
        self.chip_select.set_low();
        let result = match with_timeout(timeout, self.data_ready.wait_for_low()).await {
            Ok(()) => SpiBus::transfer_in_place(&mut self.spi, &mut transaction)
                .await
                .map_err(|_| Ads1220Error::Spi),
            Err(_) => Err(Ads1220Error::DataReadyTimeout),
        };
        self.chip_select.set_high();
        result?;

        // Sign-extend the 24-bit two's complement result.
        Ok(i32::from_be_bytes([transaction[0], transaction[1], transaction[2], 0]) >> 8)
    }

    async fn take_samples(&mut self, num_samples: usize) -> Result<f32, Ads1220Error> {
        let mut total = 0i64;
        for _ in 0..num_samples {
            total += i64::from(self.read_raw().await?);
        }
        Ok((total as f64 / num_samples as f64) as f32)
    }

    /// Put the ADC into its low-power mode after the current conversion.
    pub async fn power_down(&mut self) -> Result<(), Ads1220Error> {
        debug!("Powering down ADS1220");
        self.send_command(COMMAND_POWERDOWN).await?;
        self.powered_down = true;
        Ok(())
    }

    /// Returns whether the ADC is in power-down mode.
    pub fn is_powered_down(&self) -> bool {
        self.powered_down
    }

    /// Restart continuous conversions if the ADC is powered down.
    pub async fn wake(&mut self) -> Result<(), Ads1220Error> {
        if self.powered_down {
            debug!("Powering up ADS1220");
            self.start_conversions().await?;
            self.powered_down = false;
        }
        Ok(())
    }

    /// Tare the load cell with an average of multiple readings.
    pub async fn tare(&mut self) -> Result<(), Ads1220Error> {
        debug!("Taring the scale");
        let average = self
            .take_samples(self.sample_rate.samples_in(TARING_TIME))
            .await?;
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

    /// Collect an averaged raw calibration point at the calibration sample rate.
    pub async fn perform_calibration(&mut self) -> Result<f32, Ads1220Error> {
        let sample_rate = self.sample_rate;
        self.switch_sample_rate(CALIBRATION_SAMPLE_RATE).await?;
        let average = self
            .take_samples(CALIBRATION_SAMPLE_RATE.samples_in(CALIBRATION_TIME))
            .await;
        self.switch_sample_rate(sample_rate).await?;
        let average = average?;
        debug!("Calibration point collected: {}", average);
        Ok(average)
    }

    fn storage_offset(&mut self) -> Result<u32, Ads1220Error> {
        let mut table_buffer = [0u8; PARTITION_TABLE_MAX_LEN];
        match read_partition_table(&mut self.flash, &mut table_buffer) {
            Ok(table) => {
                if let Some(partition) = table
                    .iter()
                    .find(|partition| partition.label_as_str() == CALIBRATION_PARTITION_LABEL)
                {
                    if (partition.len() as usize) < STORAGE_SIZE {
                        error!("Calibration partition too small");
                        return Err(Ads1220Error::FlashError);
                    }
                    return Ok(partition.offset());
                }
            }
            Err(e) => warn!("Failed to read partition table: {:?}", e),
        }

        // Updates that only flash the app keep the previous partition table, which has no
        // calibration partition. Fall back to the last flash sector: the calibration was stored
        // there before the partition existed, and the partition starts at the same offset.
        warn!("Calibration partition not found, using the last flash sector");
        let capacity = self.flash.capacity();
        let sector_size = FlashStorage::SECTOR_SIZE as usize;

        if capacity < sector_size {
            error!("Flash capacity too small for calibration storage");
            return Err(Ads1220Error::FlashError);
        }

        Ok((capacity - sector_size) as u32)
    }

    fn read_from_flash(&mut self) -> Result<f32, Ads1220Error> {
        let Some(factor_bits) = self.read_record(&CALIBRATION_RECORD)? else {
            return Err(Ads1220Error::InvalidCalibration);
        };

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
        self.write_record(&CALIBRATION_RECORD, calibration_factor.to_bits())
    }

    /// Read a stored value, or `None` when the record is missing or corrupt.
    fn read_record(&mut self, record: &StorageRecord) -> Result<Option<u32>, Ads1220Error> {
        let mut bytes = [0; STORAGE_RECORD_SIZE];
        let offset = self.storage_offset()? + record.offset;
        self.flash.read(offset, &mut bytes).map_err(|_| {
            error!(
                "Failed to read storage record {:#x} from flash",
                record.magic
            );
            Ads1220Error::FlashError
        })?;

        let words = bytes.as_chunks::<4>().0;
        let [magic, version, value, checksum] =
            core::array::from_fn(|i| u32::from_le_bytes(words[i]));
        if magic != record.magic || version != record.version {
            info!(
                "Storage record {:#x} missing or version mismatch",
                record.magic
            );
            return Ok(None);
        }
        if checksum != record.checksum(value) {
            error!("Storage record {:#x} checksum mismatch", record.magic);
            return Ok(None);
        }
        Ok(Some(value))
    }

    fn write_record(&mut self, record: &StorageRecord, value: u32) -> Result<(), Ads1220Error> {
        let words = [record.magic, record.version, value, record.checksum(value)];
        let mut bytes = [0; STORAGE_RECORD_SIZE];
        for (chunk, word) in bytes.as_chunks_mut::<4>().0.iter_mut().zip(words) {
            *chunk = word.to_le_bytes();
        }

        // FlashStorage::write preserves the rest of the sector, including the other records.
        let offset = self.storage_offset()? + record.offset;
        self.flash.write(offset, &bytes).map_err(|_| {
            error!(
                "Failed to write storage record {:#x} to flash",
                record.magic
            );
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

        let is_valid = |(raw_value, weight): &&(f32, f32)| {
            raw_value.is_finite() && weight.is_finite() && *weight >= 0.0
        };
        for (raw_value, weight) in calibration_points.iter().filter(|point| !is_valid(point)) {
            error!(
                "Skipping invalid calibration point raw={}, weight={}",
                raw_value, weight
            );
        }

        // Incoming calibration weights are expressed in kg, while calibration_factor operates on
        // grams before read_calibrated() converts back to kg.
        let valid_points = || {
            calibration_points
                .iter()
                .filter(is_valid)
                .map(|(raw_value, weight)| (f64::from(*raw_value), f64::from(*weight) * 1000.0))
        };

        let valid_count = valid_points().count();
        if valid_count < 2 {
            error!("Calibration requires at least two valid points");
            return false;
        }

        // Least-squares slope around the mean, so no single point anchors the fit.
        let (sum_raw, sum_weight) = valid_points()
            .fold((0.0, 0.0), |(sum_raw, sum_weight), (raw, weight)| {
                (sum_raw + raw, sum_weight + weight)
            });
        let mean_raw = sum_raw / valid_count as f64;
        let mean_weight = sum_weight / valid_count as f64;
        let (sum_delta_raw_weight, sum_delta_raw_sq) = valid_points().fold(
            (0.0, 0.0),
            |(sum_delta_raw_weight, sum_delta_raw_sq), (raw, weight)| {
                let delta_raw = raw - mean_raw;
                (
                    sum_delta_raw_weight + delta_raw * (weight - mean_weight),
                    sum_delta_raw_sq + delta_raw * delta_raw,
                )
            },
        );

        if sum_delta_raw_sq < f64::EPSILON {
            error!("Invalid calibration - points are too close together");
            return false;
        }

        let scale_factor = (sum_delta_raw_weight / sum_delta_raw_sq) as f32;
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
