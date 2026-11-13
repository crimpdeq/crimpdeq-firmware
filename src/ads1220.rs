//! ADS1220 load-cell ADC driver.
//!
//! The crimpdeq-tscircuit board connects the ADS1220 over SPI and uses the
//! bridge excitation voltage as the ADC reference. The bridge low side (E−)
//! returns to AVSS through the ADS1220 low-side switch on AIN3/REFN1, so the
//! bridge is excited only between START/SYNC and POWERDOWN (datasheet section
//! 8.3.9). REFN0 senses E− above the switch, so the reference stays
//! ratiometric. The driver waits on the dedicated DRDY pin before each read.

use crimpdeq_calibration::Calibration;
use defmt::{Format, debug, info};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_hal_async::spi::SpiBus;
use esp_hal::{
    Async,
    gpio::{Input, Output},
    spi::master::Spi,
};

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
        0x48,                                       // External reference on REFP0/REFN0, PSW = 1
        0x00,                                       // IDACs disabled; data ready only on DRDY
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

const TARING_TIME: Duration = Duration::from_millis(200);
const CALIBRATION_TIME: Duration = Duration::from_millis(1250);
/// Calibration points are averaged at a low rate, which has less noise per conversion.
const CALIBRATION_SAMPLE_RATE: SampleRate = SampleRate::Sps90;

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

    /// Time from START/SYNC until the first conversion completes. Until then, DRDY can still
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
#[derive(Format)]
pub enum Ads1220Error {
    /// SPI communication failed.
    Spi,
    /// No conversion became ready on DRDY.
    DataReadyTimeout,
    /// ADS1220 configuration register readback did not match.
    ConfigurationMismatch {
        expected: [u8; REGISTER_COUNT],
        actual: [u8; REGISTER_COUNT],
    },
}

/// ADS1220 load-cell front end.
pub struct Ads1220<'d> {
    spi: Spi<'d, Async>,
    chip_select: Output<'d>,
    /// Dedicated DRDY pin, actively driven also while chip select is high.
    data_ready: Input<'d>,
    tare_value: i32,
    first_conversion_at: Instant,
    powered_down: bool,
    sample_rate: SampleRate,
}

impl<'d> Ads1220<'d> {
    /// Create a driver. Call [`Self::initialize`] before taking measurements.
    pub fn new(spi: Spi<'d, Async>, mut chip_select: Output<'d>, data_ready: Input<'d>) -> Self {
        chip_select.set_high();

        Self {
            spi,
            chip_select,
            data_ready,
            tare_value: 0,
            first_conversion_at: Instant::now(),
            powered_down: false,
            sample_rate: SampleRate::DEFAULT,
        }
    }

    /// Reset and configure the ADC for the switched ratiometric load cell.
    pub async fn initialize(&mut self, sample_rate: SampleRate) -> Result<(), Ads1220Error> {
        self.send_command(COMMAND_RESET).await?;
        Timer::after(RESET_DELAY).await;

        self.configure(sample_rate).await?;
        self.start_conversions().await?;
        info!("ADS1220 initialized at {} SPS", self.sample_rate.sps());
        Ok(())
    }

    /// Returns the configured data rate.
    pub fn sample_rate(&self) -> SampleRate {
        self.sample_rate
    }

    /// Change the data rate. Restarts conversions unless the ADC is powered down, which keeps the
    /// registers until the next START/SYNC.
    pub async fn set_sample_rate(&mut self, sample_rate: SampleRate) -> Result<(), Ads1220Error> {
        self.configure(sample_rate).await?;
        if !self.powered_down {
            self.start_conversions().await?;
        }
        Ok(())
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

        // DRDY falls once a conversion is ready and rises on the first SCLK of its read, so every
        // conversion is read exactly once. The data is clocked out directly, with DIN low, without
        // RDATA (datasheet section 8.5.4).
        with_timeout(
            self.sample_rate.data_ready_timeout(),
            self.data_ready.wait_for_low(),
        )
        .await
        .map_err(|_| Ads1220Error::DataReadyTimeout)?;
        let mut transaction = [0; 3];
        self.chip_select.set_low();
        let result = SpiBus::transfer_in_place(&mut self.spi, &mut transaction).await;
        self.chip_select.set_high();
        result.map_err(|_| Ads1220Error::Spi)?;

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

    /// Put the ADC into its low-power mode after the current conversion. This also opens the
    /// low-side switch, which turns off the bridge excitation.
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

    /// Restart continuous conversions, and with them the bridge excitation, if the ADC is powered
    /// down.
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
        self.tare_value = self.read_tare_average().await?;
        debug!("Tare value set to: {}", self.tare_value);
        Ok(())
    }

    /// Average ADC code over the tare time, without taring.
    pub async fn read_tare_average(&mut self) -> Result<i32, Ads1220Error> {
        let average = self
            .take_samples(self.sample_rate.samples_in(TARING_TIME))
            .await?;
        Ok(average as i32)
    }

    pub fn tare_value(&self) -> i32 {
        self.tare_value
    }

    pub fn set_tare_value(&mut self, tare_value: i32) {
        self.tare_value = tare_value;
    }

    /// Read the load in kilograms on top of the tare.
    pub async fn read_calibrated(
        &mut self,
        calibration: &Calibration,
        zero_shift: f32,
    ) -> Result<f32, Ads1220Error> {
        let raw = self.read_raw().await?;
        Ok(calibration.net_kg(raw, self.tare_value, zero_shift))
    }

    /// Collect an averaged raw calibration point at the calibration sample rate.
    pub async fn perform_calibration(&mut self) -> Result<f32, Ads1220Error> {
        let sample_rate = self.sample_rate;
        self.set_sample_rate(CALIBRATION_SAMPLE_RATE).await?;
        let average = self
            .take_samples(CALIBRATION_SAMPLE_RATE.samples_in(CALIBRATION_TIME))
            .await;
        self.set_sample_rate(sample_rate).await?;
        let average = average?;
        debug!("Calibration point collected: {}", average);
        Ok(average)
    }
}
