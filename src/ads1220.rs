//! ADS1220 load-cell ADC driver.
//!
//! The revision-3 PCB connects the ADS1220 over SPI and uses the bridge
//! excitation voltage as the ADC reference. The dedicated DRDY pin is not
//! connected, so the ADC also signals data ready on DOUT/DRDY, which the
//! driver watches while chip select is low (datasheet section 8.5.6).
//!
//! The async SPI driver does not keep the chip out of automatic light sleep, so every SPI
//! transaction holds a [`WakeLock`]. Continuous conversions hold one as well: waking from light
//! sleep for each conversion at 1000 SPS could miss the next one.

use defmt::{Format, debug, error, info};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_hal_async::spi::SpiBus;
use esp_hal::{
    Async,
    gpio::{Input, Output},
    rtc_cntl::WakeLock,
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

const TARING_TIME: Duration = Duration::from_millis(200);
const CALIBRATION_TIME: Duration = Duration::from_millis(1250);
/// Calibration points are averaged at a low rate, which has less noise per conversion.
const CALIBRATION_SAMPLE_RATE: SampleRate = SampleRate::Sps90;
pub const DEFAULT_CALIBRATION_FACTOR: f32 = 0.109;

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
#[derive(Format)]
pub enum Ads1220Error {
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

/// ADS1220 load-cell front end.
pub struct Ads1220<'d> {
    spi: Spi<'d, Async>,
    chip_select: Output<'d>,
    /// DOUT/DRDY, which is also the SPI MISO input.
    data_ready: Input<'d>,
    tare_value: i32,
    first_conversion_at: Instant,
    /// Held while the ADC converts, `None` in power-down mode.
    converting: Option<WakeLock>,
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
            converting: Some(WakeLock::new()),
            sample_rate: SampleRate::DEFAULT,
        }
    }

    /// Reset and configure the ADC for the revision-3 ratiometric load cell.
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
        if !self.is_powered_down() {
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
        let _awake = WakeLock::new();
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

        let _awake = WakeLock::new();
        self.chip_select.set_low();
        let result = SpiBus::write(&mut self.spi, &bytes).await;
        self.chip_select.set_high();
        result.map_err(|_| Ads1220Error::Spi)
    }

    async fn read_configuration(&mut self) -> Result<[u8; REGISTER_COUNT], Ads1220Error> {
        let command = COMMAND_RREG | (REGISTER_COUNT as u8 - 1);
        let mut registers = [0; REGISTER_COUNT];

        let _awake = WakeLock::new();
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
        self.converting = None;
        Ok(())
    }

    /// Returns whether the ADC is in power-down mode.
    pub fn is_powered_down(&self) -> bool {
        self.converting.is_none()
    }

    /// Restart continuous conversions if the ADC is powered down.
    pub async fn wake(&mut self) -> Result<(), Ads1220Error> {
        if self.is_powered_down() {
            debug!("Powering up ADS1220");
            self.start_conversions().await?;
            self.converting = Some(WakeLock::new());
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

    /// Read an ADC code with the tare offset removed.
    pub async fn read_tared(&mut self) -> Result<i32, Ads1220Error> {
        Ok(self.read_raw().await? - self.tare_value)
    }

    /// Read a calibrated load-cell value in kilograms.
    pub async fn read_calibrated(&mut self, calibration_factor: f32) -> Result<f32, Ads1220Error> {
        let raw_tared = self.read_tared().await?;
        Ok((raw_tared as f32) * calibration_factor / 1000.0)
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

/// How many times above or below DEFAULT_CALIBRATION_FACTOR a factor can be before it is rejected
/// as implausible for the hardware, such as a fit of points whose load did not change.
const CALIBRATION_FACTOR_TOLERANCE: f32 = 10.0;

/// Check whether a calibration factor is plausible for the hardware. Either sign is accepted, so a
/// load cell wired in reverse can still be calibrated.
pub fn is_valid_calibration_factor(factor: f32) -> bool {
    (DEFAULT_CALIBRATION_FACTOR / CALIBRATION_FACTOR_TOLERANCE
        ..=DEFAULT_CALIBRATION_FACTOR * CALIBRATION_FACTOR_TOLERANCE)
        .contains(&factor.abs())
}

/// Returns the best-fit factor for raw-code/weight calibration points, or `None` when the points
/// do not define one. The raw codes are averages of ADC codes and the weights were validated when
/// received, so every point is finite.
///
/// Measurements are tared with no load, so the applied line passes through the no-load
/// reading. To match, the fit passes through the average of the zero-weight points, or through
/// the mean point, the least-squares estimate of that reading, when there is none.
pub fn fit_calibration_factor(calibration_points: &[(f32, f32)]) -> Option<f32> {
    if calibration_points.len() < 2 {
        error!("Calibration requires at least two points");
        return None;
    }

    // Incoming calibration weights are expressed in kg, while calibration_factor operates on
    // grams before read_calibrated() converts back to kg.
    let points = || {
        calibration_points
            .iter()
            .map(|(raw_value, weight)| (f64::from(*raw_value), f64::from(*weight) * 1000.0))
    };
    let (anchor_raw, anchor_weight) =
        match mean_point(points().filter(|&(_, weight)| weight == 0.0)) {
            Some(zero_point) => zero_point,
            None => mean_point(points()).expect("checked that there are at least two points"),
        };

    // Least-squares slope of a line through the anchor.
    let (sum_delta_raw_weight, sum_delta_raw_sq) = points().fold(
        (0.0, 0.0),
        |(sum_delta_raw_weight, sum_delta_raw_sq), (raw, weight)| {
            let delta_raw = raw - anchor_raw;
            (
                sum_delta_raw_weight + delta_raw * (weight - anchor_weight),
                sum_delta_raw_sq + delta_raw * delta_raw,
            )
        },
    );

    if sum_delta_raw_sq < f64::EPSILON {
        error!("Invalid calibration - points are too close together");
        return None;
    }

    let factor = (sum_delta_raw_weight / sum_delta_raw_sq) as f32;
    if !is_valid_calibration_factor(factor) {
        error!("Invalid calibration factor: {}", factor);
        return None;
    }
    Some(factor)
}

/// Average of (raw code, weight) points, or `None` without points.
fn mean_point(points: impl Iterator<Item = (f64, f64)>) -> Option<(f64, f64)> {
    let (count, sum_raw, sum_weight) = points.fold(
        (0usize, 0.0, 0.0),
        |(count, sum_raw, sum_weight), (raw, weight)| {
            (count + 1, sum_raw + raw, sum_weight + weight)
        },
    );
    (count > 0).then(|| (sum_raw / count as f64, sum_weight / count as f64))
}
