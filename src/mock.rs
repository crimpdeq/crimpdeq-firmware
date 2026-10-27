//! Simulated load cell and battery, so the firmware runs without a load cell or battery on PCB
//! revision 3 or on the ESP32-C3 Rust board (https://github.com/esp-rs/esp-rust-board), which
//! has neither an ADS1220 nor a MAX17048.

use core::convert::Infallible;

use embassy_time::{Duration, Instant, Timer};

use crate::{
    ads1220::{DEFAULT_CALIBRATION_FACTOR, SampleRate},
    progressor::BatteryStatus,
};

/// Peak of each simulated pull, in kg, repeated in order.
const PULL_PEAKS_KG: [f32; 5] = [18.0, 30.0, 24.0, 12.0, 27.0];
const REST_S: f32 = 2.0;
const RAMP_UP_S: f32 = 1.5;
const HOLD_S: f32 = 4.0;
const RAMP_DOWN_S: f32 = 1.0;
const PULL_S: f32 = REST_S + RAMP_UP_S + HOLD_S + RAMP_DOWN_S;
/// Share of the peak lost by the end of the hold, as the fingers tire.
const HOLD_FATIGUE: f32 = 0.1;

/// Weight of a pull cycle `elapsed_s` after the load cell woke: rest, ramp up, hold while tiring,
/// release. Stays between 0 and the peak.
fn pull_weight_kg(elapsed_s: f32) -> f32 {
    let pull = (elapsed_s / PULL_S) as usize;
    let peak = PULL_PEAKS_KG[pull % PULL_PEAKS_KG.len()];
    let held = peak * (1.0 - HOLD_FATIGUE);

    let t = elapsed_s - pull as f32 * PULL_S - REST_S;
    if t < 0.0 {
        return 0.0;
    }
    if t < RAMP_UP_S {
        return peak * smoothstep(t / RAMP_UP_S);
    }
    let t = t - RAMP_UP_S;
    if t < HOLD_S {
        return peak - (peak - held) * t / HOLD_S;
    }
    held * (1.0 - smoothstep((t - HOLD_S) / RAMP_DOWN_S))
}

/// Eases 0..=1 into 0..=1 with zero slope at both ends.
fn smoothstep(x: f32) -> f32 {
    let x = x.clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

/// Load cell with the [`crate::ads1220::Ads1220`] interface that converts simulated pulls to ADC
/// codes, so taring and calibration run as on the ADS1220. With the default calibration the
/// readings span 0 to 30 kg.
pub struct MockLoadCell {
    woke_at: Instant,
    next_conversion_at: Instant,
    tare_value: i32,
    powered_down: bool,
    sample_rate: SampleRate,
}

impl MockLoadCell {
    pub fn new() -> Self {
        Self {
            woke_at: Instant::now(),
            next_conversion_at: Instant::now(),
            tare_value: 0,
            powered_down: false,
            sample_rate: SampleRate::DEFAULT,
        }
    }

    pub async fn initialize(&mut self, sample_rate: SampleRate) -> Result<(), Infallible> {
        self.sample_rate = sample_rate;
        Ok(())
    }

    pub fn sample_rate(&self) -> SampleRate {
        self.sample_rate
    }

    pub async fn set_sample_rate(&mut self, sample_rate: SampleRate) -> Result<(), Infallible> {
        self.sample_rate = sample_rate;
        Ok(())
    }

    pub async fn power_down(&mut self) -> Result<(), Infallible> {
        self.powered_down = true;
        Ok(())
    }

    pub fn is_powered_down(&self) -> bool {
        self.powered_down
    }

    /// Restarts the pull cycle from rest. Each measurement start wakes the load cell, so every
    /// measurement starts unloaded and repeats the same pulls, also when a tare or calibration
    /// woke it before.
    pub async fn wake(&mut self) -> Result<(), Infallible> {
        self.woke_at = Instant::now();
        self.powered_down = false;
        Ok(())
    }

    /// Waits for the next conversion at the sample rate and returns its ADC code. Like the
    /// ADS1220, conversions missed while not reading are skipped rather than queued.
    async fn read_raw(&mut self) -> i32 {
        let now = Instant::now();
        if self.next_conversion_at < now {
            self.next_conversion_at = now;
        }
        Timer::at(self.next_conversion_at).await;
        self.next_conversion_at += Duration::from_hz(self.sample_rate.sps().into());

        let elapsed_s = self.woke_at.elapsed().as_micros() as f32 / 1_000_000.0;
        (pull_weight_kg(elapsed_s) * 1000.0 / DEFAULT_CALIBRATION_FACTOR) as i32
    }

    pub async fn tare(&mut self) -> Result<(), Infallible> {
        self.tare_value = self.read_raw().await;
        Ok(())
    }

    pub async fn read_calibrated(&mut self, calibration_factor: f32) -> Result<f32, Infallible> {
        let raw_tared = self.read_raw().await - self.tare_value;
        Ok((raw_tared as f32) * calibration_factor / 1000.0)
    }

    pub async fn perform_calibration(&mut self) -> Result<f32, Infallible> {
        Ok(self.read_raw().await as f32)
    }
}

/// Lowest simulated charge, where the battery starts charging again. Stays above the low battery
/// LED and shutdown voltages.
const BATTERY_MIN_SOC_PERCENT: u8 = 20;

/// Battery that discharges 1 % per reading down to [`BATTERY_MIN_SOC_PERCENT`], then charges
/// back to full.
pub struct MockBattery {
    status: BatteryStatus,
}

impl MockBattery {
    pub fn new() -> Self {
        Self {
            status: BatteryStatus {
                soc_percent: 100,
                charging: false,
            },
        }
    }

    /// Advances one reading and returns the voltage in millivolts with the gauge status.
    pub fn read(&mut self) -> (u32, BatteryStatus) {
        let status = &mut self.status;
        if status.charging {
            status.soc_percent += 1;
            status.charging = status.soc_percent < 100;
        } else {
            status.soc_percent -= 1;
            status.charging = status.soc_percent <= BATTERY_MIN_SOC_PERCENT;
        }
        // Linear from 3.6 V empty to 4.2 V full.
        let voltage_mv = 3600 + 6 * u32::from(status.soc_percent);
        (voltage_mv, *status)
    }
}
