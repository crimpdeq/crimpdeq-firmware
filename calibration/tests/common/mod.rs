//! Calibration data for the tests.
#![allow(dead_code)]

use crimpdeq_calibration::{DEFAULT_FACTOR, Point};

/// Real calibration session: a reading of the default linear conversion and the reference load,
/// both in kg. The readings are rounded kg, not ADC codes, so they stand in for raw codes only up
/// to an affine map.
pub const SESSION: [(f32, f32); 16] = [
    (0.000, 0.000),
    (5.265, 5.025),
    (10.408, 10.000),
    (15.692, 15.000),
    (26.043, 25.000),
    (4.207, 4.000),
    (8.243, 7.900),
    (69.462, 69.700),
    (76.548, 77.550),
    (21.356, 20.450),
    (29.512, 28.350),
    (37.764, 36.250),
    (45.930, 44.250),
    (51.487, 50.150),
    (87.935, 90.400),
    (69.509, 69.800),
];

/// Raw codes of the session: the reading converted back with the default factor, on top of an
/// unloaded code and with the polarity of the wiring.
pub fn session_points(unloaded_raw: f32, polarity: f32) -> Vec<Point> {
    SESSION
        .iter()
        .map(|&(reading, load)| (raw(unloaded_raw, polarity, reading), load))
        .collect()
}

/// Raw code of a load-cell reading `u`, in kg of the default factor.
pub fn raw(unloaded_raw: f32, polarity: f32, u: f32) -> f32 {
    unloaded_raw + polarity * u * 1000.0 / DEFAULT_FACTOR
}

/// Load-cell response: the load that gives a reading `u` of the default factor.
#[derive(Clone, Copy)]
pub enum Truth {
    Linear,
    /// Sensitivity rising with the load.
    Quadratic,
    /// Sensitivity stepping up by 10 % at 48 kg.
    Piecewise,
}

impl Truth {
    pub fn load(self, u: f64) -> f64 {
        match self {
            Self::Linear => u,
            Self::Quadratic => u + 0.0015 * u * u,
            Self::Piecewise => {
                if u < 48.0 {
                    u
                } else {
                    48.0 + 1.1 * (u - 48.0)
                }
            }
        }
    }
}

/// Readings, in kg of the default factor, of a calibration from 0 to 85 kg.
pub const CALIBRATION_READINGS: [f32; 12] = [
    0.0, 5.0, 10.0, 17.0, 25.0, 33.0, 41.0, 50.0, 58.0, 66.0, 75.0, 85.0,
];
/// Readings between the calibration readings, to check a fit away from its points.
pub const HELD_OUT_READINGS: [f32; 8] = [2.5, 13.0, 21.0, 37.0, 45.5, 54.0, 70.0, 80.0];

/// Deterministic noise in [-1, 1].
pub struct Noise(u64);

impl Noise {
    pub fn new(seed: u64) -> Self {
        Self(seed)
    }

    pub fn next(&mut self) -> f32 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        ((self.0 >> 40) as f32 / (1u64 << 24) as f32) * 2.0 - 1.0
    }
}

/// Calibration points of a response, with raw noise in counts and reference noise in kg.
pub fn synthetic_points(
    truth: Truth,
    unloaded_raw: f32,
    polarity: f32,
    raw_noise: f32,
    load_noise_kg: f32,
    seed: u64,
) -> Vec<Point> {
    let mut noise = Noise::new(seed);
    CALIBRATION_READINGS
        .iter()
        .map(|&u| {
            let raw = raw(unloaded_raw, polarity, u) + raw_noise * noise.next();
            // The zero point has no reference weight, so its load is exact.
            let load = if u == 0.0 {
                0.0
            } else {
                truth.load(f64::from(u)) as f32 + load_noise_kg * noise.next()
            };
            (raw, load)
        })
        .collect()
}

pub fn rms(values: impl IntoIterator<Item = f32>) -> f32 {
    let values: Vec<f32> = values.into_iter().collect();
    (values.iter().map(|v| v * v).sum::<f32>() / values.len() as f32).sqrt()
}
