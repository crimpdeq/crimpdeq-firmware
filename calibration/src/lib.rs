//! Calibration models of the Crimpdeq load cell: fitting, evaluation, the edit rules of the stored
//! calibration points, the flash record, and the BLE payloads that describe them.
//!
//! The crate has no hardware dependencies, so `cargo test-host` runs its tests on the host.
//!
//! # Units and coordinates
//!
//! - `raw`: ADS1220 code, or an average of codes, before the tare.
//! - Loads are in kg. The factor of the linear model is in g per raw count, as in earlier firmware.
//! - Nonlinear models are fit and evaluated in the calibration coordinate
//!   `x = (raw - reference_raw) / scale`:
//!   - `reference_raw` is the average raw code of the zero-load points, the measured unloaded
//!     reference, or the average raw code of all points when there is none.
//!   - `scale` is the raw span of the points, with the sign of the load-cell wiring, so `x` grows
//!     with the load for both wiring polarities and the points span at most 1.
//!
//!   Curve coefficients are in kg per unit of `x` (`x` itself has no unit).
#![no_std]

mod fit;
pub mod persist;
mod record;
pub mod wire;

pub use fit::fit;
pub use record::{CalibrationRecord, Diagnostics};

/// Calibration points stored on the device.
pub const MAX_POINTS: usize = 20;

/// Factor, in g per raw count, of the default calibration.
pub const DEFAULT_FACTOR: f32 = 0.109;

/// How many times above or below [`DEFAULT_FACTOR`] a sensitivity can be before it is rejected as
/// implausible for the hardware, such as a fit of points whose load did not change.
const SENSITIVITY_TOLERANCE: f32 = 10.0;

/// Largest ratio between the sensitivities at the two ends of the calibrated range. A real load
/// cell changes its sensitivity by a few percent, so a larger bend is a bad point.
pub const MAX_SENSITIVITY_RATIO: f32 = 1.5;

/// Largest load, in kg, that a zero update treats as baseline drift instead of an attached load.
pub const MAX_ZERO_LOAD_KG: f32 = 1.0;

/// A raw ADC value, before the tare, and the known load in kg that produced it.
pub type Point = (f32, f32);

/// Calibration model that the user selects.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Model {
    /// `kg = offset + factor * (raw - reference) / 1000`, the model of earlier firmware.
    Linear = 0,
    /// `kg = a*x² + b*x + c`, extended past the calibrated range with its boundary slopes.
    Quadratic = 1,
    /// `kg = c + m1*x + (m2 - m1)*max(0, x - breakpoint)`: two straight segments joined at the
    /// breakpoint.
    Piecewise = 2,
}

impl Model {
    pub const ALL: [Self; 3] = [Self::Linear, Self::Quadratic, Self::Piecewise];

    pub fn from_code(code: u8) -> Option<Self> {
        Self::ALL.into_iter().find(|model| *model as u8 == code)
    }

    /// Points and distinct load levels, zero included, that a fit needs. Each is one more than
    /// the free parameters of the model, so the residuals can show a bad fit, and the piecewise
    /// model needs three levels on each side of its breakpoint.
    pub const fn min_load_levels(self) -> usize {
        match self {
            Self::Linear => 2,
            Self::Quadratic => 4,
            Self::Piecewise => 6,
        }
    }
}

/// Why a calibration command or fit failed. The discriminant is its BLE code.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Error {
    /// Fewer points than [`Model::min_load_levels`].
    NotEnoughPoints = 1,
    /// Fewer distinct load levels than [`Model::min_load_levels`]. Loads closer than 2 % of the
    /// load span are one level.
    NotEnoughLoadLevels = 2,
    /// The quadratic model has no load level between 25 % and 75 % of the load span, which
    /// defines its curvature.
    NoMidRangeLevel = 3,
    /// The best breakpoint has fewer than three load levels on one side.
    InsufficientSegmentSupport = 4,
    /// The two segments differ in slope by less than 1 %, so the breakpoint is arbitrary.
    NoBreakpoint = 5,
    /// A sensitivity is more than 10 times above or below the default.
    ImplausibleSensitivity = 6,
    /// The sensitivities at the two ends of the calibrated range differ by more than
    /// [`MAX_SENSITIVITY_RATIO`].
    ExcessiveNonlinearity = 7,
    /// The load would decrease as the raw value moves with the load.
    NotMonotonic = 8,
    /// A point or coefficient is not finite.
    NonFinite = 9,
    /// The points do not determine the curve, such as points with the same raw value.
    Degenerate = 10,
    /// The device does not support the requested model.
    UnsupportedModel = 11,
    /// [`MAX_POINTS`] points are stored.
    PointLimitReached = 12,
    /// No point has the index.
    InvalidIndex = 13,
    /// The ADC failed to sample.
    SamplingFailed = 14,
    /// A zero update read more than [`MAX_ZERO_LOAD_KG`].
    ZeroOutOfRange = 15,
}

impl Error {
    const ALL: [Self; 15] = [
        Self::NotEnoughPoints,
        Self::NotEnoughLoadLevels,
        Self::NoMidRangeLevel,
        Self::InsufficientSegmentSupport,
        Self::NoBreakpoint,
        Self::ImplausibleSensitivity,
        Self::ExcessiveNonlinearity,
        Self::NotMonotonic,
        Self::NonFinite,
        Self::Degenerate,
        Self::UnsupportedModel,
        Self::PointLimitReached,
        Self::InvalidIndex,
        Self::SamplingFailed,
        Self::ZeroOutOfRange,
    ];

    pub fn from_code(code: u8) -> Option<Self> {
        Self::ALL.into_iter().find(|error| *error as u8 == code)
    }

    /// Whether more or better spread points can resolve the error, rather than the points being
    /// wrong.
    pub fn needs_more_data(self) -> bool {
        matches!(
            self,
            Self::NotEnoughPoints
                | Self::NotEnoughLoadLevels
                | Self::NoMidRangeLevel
                | Self::InsufficientSegmentSupport
        )
    }
}

/// Result of a calibration command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Outcome {
    /// The calibration fit from the stored points is in use.
    Applied,
    /// The points were stored but cannot be fit yet, so the previous calibration stays in use.
    Pending(Error),
    /// Nothing changed.
    Rejected(Error),
}

/// Calibration that converts raw ADC values to kg.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Calibration {
    /// Linear factor in g per raw count with no fit behind it: the default, or a record of earlier
    /// firmware, which stored only the factor. It has no reference or calibrated range.
    Factor(f32),
    /// Calibration fit from points.
    Fitted(Fit),
}

/// Calibration fit from points.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Fit {
    pub curve: Curve,
    /// Raw value at `x = 0`.
    pub reference_raw: f32,
    /// Signed raw counts per unit of `x`.
    pub scale: f32,
    /// Calibrated range, in `x`, of the points of the fit.
    pub x_min: f32,
    pub x_max: f32,
}

/// Curve of a fit, in kg.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Curve {
    /// `kg = offset_kg + factor * (raw - reference_raw) / 1000`, with the factor in g per count.
    Linear { factor: f32, offset_kg: f32 },
    /// `kg = a*x² + b*x + c` inside the calibrated range.
    Quadratic { a: f32, b: f32, c: f32 },
    /// `kg = c + m1*x + (m2 - m1)*max(0, x - breakpoint)`.
    Piecewise {
        c: f32,
        m1: f32,
        m2: f32,
        breakpoint: f32,
    },
}

impl Calibration {
    pub const DEFAULT: Self = Self::Factor(DEFAULT_FACTOR);

    pub fn model(&self) -> Model {
        match self {
            Self::Factor(_) => Model::Linear,
            Self::Fitted(fit) => fit.model(),
        }
    }

    pub fn fit(&self) -> Option<&Fit> {
        match self {
            Self::Factor(_) => None,
            Self::Fitted(fit) => Some(fit),
        }
    }

    /// Factor for clients of the earlier linear-only protocol. NaN for a nonlinear model, which no
    /// single factor describes.
    pub fn legacy_factor(&self) -> f32 {
        match self {
            Self::Factor(factor)
            | Self::Fitted(Fit {
                curve: Curve::Linear { factor, .. },
                ..
            }) => *factor,
            Self::Fitted(_) => f32::NAN,
        }
    }

    /// Load in kg on top of the load at the tare: `f(raw - shift) - f(tare - shift)`, where the
    /// zero shift is the baseline drift measured by a zero update.
    ///
    /// Evaluating the curve at both values, instead of at `raw - tare`, keeps the breakpoint and
    /// slopes of a nonlinear curve at the loads where they were calibrated, also when tared under
    /// a preload. A linear curve gives the same value either way, so it keeps the conversion of
    /// earlier firmware.
    pub fn net_kg(&self, raw: i32, tare: i32, zero_shift: f32) -> f32 {
        match self {
            Self::Factor(factor)
            | Self::Fitted(Fit {
                curve: Curve::Linear { factor, .. },
                ..
            }) => (raw - tare) as f32 * factor / 1000.0,
            Self::Fitted(fit) => fit.kg(raw as f32 - zero_shift) - fit.kg(tare as f32 - zero_shift),
        }
    }

    /// Zero shift that makes `unloaded_raw` read 0 kg, so the curve follows the drift of the
    /// unloaded baseline. A factor-only calibration has no reference, and a linear curve reads
    /// the same with any shift, so only nonlinear fits depend on it.
    pub fn zero_shift(&self, unloaded_raw: f32) -> Result<f32, Error> {
        let Self::Fitted(fit) = self else {
            return Ok(0.0);
        };
        let load = fit.kg(unloaded_raw);
        if load.is_nan() || load.abs() > MAX_ZERO_LOAD_KG {
            return Err(Error::ZeroOutOfRange);
        }
        Ok(unloaded_raw - fit.raw_at_zero_load())
    }
}

impl Fit {
    pub fn model(&self) -> Model {
        match self.curve {
            Curve::Linear { .. } => Model::Linear,
            Curve::Quadratic { .. } => Model::Quadratic,
            Curve::Piecewise { .. } => Model::Piecewise,
        }
    }

    pub fn x(&self, raw: f32) -> f32 {
        (raw - self.reference_raw) / self.scale
    }

    /// Absolute load in kg at a raw value, with no tare.
    pub fn kg(&self, raw: f32) -> f32 {
        match self.curve {
            Curve::Linear { factor, offset_kg } => {
                offset_kg + (raw - self.reference_raw) * factor / 1000.0
            }
            _ => self.kg_at(self.x(raw)),
        }
    }

    /// Load in kg at `x`. Past the calibrated range, the curve continues as a straight line with
    /// the slope at the end of the range, so it cannot turn back or grow faster than calibrated.
    /// Linear and piecewise curves are straight there already.
    pub fn kg_at(&self, x: f32) -> f32 {
        match self.curve {
            Curve::Linear { factor, offset_kg } => offset_kg + factor * self.scale * x / 1000.0,
            Curve::Quadratic { a, b, c } => {
                let end = x.clamp(self.x_min, self.x_max);
                (a * end + b) * end + c + (2.0 * a * end + b) * (x - end)
            }
            Curve::Piecewise {
                c,
                m1,
                m2,
                breakpoint,
            } => c + m1 * x + (m2 - m1) * (x - breakpoint).max(0.0),
        }
    }

    /// Slope in kg per unit of `x`, for `x` inside the calibrated range, and at its ends the
    /// slope of the straight extension.
    fn slope_at(&self, x: f32) -> f32 {
        match self.curve {
            Curve::Linear { factor, .. } => factor * self.scale / 1000.0,
            Curve::Quadratic { a, b, .. } => 2.0 * a * x.clamp(self.x_min, self.x_max) + b,
            Curve::Piecewise {
                m1, m2, breakpoint, ..
            } => {
                if x < breakpoint {
                    m1
                } else {
                    m2
                }
            }
        }
    }

    /// Loads in kg at the ends of the calibrated range.
    pub fn load_range_kg(&self) -> (f32, f32) {
        (self.kg_at(self.x_min), self.kg_at(self.x_max))
    }

    /// Raw value where the curve reads 0 kg, by Newton's method. The curve is increasing with
    /// bounded slopes, so a few steps converge.
    fn raw_at_zero_load(&self) -> f32 {
        let mut x = 0.0;
        for _ in 0..16 {
            x -= self.kg_at(x) / self.slope_at(x);
        }
        self.reference_raw + x * self.scale
    }

    /// Model coefficients in the order of the BLE payload: linear `[factor, offset_kg]`,
    /// quadratic `[a, b, c]`, piecewise `[c, m1, m2, breakpoint]`, padded with zeros.
    pub fn coefficients(&self) -> [f32; 4] {
        match self.curve {
            Curve::Linear { factor, offset_kg } => [factor, offset_kg, 0.0, 0.0],
            Curve::Quadratic { a, b, c } => [a, b, c, 0.0],
            Curve::Piecewise {
                c,
                m1,
                m2,
                breakpoint,
            } => [c, m1, m2, breakpoint],
        }
    }

    /// Rebuilds a fit from its model and [`Self::coefficients`].
    pub fn from_coefficients(
        model: Model,
        [c0, c1, c2, c3]: [f32; 4],
        reference_raw: f32,
        scale: f32,
        (x_min, x_max): (f32, f32),
    ) -> Self {
        let curve = match model {
            Model::Linear => Curve::Linear {
                factor: c0,
                offset_kg: c1,
            },
            Model::Quadratic => Curve::Quadratic {
                a: c0,
                b: c1,
                c: c2,
            },
            Model::Piecewise => Curve::Piecewise {
                c: c0,
                m1: c1,
                m2: c2,
                breakpoint: c3,
            },
        };
        Self {
            curve,
            reference_raw,
            scale,
            x_min,
            x_max,
        }
    }

    /// Checks that the fit is finite, increasing with the load, and plausible for the hardware
    /// over the whole raw range. Past the calibrated range the curve keeps its boundary slopes, so
    /// checking them covers every reading.
    pub fn validate(&self) -> Result<(), Error> {
        let finite = self.coefficients().iter().all(|value| value.is_finite())
            && [self.reference_raw, self.scale, self.x_min, self.x_max]
                .iter()
                .all(|value| value.is_finite());
        if !finite {
            return Err(Error::NonFinite);
        }
        if self.scale == 0.0 || self.x_min > self.x_max {
            return Err(Error::Degenerate);
        }
        match self.curve {
            // Earlier firmware accepted a factor of either sign, for either wiring polarity, and
            // so does `scale` here.
            Curve::Linear { factor, .. } => {
                return if is_valid_factor(factor) {
                    Ok(())
                } else {
                    Err(Error::ImplausibleSensitivity)
                };
            }
            Curve::Piecewise { breakpoint, .. }
                if !(self.x_min..=self.x_max).contains(&breakpoint) =>
            {
                return Err(Error::InsufficientSegmentSupport);
            }
            _ => {}
        }

        let slopes = match self.curve {
            Curve::Piecewise { m1, m2, .. } => [m1, m2],
            _ => [self.slope_at(self.x_min), self.slope_at(self.x_max)],
        };
        if slopes.iter().any(|slope| *slope <= 0.0) {
            return Err(Error::NotMonotonic);
        }
        // Slopes in g per raw count, like the linear factor.
        let sensitivities = slopes.map(|slope| slope * 1000.0 / self.scale.abs());
        if !sensitivities
            .iter()
            .all(|sensitivity| is_valid_factor(*sensitivity))
        {
            return Err(Error::ImplausibleSensitivity);
        }
        let [low, high] = sensitivities;
        if low.max(high) > MAX_SENSITIVITY_RATIO * low.min(high) {
            return Err(Error::ExcessiveNonlinearity);
        }
        Ok(())
    }
}

/// Checks whether a linear factor, in g per count, is plausible for the hardware. Either sign is
/// accepted, so a load cell wired in reverse can still be calibrated.
pub fn is_valid_factor(factor: f32) -> bool {
    (DEFAULT_FACTOR / SENSITIVITY_TOLERANCE..=DEFAULT_FACTOR * SENSITIVITY_TOLERANCE)
        .contains(&factor.abs())
}
