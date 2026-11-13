//! Calibration in use, with the points that edit it and their rules.

use arrayvec::ArrayVec;

use crate::{Calibration, Error, MAX_POINTS, Model, Outcome, Point, fit};

/// Calibration in use and the stored points, as persisted to flash.
///
/// The model in use is the selected model: selecting a model that does not fit the points changes
/// nothing. Editing the points refits the model in use. When the edited points cannot be fit, the
/// previous calibration stays in use and `pending` reports why, until an edit or a model selection
/// fits the points.
#[derive(Clone, Debug, PartialEq)]
pub struct CalibrationRecord {
    pub active: Calibration,
    pub points: ArrayVec<Point, MAX_POINTS>,
    /// Why `active` is not the fit of `points`, or `None` when it is, or when there are no points
    /// and `active` is the default or a factor of earlier firmware.
    pub pending: Option<Error>,
}

/// Calibration-fit residuals of the stored points against the calibration in use, in kg. They
/// show how well the curve follows the points, not the accuracy of other loads.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Diagnostics {
    pub rms_kg: f32,
    pub max_abs_kg: f32,
}

impl Default for CalibrationRecord {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl CalibrationRecord {
    pub const DEFAULT: Self = Self {
        active: Calibration::DEFAULT,
        points: ArrayVec::new_const(),
        pending: None,
    };

    pub fn model(&self) -> Model {
        self.active.model()
    }

    /// Adds a point and refits the model in use.
    ///
    /// A point that leaves the fit short of data is stored, so the calibration can grow to the
    /// points the model needs. A point that makes a fit of applied points invalid is rejected,
    /// so a wrong point, such as a weight that was not hung, cannot skew the calibration. Points
    /// that are already pending were never validated together, so the new point joins them.
    pub fn add_point(&mut self, point: Point) -> Outcome {
        if self.points.is_full() {
            return Outcome::Rejected(Error::PointLimitReached);
        }
        if !point.0.is_finite() || !point.1.is_finite() {
            return Outcome::Rejected(Error::NonFinite);
        }
        let mut points = self.points.clone();
        points.push(point);
        match fit(self.model(), &points) {
            Ok(fit) => self.apply(points, Calibration::Fitted(fit)),
            Err(error) if error.needs_more_data() || self.pending.is_some() => {
                self.points = points;
                self.pending = Some(error);
                Outcome::Pending(error)
            }
            Err(error) => Outcome::Rejected(error),
        }
    }

    /// Removes a point and refits the model in use. The removal is kept when the remaining
    /// points cannot be fit; the previous calibration then stays in use.
    pub fn remove_point(&mut self, index: usize) -> Outcome {
        if index >= self.points.len() {
            return Outcome::Rejected(Error::InvalidIndex);
        }
        let mut points = self.points.clone();
        points.remove(index);
        match fit(self.model(), &points) {
            Ok(fit) => self.apply(points, Calibration::Fitted(fit)),
            Err(error) => {
                self.points = points;
                self.pending = Some(error);
                Outcome::Pending(error)
            }
        }
    }

    /// Fits the model to the stored points and uses it, or changes nothing when it does not fit.
    pub fn select_model(&mut self, model: Model) -> Outcome {
        match fit(model, &self.points) {
            Ok(fit) => self.apply(self.points.clone(), Calibration::Fitted(fit)),
            Err(error) => Outcome::Rejected(error),
        }
    }

    /// Restores the default linear calibration and deletes the points.
    pub fn reset(&mut self) {
        *self = Self::DEFAULT;
    }

    fn apply(&mut self, points: ArrayVec<Point, MAX_POINTS>, active: Calibration) -> Outcome {
        self.points = points;
        self.active = active;
        self.pending = None;
        Outcome::Applied
    }

    /// Load of each point by the calibration in use, `None` without a fit.
    pub fn fitted_kg(&self) -> impl Iterator<Item = Option<f32>> + '_ {
        self.points
            .iter()
            .map(|(raw, _)| self.active.fit().map(|fit| fit.kg(*raw)))
    }

    /// Residuals of the points, `None` without points or a fit.
    pub fn diagnostics(&self) -> Option<Diagnostics> {
        let fit = self.active.fit()?;
        if self.points.is_empty() {
            return None;
        }
        let (sum_sq, max_abs) = self
            .points
            .iter()
            .map(|(raw, weight)| f64::from(fit.kg(*raw)) - f64::from(*weight))
            .fold((0.0, 0.0_f64), |(sum_sq, max_abs), residual| {
                (sum_sq + residual * residual, max_abs.max(residual.abs()))
            });
        Some(Diagnostics {
            rms_kg: sqrt(sum_sq / self.points.len() as f64) as f32,
            max_abs_kg: max_abs as f32,
        })
    }
}

/// Square root by Newton's method, which `core` lacks.
fn sqrt(value: f64) -> f64 {
    if value <= 0.0 {
        return 0.0;
    }
    let mut root = value.max(1.0);
    for _ in 0..64 {
        root = 0.5 * (root + value / root);
    }
    root
}
