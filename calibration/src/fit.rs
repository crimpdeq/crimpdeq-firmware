//! Least-squares fits of the calibration models, in f64 and with a fixed amount of work, so a fit
//! takes a bounded time on the device.

use arrayvec::ArrayVec;

use crate::{Curve, Error, Fit, MAX_POINTS, Model, Point, is_valid_factor};

/// Loads closer than this fraction of the load span count as one load level.
const LEVEL_TOLERANCE: f64 = 0.02;
/// The quadratic model needs a load level between these fractions of the load span.
const MID_RANGE: (f64, f64) = (0.25, 0.75);
/// Load levels that each side of the piecewise breakpoint needs.
const SEGMENT_LEVELS: usize = 3;
/// Smallest relative slope change between the piecewise segments that identifies a breakpoint.
const MIN_SLOPE_CHANGE: f64 = 0.01;
/// Breakpoint candidates evaluated across the search interval, before refining the best one.
const BREAKPOINT_GRID_STEPS: usize = 64;
/// Golden-section steps that refine the breakpoint around the best grid candidate.
const BREAKPOINT_REFINE_STEPS: usize = 32;

/// Fits the model to the points, or explains why the points do not define a valid calibration.
///
/// Points with zero load are the measured unloaded reference: the curve passes through their
/// average, which is `x = 0`. Without them, the curve has a free offset and `x = 0` is the
/// average raw value.
pub fn fit(model: Model, points: &[Point]) -> Result<Fit, Error> {
    if points
        .iter()
        .any(|(raw, weight)| !raw.is_finite() || !weight.is_finite())
    {
        return Err(Error::NonFinite);
    }
    let min_levels = model.min_load_levels();
    if points.len() < min_levels {
        return Err(Error::NotEnoughPoints);
    }
    // Before the coordinate, which a single load level does not define.
    let loads: ArrayVec<(f64, f64), MAX_POINTS> = points
        .iter()
        .map(|(_, weight)| (0.0, f64::from(*weight)))
        .collect();
    if load_levels(&loads).len() < min_levels {
        return Err(Error::NotEnoughLoadLevels);
    }
    let data = Data::new(points)?;

    let fit = match model {
        Model::Linear => data.linear(points)?,
        Model::Quadratic => data.quadratic()?,
        Model::Piecewise => data.piecewise()?,
    };
    fit.validate()?;
    Ok(fit)
}

/// Distinct load, with the average `x` and load of its points.
#[derive(Clone, Copy)]
struct Level {
    x: f64,
    weight: f64,
}

/// Points in the calibration coordinate.
struct Data {
    /// `(x, kg)` of each point.
    points: ArrayVec<(f64, f64), MAX_POINTS>,
    reference_raw: f64,
    /// Load at the reference: 0 for a measured zero, otherwise the average load.
    reference_weight: f64,
    /// Whether the points include zero load, which the curve then passes through.
    zero_anchored: bool,
    scale: f64,
    x_range: (f64, f64),
    /// Load levels in increasing load.
    levels: ArrayVec<Level, MAX_POINTS>,
}

impl Data {
    fn new(points: &[Point]) -> Result<Self, Error> {
        let raw_weight = || {
            points
                .iter()
                .map(|(raw, weight)| (f64::from(*raw), f64::from(*weight)))
        };
        let (zero_count, zero_raw_sum) = raw_weight()
            .filter(|(_, weight)| *weight == 0.0)
            .fold((0, 0.0), |(count, sum), (raw, _)| (count + 1, sum + raw));
        let (reference_raw, reference_weight, zero_anchored) = if zero_count > 0 {
            (zero_raw_sum / f64::from(zero_count), 0.0, true)
        } else {
            let n = points.len() as f64;
            let (raw_sum, weight_sum) = raw_weight()
                .fold((0.0, 0.0), |(raw_sum, weight_sum), (raw, weight)| {
                    (raw_sum + raw, weight_sum + weight)
                });
            (raw_sum / n, weight_sum / n, false)
        };

        // The sign of the load-raw trend through the reference is the wiring polarity.
        let trend: f64 = raw_weight()
            .map(|(raw, weight)| (raw - reference_raw) * (weight - reference_weight))
            .sum();
        let (raw_min, raw_max) = min_max(raw_weight().map(|(raw, _)| raw));
        let span = raw_max - raw_min;
        if span <= 0.0 || trend == 0.0 {
            return Err(Error::Degenerate);
        }
        let scale = if trend > 0.0 { span } else { -span };

        let xy: ArrayVec<(f64, f64), MAX_POINTS> = raw_weight()
            .map(|(raw, weight)| ((raw - reference_raw) / scale, weight))
            .collect();
        let x_range = min_max(xy.iter().map(|(x, _)| *x));
        let levels = load_levels(&xy);
        Ok(Self {
            points: xy,
            reference_raw,
            reference_weight,
            zero_anchored,
            scale,
            x_range,
            levels,
        })
    }

    fn fit(&self, curve: Curve) -> Fit {
        Fit {
            curve,
            reference_raw: self.reference_raw as f32,
            scale: self.scale as f32,
            x_min: self.x_range.0 as f32,
            x_max: self.x_range.1 as f32,
        }
    }

    /// The fit of earlier firmware: the least-squares slope of a line through the average
    /// zero-load point, or through the mean point when there is none, with raw values in counts
    /// and loads in grams.
    fn linear(&self, points: &[Point]) -> Result<Fit, Error> {
        let grams = || points.iter().map(|(_, weight)| f64::from(*weight) * 1000.0);
        // Averaged in grams, as earlier firmware did, so the factor rounds the same.
        let reference_grams = if self.zero_anchored {
            0.0
        } else {
            grams().sum::<f64>() / points.len() as f64
        };
        let (sum_delta_raw_weight, sum_delta_raw_sq) = points.iter().zip(grams()).fold(
            (0.0, 0.0),
            |(sum_delta_raw_weight, sum_delta_raw_sq), ((raw, _), grams)| {
                let delta_raw = f64::from(*raw) - self.reference_raw;
                let delta_weight = grams - reference_grams;
                (
                    sum_delta_raw_weight + delta_raw * delta_weight,
                    sum_delta_raw_sq + delta_raw * delta_raw,
                )
            },
        );
        if sum_delta_raw_sq < f64::EPSILON {
            return Err(Error::Degenerate);
        }
        let factor = (sum_delta_raw_weight / sum_delta_raw_sq) as f32;
        if !is_valid_factor(factor) {
            return Err(Error::ImplausibleSensitivity);
        }
        Ok(self.fit(Curve::Linear {
            factor,
            offset_kg: self.reference_weight as f32,
        }))
    }

    fn quadratic(&self) -> Result<Fit, Error> {
        let (low, high) = (
            self.levels[0].weight,
            self.levels[self.levels.len() - 1].weight,
        );
        let mid_range = (low + MID_RANGE.0 * (high - low))..=(low + MID_RANGE.1 * (high - low));
        if !self
            .levels
            .iter()
            .any(|level| mid_range.contains(&level.weight))
        {
            return Err(Error::NoMidRangeLevel);
        }

        let curve = if self.zero_anchored {
            let ([b, a], _) = least_squares(&self.points, |x| [x, x * x])?;
            Curve::Quadratic {
                a: a as f32,
                b: b as f32,
                c: 0.0,
            }
        } else {
            let ([c, b, a], _) = least_squares(&self.points, |x| [1.0, x, x * x])?;
            Curve::Quadratic {
                a: a as f32,
                b: b as f32,
                c: c as f32,
            }
        };
        Ok(self.fit(curve))
    }

    /// Searches the breakpoint between the third-lowest and third-highest load levels, so each
    /// segment has at least three levels: a grid search, then a golden-section refinement around
    /// the best grid candidate.
    fn piecewise(&self) -> Result<Fit, Error> {
        let low = self.levels[SEGMENT_LEVELS - 1].x;
        let high = self.levels[self.levels.len() - SEGMENT_LEVELS].x;
        if low >= high {
            return Err(Error::InsufficientSegmentSupport);
        }
        let candidate =
            |step: usize| low + (high - low) * step as f64 / BREAKPOINT_GRID_STEPS as f64;
        let sse = |breakpoint: f64| {
            self.segments(breakpoint)
                .map_or(f64::INFINITY, |(_, sse)| sse)
        };

        let (best_step, _) = (0..=BREAKPOINT_GRID_STEPS)
            .map(|step| (step, sse(candidate(step))))
            .fold((0, f64::INFINITY), |best, current| {
                if current.1 < best.1 { current } else { best }
            });
        let on_grid = candidate(best_step);
        let breakpoint = golden_section_minimum(
            (
                candidate(best_step.saturating_sub(1)),
                candidate((best_step + 1).min(BREAKPOINT_GRID_STEPS)),
            ),
            sse,
        );
        let (breakpoint, ([c, m1, m2], _)) = match self.segments(breakpoint) {
            Ok(segments) if segments.1 <= sse(on_grid) => (breakpoint, segments),
            _ => (on_grid, self.segments(on_grid)?),
        };
        // Straight points fit any breakpoint equally, so the search lands anywhere.
        if (m2 - m1).abs() < MIN_SLOPE_CHANGE * m1.abs().min(m2.abs()) {
            return Err(Error::NoBreakpoint);
        }
        // A minimum at the end of the search may lie past it, with too few levels on one side.
        if best_step == 0 || best_step == BREAKPOINT_GRID_STEPS {
            return Err(Error::InsufficientSegmentSupport);
        }
        Ok(self.fit(Curve::Piecewise {
            c: c as f32,
            m1: m1 as f32,
            m2: m2 as f32,
            breakpoint: breakpoint as f32,
        }))
    }

    /// Least-squares `[c, m1, m2]` of the two segments joined at the breakpoint, with their sum
    /// of squared residuals.
    fn segments(&self, breakpoint: f64) -> Result<([f64; 3], f64), Error> {
        let hinge = move |x: f64| (x - breakpoint).max(0.0);
        if self.zero_anchored {
            let ([m1, change], sse) = least_squares(&self.points, |x| [x, hinge(x)])?;
            Ok(([0.0, m1, m1 + change], sse))
        } else {
            let ([c, m1, change], sse) = least_squares(&self.points, |x| [1.0, x, hinge(x)])?;
            Ok(([c, m1, m1 + change], sse))
        }
    }
}

/// Groups the points into load levels, in increasing load.
fn load_levels(points: &[(f64, f64)]) -> ArrayVec<Level, MAX_POINTS> {
    let mut sorted: ArrayVec<(f64, f64), MAX_POINTS> = points.iter().copied().collect();
    sorted.sort_unstable_by(|a, b| a.1.total_cmp(&b.1));
    let (low, high) = min_max(sorted.iter().map(|(_, weight)| *weight));
    let tolerance = LEVEL_TOLERANCE * (high - low);

    let mut levels = ArrayVec::new();
    let mut start = 0;
    while start < sorted.len() {
        let end = sorted[start..]
            .iter()
            .position(|(_, weight)| weight - sorted[start].1 > tolerance)
            .map_or(sorted.len(), |offset| start + offset);
        let group = &sorted[start..end];
        let n = group.len() as f64;
        levels.push(Level {
            x: group.iter().map(|(x, _)| x).sum::<f64>() / n,
            weight: group.iter().map(|(_, weight)| weight).sum::<f64>() / n,
        });
        start = end;
    }
    levels
}

/// Least-squares coefficients of the basis functions, by Gaussian elimination of the normal
/// equations, with the sum of squared residuals. `x` spans at most 1, so the normal equations of
/// these small bases are well conditioned unless the points do not determine the coefficients.
fn least_squares<const N: usize>(
    points: &[(f64, f64)],
    basis: impl Fn(f64) -> [f64; N],
) -> Result<([f64; N], f64), Error> {
    let mut matrix = [[0.0; N]; N];
    let mut rhs = [0.0; N];
    for &(x, y) in points {
        let row = basis(x);
        for i in 0..N {
            rhs[i] += row[i] * y;
            for j in 0..N {
                matrix[i][j] += row[i] * row[j];
            }
        }
    }
    let diagonal_scale = (0..N).map(|i| matrix[i][i]).fold(0.0, f64::max);

    for column in 0..N {
        let pivot = (column..N)
            .max_by(|a, b| {
                matrix[*a][column]
                    .abs()
                    .total_cmp(&matrix[*b][column].abs())
            })
            .expect("the range is not empty");
        let magnitude = matrix[pivot][column].abs();
        if magnitude.is_nan() || magnitude <= 1e-12 * diagonal_scale {
            return Err(Error::Degenerate);
        }
        matrix.swap(column, pivot);
        rhs.swap(column, pivot);
        for row in column + 1..N {
            let ratio = matrix[row][column] / matrix[column][column];
            let (above, below) = matrix.split_at_mut(row);
            for (target, pivot_value) in below[0][column..].iter_mut().zip(&above[column][column..])
            {
                *target -= ratio * pivot_value;
            }
            rhs[row] -= ratio * rhs[column];
        }
    }
    let mut coefficients = [0.0; N];
    for row in (0..N).rev() {
        let known: f64 = (row + 1..N).map(|k| matrix[row][k] * coefficients[k]).sum();
        coefficients[row] = (rhs[row] - known) / matrix[row][row];
    }

    let sse = points
        .iter()
        .map(|&(x, y)| {
            let fitted: f64 = basis(x)
                .iter()
                .zip(coefficients)
                .map(|(value, coefficient)| value * coefficient)
                .sum();
            (fitted - y) * (fitted - y)
        })
        .sum();
    if coefficients.iter().all(|value| value.is_finite()) {
        Ok((coefficients, sse))
    } else {
        Err(Error::NonFinite)
    }
}

/// Minimum of `f` in the interval by golden-section search.
fn golden_section_minimum(mut interval: (f64, f64), f: impl Fn(f64) -> f64) -> f64 {
    const INVERSE_PHI: f64 = 0.618_033_988_749_895;
    let probe = |(low, high): (f64, f64)| {
        (
            high - INVERSE_PHI * (high - low),
            low + INVERSE_PHI * (high - low),
        )
    };
    let (mut left, mut right) = probe(interval);
    let (mut f_left, mut f_right) = (f(left), f(right));
    for _ in 0..BREAKPOINT_REFINE_STEPS {
        if f_left <= f_right {
            interval.1 = right;
            (right, f_right) = (left, f_left);
            left = probe(interval).0;
            f_left = f(left);
        } else {
            interval.0 = left;
            (left, f_left) = (right, f_right);
            right = probe(interval).1;
            f_right = f(right);
        }
    }
    (interval.0 + interval.1) / 2.0
}

fn min_max(values: impl Iterator<Item = f64>) -> (f64, f64) {
    values.fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), value| {
        (low.min(value), high.max(value))
    })
}
