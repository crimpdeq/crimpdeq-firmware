//! Fitting and evaluation of the calibration models.

mod common;

use common::{
    CALIBRATION_READINGS,
    HELD_OUT_READINGS,
    Truth,
    raw,
    rms,
    session_points,
    synthetic_points,
};
use crimpdeq_calibration::{Calibration, Curve, DEFAULT_FACTOR, Error, Fit, Model, Point, fit};

/// The fit of firmware 3.0, kept verbatim to check that the linear model still matches it.
fn legacy_fit_calibration_factor(calibration_points: &[(f32, f32)]) -> Option<f32> {
    if calibration_points.len() < 2 {
        return None;
    }
    let points = || {
        calibration_points
            .iter()
            .map(|(raw_value, weight)| (f64::from(*raw_value), f64::from(*weight) * 1000.0))
    };
    let mean_point = |points: &mut dyn Iterator<Item = (f64, f64)>| {
        let (count, sum_raw, sum_weight) = points.fold(
            (0usize, 0.0, 0.0),
            |(count, sum_raw, sum_weight), (raw, weight)| {
                (count + 1, sum_raw + raw, sum_weight + weight)
            },
        );
        (count > 0).then(|| (sum_raw / count as f64, sum_weight / count as f64))
    };
    let (anchor_raw, anchor_weight) =
        match mean_point(&mut points().filter(|&(_, weight)| weight == 0.0)) {
            Some(zero_point) => zero_point,
            None => mean_point(&mut points()).unwrap(),
        };
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
        return None;
    }
    let factor = (sum_delta_raw_weight / sum_delta_raw_sq) as f32;
    ((DEFAULT_FACTOR / 10.0..=DEFAULT_FACTOR * 10.0).contains(&factor.abs())).then_some(factor)
}

fn linear_factor(fit: &Fit) -> f32 {
    match fit.curve {
        Curve::Linear { factor, .. } => factor,
        _ => panic!("not linear: {fit:?}"),
    }
}

fn breakpoint(fit: &Fit) -> f32 {
    match fit.curve {
        Curve::Piecewise { breakpoint, .. } => breakpoint,
        _ => panic!("not piecewise: {fit:?}"),
    }
}

fn residuals(fit: &Fit, points: &[Point]) -> Vec<f32> {
    points
        .iter()
        .map(|(raw, load)| fit.kg(*raw) - load)
        .collect()
}

fn max_abs(values: &[f32]) -> f32 {
    values.iter().fold(0.0, |max, value| max.max(value.abs()))
}

#[test]
fn linear_matches_the_earlier_firmware() {
    let sets: Vec<Vec<Point>> = vec![
        vec![(1000.0, 0.0), (92_743.0, 10.0)],
        // Two zero points and no zero point.
        vec![
            (900.0, 0.0),
            (1100.0, 0.0),
            (92_743.0, 10.0),
            (184_000.0, 20.3),
        ],
        vec![(92_743.0, 10.0), (184_000.0, 20.3), (277_000.0, 30.1)],
        // Reversed wiring.
        vec![(-1000.0, 0.0), (-93_000.0, 10.0), (-460_000.0, 50.0)],
        session_points(12_345.0, 1.0),
        session_points(-4_000.0, -1.0),
        synthetic_points(Truth::Quadratic, 2_000.0, 1.0, 30.0, 0.05, 7),
    ];
    for points in sets {
        let legacy = legacy_fit_calibration_factor(&points).unwrap();
        let fit = fit(Model::Linear, &points).unwrap();
        assert_eq!(
            linear_factor(&fit).to_bits(),
            legacy.to_bits(),
            "{points:?}"
        );
    }
}

#[test]
fn linear_rejects_what_the_earlier_firmware_rejected() {
    // One point, one load, the same raw value, and a factor 20 times the default.
    for points in [
        vec![(1000.0, 0.0)],
        vec![(1000.0, 5.0), (2000.0, 5.0)],
        vec![(1000.0, 0.0), (1000.0, 5.0)],
        vec![(0.0, 0.0), (2294.0, 5.0)],
    ] {
        assert_eq!(legacy_fit_calibration_factor(&points), None);
        assert!(fit(Model::Linear, &points).is_err(), "{points:?}");
    }
}

#[test]
fn linear_readings_match_the_earlier_conversion() {
    let points = session_points(5_000.0, 1.0);
    let fitted = Calibration::Fitted(fit(Model::Linear, &points).unwrap());
    let factor = fitted.legacy_factor();
    for (raw, tare) in [
        (812_345, 5_012),
        (-3, 100),
        (5_000, 5_000),
        (8_000_000, -8_000_000),
    ] {
        let legacy = (raw - tare) as f32 * factor / 1000.0;
        // The zero shift of a zero update does not change a linear reading.
        for zero_shift in [0.0, 250.0] {
            assert_eq!(
                fitted.net_kg(raw, tare, zero_shift).to_bits(),
                legacy.to_bits()
            );
            assert_eq!(
                Calibration::Factor(factor)
                    .net_kg(raw, tare, zero_shift)
                    .to_bits(),
                legacy.to_bits()
            );
        }
    }
}

#[test]
fn fits_synthetic_responses_at_held_out_loads() {
    // Loads of each model and response: (model, response, largest held-out error in kg).
    let cases = [
        (Model::Linear, Truth::Linear, 0.01),
        (Model::Quadratic, Truth::Linear, 0.01),
        (Model::Quadratic, Truth::Quadratic, 0.01),
        (Model::Piecewise, Truth::Piecewise, 0.01),
    ];
    for (model, truth, tolerance) in cases {
        for (unloaded_raw, polarity) in [(0.0, 1.0), (25_000.0, 1.0), (-40_000.0, -1.0)] {
            let points = synthetic_points(truth, unloaded_raw, polarity, 0.0, 0.0, 0);
            let fit = fit(model, &points).unwrap_or_else(|e| panic!("{model:?}: {e:?}"));
            assert_eq!(fit.model(), model);
            for u in HELD_OUT_READINGS {
                let expected = truth.load(f64::from(u)) as f32;
                let fitted = fit.kg(raw(unloaded_raw, polarity, u));
                assert!(
                    (fitted - expected).abs() < tolerance,
                    "{model:?} polarity {polarity}: {fitted} vs {expected} kg at {u}"
                );
            }
        }
    }
}

#[test]
fn noisy_points_fit_within_the_noise() {
    let cases = [
        (Model::Linear, Truth::Linear),
        (Model::Quadratic, Truth::Quadratic),
        (Model::Piecewise, Truth::Piecewise),
    ];
    for (model, truth) in cases {
        for seed in 1..=10 {
            // 20 counts are about 2 g, below the noise of the reference weights.
            let points = synthetic_points(truth, 3_000.0, -1.0, 20.0, 0.03, seed);
            let fit = fit(model, &points).unwrap_or_else(|e| panic!("{model:?} {seed}: {e:?}"));
            let errors: Vec<f32> = HELD_OUT_READINGS
                .iter()
                .map(|&u| fit.kg(raw(3_000.0, -1.0, u)) - truth.load(f64::from(u)) as f32)
                .collect();
            assert!(max_abs(&errors) < 0.08, "{model:?} {seed}: {errors:?}");
        }
    }
}

#[test]
fn the_linear_model_misses_a_nonlinear_response() {
    let points = synthetic_points(Truth::Piecewise, 0.0, 1.0, 0.0, 0.0, 0);
    let linear = fit(Model::Linear, &points).unwrap();
    let piecewise = fit(Model::Piecewise, &points).unwrap();

    assert!(rms(residuals(&linear, &points)) > 0.5);
    assert!(rms(residuals(&piecewise, &points)) < 0.005);
    // The breakpoint is where the sensitivity steps up.
    let breakpoint_kg = piecewise.kg_at(breakpoint(&piecewise));
    assert!((breakpoint_kg - 48.0).abs() < 0.1, "{breakpoint_kg}");
}

#[test]
fn session_regression() {
    for (unloaded_raw, polarity) in [(0.0, 1.0), (61_000.0, 1.0), (-15_000.0, -1.0)] {
        let points = session_points(unloaded_raw, polarity);
        let linear = fit(Model::Linear, &points).unwrap();
        let quadratic = fit(Model::Quadratic, &points).unwrap();
        let piecewise = fit(Model::Piecewise, &points).unwrap();

        let linear_residuals = residuals(&linear, &points);
        let quadratic_residuals = residuals(&quadratic, &points);
        let piecewise_residuals = residuals(&piecewise, &points);
        // About 1.07, 0.24 and 0.087 kg RMS with a zero-constrained least-squares fit.
        assert!(rms(linear_residuals.iter().copied()) > 0.9);
        assert!((0.2..0.3).contains(&rms(quadratic_residuals.iter().copied())));
        assert!(max_abs(&quadratic_residuals) < 0.6);
        assert!(rms(piecewise_residuals.iter().copied()) < 0.1);
        assert!(max_abs(&piecewise_residuals) < 0.2);

        // The sensitivity changes around 45-50 kg.
        let breakpoint_kg = piecewise.kg_at(breakpoint(&piecewise));
        assert!((44.0..51.0).contains(&breakpoint_kg), "{breakpoint_kg}");
        // Zero-constrained: the unloaded point reads 0.
        assert!(quadratic.kg(unloaded_raw).abs() < 1e-4);
        assert!(piecewise.kg(unloaded_raw).abs() < 1e-4);
    }
}

#[test]
fn session_regression_without_the_zero_point() {
    let points: Vec<Point> = session_points(0.0, 1.0)
        .into_iter()
        .filter(|(_, load)| *load > 0.0)
        .collect();
    let quadratic = fit(Model::Quadratic, &points).unwrap();
    let piecewise = fit(Model::Piecewise, &points).unwrap();

    assert!(rms(residuals(&quadratic, &points)) < 0.3);
    assert!(rms(residuals(&piecewise, &points)) < 0.1);
    // Without a measured zero, the offset is fit, so the unloaded code reads close to 0.
    assert!(piecewise.kg(0.0).abs() < 0.3, "{}", piecewise.kg(0.0));
}

#[test]
fn the_piecewise_curve_is_continuous_at_the_breakpoint() {
    let fit = fit(Model::Piecewise, &session_points(0.0, 1.0)).unwrap();
    let Curve::Piecewise {
        m1, m2, breakpoint, ..
    } = fit.curve
    else {
        unreachable!()
    };
    // Across the breakpoint, the curve changes by no more than its slopes allow: no jump.
    for epsilon in [1e-2, 1e-4] {
        let change = fit.kg_at(breakpoint + epsilon) - fit.kg_at(breakpoint - epsilon);
        assert!(change > 0.0);
        assert!(change <= m1.max(m2) * 2.0 * epsilon + 1e-4, "{change}");
        assert!(change >= m1.min(m2) * 2.0 * epsilon - 1e-4, "{change}");
    }
}

/// A quadratic whose slope falls with the load, so past the calibration it would turn back.
fn concave_quadratic() -> Fit {
    // 0.1 g per count at x = 0, falling to 0.08 at x = 1, with 10^6 counts per x.
    Fit {
        curve: Curve::Quadratic {
            a: -10.0,
            b: 100.0,
            c: 0.0,
        },
        reference_raw: 0.0,
        scale: 1_000_000.0,
        x_min: 0.0,
        x_max: 1.0,
    }
}

#[test]
fn quadratic_extrapolation_continues_with_the_boundary_slope() {
    let fit = concave_quadratic();
    fit.validate().unwrap();
    let (low_kg, high_kg) = fit.load_range_kg();
    assert_eq!((low_kg, high_kg), (0.0, 90.0));

    // Continuous at both ends, and straight past them with the slope there: 80 kg per x at
    // x = 1 and 100 kg per x at x = 0.
    assert!((fit.kg_at(1.0 + 1e-4) - 90.0).abs() < 0.01);
    assert!((fit.kg_at(3.0) - (90.0 + 2.0 * 80.0)).abs() < 1e-3);
    assert!((fit.kg_at(-0.5) + 50.0).abs() < 1e-3);
    // The unrestricted quadratic peaks at x = 5 and would read 0 at x = 10.
    assert!(fit.kg_at(10.0) > fit.kg_at(5.0));
}

#[test]
fn readings_increase_with_the_load_over_the_whole_adc_range() {
    for (model, truth) in [
        (Model::Linear, Truth::Linear),
        (Model::Quadratic, Truth::Quadratic),
        (Model::Piecewise, Truth::Piecewise),
    ] {
        for polarity in [1.0, -1.0] {
            let points = synthetic_points(truth, 1_000.0, polarity, 0.0, 0.0, 0);
            let calibration = Calibration::Fitted(fit(model, &points).unwrap());
            let mut previous = f32::NEG_INFINITY;
            // From well below the unloaded code to the end of the 24-bit range.
            for step in 0..=400 {
                let u = -20.0 + step as f32 * 2.0;
                let raw = raw(1_000.0, polarity, u).clamp(-8_388_608.0, 8_388_607.0) as i32;
                let reading = calibration.net_kg(raw, 1_000, 0.0);
                assert!(reading >= previous, "{model:?} {polarity} at {u}");
                previous = reading;
            }
        }
    }
}

#[test]
fn nonlinear_tare_subtracts_the_load_at_the_tare() {
    let points = synthetic_points(Truth::Piecewise, 10_000.0, -1.0, 0.0, 0.0, 0);
    let calibration = Calibration::Fitted(fit(Model::Piecewise, &points).unwrap());
    let raw_at = |u: f32| raw(10_000.0, -1.0, u) as i32;
    let load_at = |u: f32| Truth::Piecewise.load(f64::from(u)) as f32;

    // Tared unloaded, the readings are the loads.
    for u in HELD_OUT_READINGS {
        let reading = calibration.net_kg(raw_at(u), raw_at(0.0), 0.0);
        assert!((reading - load_at(u)).abs() < 0.01, "{u}");
    }
    // Tared under a 30 kg preload, the readings are the loads on top of it, and the sensitivity
    // still steps up at 48 kg of total load. Evaluating the curve at `raw - tare` would move the
    // step to 48 kg on top of the preload.
    let preload = 30.0;
    for u in [35.0, 45.0, 55.0, 70.0] {
        let reading = calibration.net_kg(raw_at(u), raw_at(preload), 0.0);
        assert!(
            (reading - (load_at(u) - load_at(preload))).abs() < 0.01,
            "{u}"
        );
    }
    let fit = calibration.fit().unwrap();
    let naive = fit.kg((raw_at(0.0) + raw_at(70.0) - raw_at(preload)) as f32);
    assert!((naive - (load_at(70.0) - load_at(preload))).abs() > 1.0);
}

#[test]
fn zero_update_follows_the_drift_of_the_unloaded_baseline() {
    let points = synthetic_points(Truth::Piecewise, 10_000.0, 1.0, 0.0, 0.0, 0);
    let calibration = Calibration::Fitted(fit(Model::Piecewise, &points).unwrap());
    // The baseline drifted by 0.2 kg of counts since the calibration.
    let drift = 0.2 * 1000.0 / DEFAULT_FACTOR;
    let raw_at = |u: f32| raw(10_000.0, 1.0, u) + drift;

    let zero_shift = calibration.zero_shift(raw_at(0.0)).unwrap();
    assert!((zero_shift - drift).abs() < 1.0, "{zero_shift}");
    let tare = raw_at(0.0) as i32;
    for u in [20.0, 47.0, 49.0, 80.0] {
        let reading = calibration.net_kg(raw_at(u) as i32, tare, zero_shift);
        let expected = Truth::Piecewise.load(f64::from(u)) as f32;
        assert!((reading - expected).abs() < 0.01, "{u}: {reading}");
    }
    // An attached load is not drift.
    assert_eq!(
        calibration.zero_shift(raw(10_000.0, 1.0, 5.0)),
        Err(Error::ZeroOutOfRange)
    );
    // Without a fit, there is no reference to drift from.
    assert_eq!(Calibration::DEFAULT.zero_shift(123_456.0), Ok(0.0));
}

#[test]
fn zero_update_without_a_measured_zero_finds_the_zero_load() {
    let points: Vec<Point> = synthetic_points(Truth::Quadratic, 10_000.0, 1.0, 0.0, 0.0, 0)
        .into_iter()
        .filter(|(_, load)| *load > 0.0)
        .collect();
    let calibration = Calibration::Fitted(fit(Model::Quadratic, &points).unwrap());
    let zero_shift = calibration.zero_shift(10_000.0).unwrap();
    // The lowest point is 5 kg, and below it the curve is straight with the slope there, so it
    // reads 0 slightly above the true zero: 0.0375 kg, or 344 counts.
    assert!((zero_shift + 344.0).abs() < 10.0, "{zero_shift}");
    assert!(calibration.fit().unwrap().kg(10_000.0 - zero_shift).abs() < 1e-3);
}

#[test]
fn requires_enough_points_and_load_levels() {
    let readings = |loads: &[f32]| -> Vec<Point> {
        loads
            .iter()
            .map(|&load| (raw(0.0, 1.0, load), load))
            .collect()
    };
    let cases: [(Model, Vec<Point>, Error); 7] = [
        (Model::Linear, readings(&[10.0]), Error::NotEnoughPoints),
        (
            Model::Quadratic,
            readings(&[0.0, 20.0, 40.0]),
            Error::NotEnoughPoints,
        ),
        // Four points, but 40 and 40.5 kg are one level of a 60 kg span.
        (
            Model::Quadratic,
            readings(&[0.0, 20.0, 40.0, 40.5]),
            Error::NotEnoughLoadLevels,
        ),
        (
            Model::Quadratic,
            readings(&[0.0, 5.0, 10.0, 90.0]),
            Error::NoMidRangeLevel,
        ),
        (
            Model::Piecewise,
            readings(&[0.0, 10.0, 20.0, 30.0, 40.0]),
            Error::NotEnoughPoints,
        ),
        (
            Model::Piecewise,
            readings(&[0.0, 0.0, 10.0, 10.0, 20.0, 20.0, 30.0]),
            Error::NotEnoughLoadLevels,
        ),
        // Repeated points are one level.
        (
            Model::Linear,
            vec![(5000.0, 10.0); 4],
            Error::NotEnoughLoadLevels,
        ),
    ];
    for (model, points, error) in cases {
        assert_eq!(fit(model, &points), Err(error), "{model:?} {points:?}");
        assert!(error.needs_more_data());
    }
}

#[test]
fn repeated_points_fit_like_the_points_once() {
    let points = session_points(0.0, 1.0);
    for model in Model::ALL {
        let once = fit(model, &points[..10]).unwrap();
        let twice: Vec<Point> = points[..10].iter().chain(&points[..10]).copied().collect();
        let twice = fit(model, &twice).unwrap();
        for (raw, _) in &points {
            assert!((once.kg(*raw) - twice.kg(*raw)).abs() < 1e-3, "{model:?}");
        }
    }
}

#[test]
fn piecewise_rejects_an_unidentifiable_breakpoint() {
    // Straight points fit every breakpoint.
    for polarity in [1.0, -1.0] {
        let points = synthetic_points(Truth::Linear, 0.0, polarity, 0.0, 0.0, 0);
        assert_eq!(fit(Model::Piecewise, &points), Err(Error::NoBreakpoint));
        let noisy = synthetic_points(Truth::Linear, 0.0, polarity, 10.0, 0.01, 3);
        assert_eq!(fit(Model::Piecewise, &noisy), Err(Error::NoBreakpoint));
    }

    // A step at 80 kg leaves one level above it, so the search ends at the third-highest level.
    let points: Vec<Point> = CALIBRATION_READINGS
        .iter()
        .map(|&u| {
            let load = if u < 80.0 { u } else { 80.0 + 1.2 * (u - 80.0) };
            (raw(0.0, 1.0, u), load)
        })
        .collect();
    assert_eq!(
        fit(Model::Piecewise, &points),
        Err(Error::InsufficientSegmentSupport)
    );
}

#[test]
fn rejects_curves_that_decrease_or_bend_implausibly() {
    let readings = |pairs: &[(f32, f32)]| -> Vec<Point> {
        pairs
            .iter()
            .map(|&(u, load)| (raw(0.0, 1.0, u), load))
            .collect()
    };
    // The load falls past 40 kg.
    let turning_back = readings(&[
        (0.0, 0.0),
        (10.0, 10.0),
        (20.0, 18.0),
        (30.0, 22.0),
        (40.0, 22.0),
        (50.0, 18.0),
    ]);
    assert_eq!(
        fit(Model::Quadratic, &turning_back),
        Err(Error::NotMonotonic)
    );
    // Twice as sensitive past 30 kg.
    let bent = readings(&[
        (0.0, 0.0),
        (10.0, 10.0),
        (20.0, 20.0),
        (30.0, 30.0),
        (40.0, 50.0),
        (50.0, 70.0),
        (60.0, 90.0),
    ]);
    assert_eq!(
        fit(Model::Piecewise, &bent),
        Err(Error::ExcessiveNonlinearity)
    );
    // 20 times the default sensitivity.
    let stiff = readings(&[(0.0, 0.0), (1.0, 20.0), (2.0, 40.0), (3.0, 60.0)]);
    assert_eq!(
        fit(Model::Quadratic, &stiff),
        Err(Error::ImplausibleSensitivity)
    );

    let mut fit = concave_quadratic();
    fit.curve = Curve::Quadratic {
        a: -100.0,
        b: 100.0,
        c: 0.0,
    };
    assert_eq!(fit.validate(), Err(Error::NotMonotonic));
    fit.curve = Curve::Quadratic {
        a: 10.0,
        b: 100.0,
        c: 0.0,
    };
    fit.validate().unwrap();
    fit.curve = Curve::Quadratic {
        a: 30.0,
        b: 100.0,
        c: 0.0,
    };
    assert_eq!(fit.validate(), Err(Error::ExcessiveNonlinearity));
}

#[test]
fn rejects_non_finite_and_degenerate_points() {
    let mut points = session_points(0.0, 1.0);
    points[3].0 = f32::NAN;
    for model in Model::ALL {
        assert_eq!(fit(model, &points), Err(Error::NonFinite));
    }
    points[3].0 = 1000.0;
    points[4].1 = f32::INFINITY;
    assert_eq!(fit(Model::Linear, &points), Err(Error::NonFinite));

    // Different loads at one raw value.
    let same_raw: Vec<Point> = (0..8).map(|i| (5000.0, i as f32 * 10.0)).collect();
    for model in Model::ALL {
        assert_eq!(fit(model, &same_raw), Err(Error::Degenerate), "{model:?}");
    }

    let mut fit = concave_quadratic();
    fit.scale = 0.0;
    assert_eq!(fit.validate(), Err(Error::Degenerate));
    fit.scale = f32::NAN;
    assert_eq!(fit.validate(), Err(Error::NonFinite));
}

#[test]
fn reports_no_linear_factor_for_nonlinear_models() {
    let points = session_points(0.0, 1.0);
    assert_eq!(Calibration::DEFAULT.legacy_factor(), DEFAULT_FACTOR);
    let linear = Calibration::Fitted(fit(Model::Linear, &points).unwrap());
    assert!(linear.legacy_factor().is_finite());
    for model in [Model::Quadratic, Model::Piecewise] {
        let calibration = Calibration::Fitted(fit(model, &points).unwrap());
        assert!(calibration.legacy_factor().is_nan());
    }
}

/// The app draws the curves with its own evaluation; its test
/// `CalibrationCurve evaluates like the firmware` checks the same values.
#[test]
fn evaluates_the_curves_the_app_previews() {
    let fit = |model, coefficients, reference_raw, scale| {
        Fit::from_coefficients(model, coefficients, reference_raw, scale, (0.0, 1.0))
    };
    let linear = fit(Model::Linear, [0.1, 0.5, 0.0, 0.0], 1000.0, 50_000.0);
    assert!((linear.kg(21_000.0) - 2.5).abs() < 1e-5);
    assert!((linear.kg(-9_000.0) + 0.5).abs() < 1e-5);

    let quadratic = fit(Model::Quadratic, [-10.0, 100.0, 0.0, 0.0], 0.0, 1e6);
    assert!((quadratic.kg(500_000.0) - 47.5).abs() < 1e-4);
    assert!((quadratic.kg(3e6) - 250.0).abs() < 1e-4);
    assert!((quadratic.kg(-5e5) + 50.0).abs() < 1e-4);

    let piecewise = fit(Model::Piecewise, [0.2, 90.0, 100.0, 0.5], -2000.0, -1e6);
    piecewise.validate().unwrap();
    for (raw, kg) in [
        (-252_000.0, 22.7),
        (-752_000.0, 70.2),
        (-2_002_000.0, 195.2),
        (98_000.0, -8.8),
    ] {
        assert!((piecewise.kg(raw) - kg).abs() < 1e-4, "{raw}");
    }
}
