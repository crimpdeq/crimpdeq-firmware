//! Editing the calibration points and selecting the model.

mod common;

use common::{raw, session_points};
use crimpdeq_calibration::{
    Calibration,
    CalibrationRecord,
    DEFAULT_FACTOR,
    Error,
    MAX_POINTS,
    Model,
    Outcome,
};

fn record_with(points: &[(f32, f32)]) -> CalibrationRecord {
    let mut record = CalibrationRecord::DEFAULT;
    for point in points {
        record.add_point(*point);
    }
    record
}

#[test]
fn builds_a_linear_calibration_point_by_point() {
    let mut record = CalibrationRecord::DEFAULT;
    assert_eq!(record.active, Calibration::Factor(DEFAULT_FACTOR));

    assert_eq!(
        record.add_point((1000.0, 0.0)),
        Outcome::Pending(Error::NotEnoughPoints)
    );
    assert_eq!(record.points.len(), 1);
    assert_eq!(record.active, Calibration::DEFAULT);

    assert_eq!(
        record.add_point((raw(1000.0, 1.0, 20.0), 20.0)),
        Outcome::Applied
    );
    assert_eq!(record.pending, None);
    assert_eq!(record.model(), Model::Linear);
    assert!((record.active.legacy_factor() - DEFAULT_FACTOR).abs() < 1e-6);
}

#[test]
fn rejects_a_point_that_invalidates_an_applied_calibration() {
    let mut record = record_with(&[(0.0, 0.0), (raw(0.0, 1.0, 20.0), 20.0)]);
    let before = record.clone();

    // The 40 kg weight was not hung, so the reading did not change.
    assert_eq!(
        record.add_point((raw(0.0, 1.0, 0.5), 40.0)),
        Outcome::Applied,
        "still plausible"
    );
    let mut record = before.clone();
    assert_eq!(
        record.add_point((raw(0.0, 1.0, 20.0) + 1.0, 400.0)),
        Outcome::Rejected(Error::ImplausibleSensitivity)
    );
    assert_eq!(record, before);
}

#[test]
fn keeps_points_that_are_already_pending() {
    // A wrong first point makes every fit implausible, so it must stay removable.
    let mut record = record_with(&[(0.0, 0.0)]);
    assert_eq!(
        record.add_point((1.0, 50.0)),
        Outcome::Pending(Error::ImplausibleSensitivity)
    );
    assert_eq!(record.points.len(), 2);
    assert_eq!(record.active, Calibration::DEFAULT);

    assert_eq!(
        record.remove_point(1),
        Outcome::Pending(Error::NotEnoughPoints)
    );
    assert_eq!(
        record.add_point((raw(0.0, 1.0, 50.0), 50.0)),
        Outcome::Applied
    );
}

#[test]
fn selects_a_model_only_when_it_fits() {
    let points = session_points(0.0, 1.0);
    let mut record = record_with(&points[..3]);
    assert_eq!(record.model(), Model::Linear);
    let linear = record.clone();

    // Three points have three load levels; the quadratic model needs four.
    assert_eq!(
        record.select_model(Model::Quadratic),
        Outcome::Rejected(Error::NotEnoughPoints)
    );
    assert_eq!(record, linear);

    for point in &points[3..] {
        assert_eq!(record.add_point(*point), Outcome::Applied);
    }
    assert_eq!(record.select_model(Model::Piecewise), Outcome::Applied);
    assert_eq!(record.model(), Model::Piecewise);
    assert_eq!(record.select_model(Model::Quadratic), Outcome::Applied);
    assert_eq!(record.model(), Model::Quadratic);
    assert_eq!(record.select_model(Model::Linear), Outcome::Applied);
    assert_eq!(record.model(), Model::Linear);
}

#[test]
fn removing_points_refits_or_keeps_the_calibration_in_use() {
    let points = session_points(0.0, 1.0);
    let mut record = record_with(&points);
    assert_eq!(record.select_model(Model::Piecewise), Outcome::Applied);

    // Each removal refits the piecewise model while enough levels remain.
    let applied = record.active;
    assert_eq!(record.remove_point(15), Outcome::Applied);
    assert_ne!(record.active, applied);

    // With five points, the piecewise model cannot be fit: the removal stays, and so does the
    // previous piecewise calibration.
    while record.points.len() > 6 {
        record.remove_point(0);
    }
    let in_use = record.active;
    assert_eq!(
        record.remove_point(0),
        Outcome::Pending(Error::NotEnoughPoints)
    );
    assert_eq!(record.points.len(), 5);
    assert_eq!(record.active, in_use);
    assert_eq!(record.model(), Model::Piecewise);
    assert_eq!(record.pending, Some(Error::NotEnoughPoints));

    // Selecting a model that fits the remaining points applies it.
    assert_eq!(record.select_model(Model::Linear), Outcome::Applied);
    assert_eq!(record.pending, None);

    assert_eq!(
        record.remove_point(9),
        Outcome::Rejected(Error::InvalidIndex)
    );
}

#[test]
fn a_pending_edit_applies_once_the_points_fit() {
    let points = session_points(0.0, 1.0);
    let mut record = record_with(&points[..6]);
    assert_eq!(record.select_model(Model::Quadratic), Outcome::Applied);

    // Remove all but the zero point, refitting while enough points remain, then add points back.
    while record.points.len() > 1 {
        assert!(matches!(
            record.remove_point(1),
            Outcome::Pending(_) | Outcome::Applied
        ));
    }
    assert_eq!(record.pending, Some(Error::NotEnoughPoints));
    assert_eq!(record.model(), Model::Quadratic);
    let quadratic = record.active;
    // 5.025, 4 and 25 kg.
    for index in [1, 5] {
        assert_eq!(
            record.add_point(points[index]),
            Outcome::Pending(Error::NotEnoughPoints)
        );
    }
    assert_eq!(
        record.add_point(points[4]),
        Outcome::Pending(Error::NoMidRangeLevel)
    );
    // 10 kg of a 25 kg span is in the middle.
    assert_eq!(record.add_point(points[2]), Outcome::Applied);
    assert_ne!(record.active, quadratic);
    assert_eq!(record.model(), Model::Quadratic);
}

#[test]
fn stops_at_the_point_limit() {
    let mut record = CalibrationRecord::DEFAULT;
    for i in 0..MAX_POINTS {
        record.add_point((raw(0.0, 1.0, i as f32 * 4.0), i as f32 * 4.0));
    }
    assert_eq!(
        record.add_point((raw(0.0, 1.0, 100.0), 100.0)),
        Outcome::Rejected(Error::PointLimitReached)
    );
    assert_eq!(
        record.add_point((f32::NAN, 1.0)),
        Outcome::Rejected(Error::PointLimitReached)
    );
    record.remove_point(0);
    assert_eq!(
        record.add_point((f32::NAN, 1.0)),
        Outcome::Rejected(Error::NonFinite)
    );
}

#[test]
fn reset_restores_the_default() {
    let mut record = record_with(&session_points(0.0, 1.0));
    record.select_model(Model::Quadratic);
    record.reset();
    assert_eq!(record, CalibrationRecord::DEFAULT);
}

#[test]
fn reports_the_calibration_fit_residuals() {
    let mut record = record_with(&[
        (0.0, 0.0),
        (raw(0.0, 1.0, 10.0), 10.3),
        (raw(0.0, 1.0, 20.0), 20.0),
    ]);
    // Without a fit there is nothing to compare.
    assert_eq!(CalibrationRecord::DEFAULT.diagnostics(), None);

    let diagnostics = record.diagnostics().unwrap();
    let fitted: Vec<f32> = record.fitted_kg().map(Option::unwrap).collect();
    let residuals: Vec<f32> = fitted
        .iter()
        .zip(&record.points)
        .map(|(fitted, (_, load))| fitted - load)
        .collect();
    let max_abs = residuals.iter().fold(0.0_f32, |max, r| max.max(r.abs()));
    let rms = (residuals.iter().map(|r| r * r).sum::<f32>() / 3.0).sqrt();
    assert!((diagnostics.max_abs_kg - max_abs).abs() < 1e-5);
    assert!((diagnostics.rms_kg - rms).abs() < 1e-5);
    assert!(fitted[0].abs() < 1e-6, "anchored at the zero point");

    record.reset();
    assert_eq!(record.diagnostics(), None);
}
