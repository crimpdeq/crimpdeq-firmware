//! BLE payloads of the calibration details.

mod common;

use common::session_points;
use crimpdeq_calibration::{
    CalibrationRecord,
    DEFAULT_FACTOR,
    Error,
    Model,
    Outcome,
    wire::{Detail, LastCommand, MAX_PAYLOAD_SIZE, details},
};

fn f32_at(payload: &[u8], offset: usize) -> f32 {
    f32::from_le_bytes(payload[offset..offset + 4].try_into().unwrap())
}

fn piecewise_record() -> CalibrationRecord {
    let mut record = CalibrationRecord::DEFAULT;
    for point in session_points(0.0, 1.0) {
        record.add_point(point);
    }
    assert_eq!(record.select_model(Model::Piecewise), Outcome::Applied);
    record
}

#[test]
fn encodes_the_info() {
    let detail = Detail::Info {
        model: Model::Piecewise,
        point_count: 16,
        has_fit: true,
        pending: Some(Error::NotEnoughPoints),
        last: Some(LastCommand {
            op_code: 0x7A,
            outcome: Outcome::Rejected(Error::NoMidRangeLevel),
        }),
    };
    assert_eq!(
        detail.payload().as_slice(),
        [0, 1, 0b111, 2, 16, 1, 1, 0x7A, 2, 3]
    );
    let applied = Detail::Info {
        model: Model::Linear,
        point_count: 0,
        has_fit: false,
        pending: None,
        last: None,
    };
    assert_eq!(
        applied.payload().as_slice(),
        [0, 1, 0b111, 0, 0, 0, 0, 0, 0, 0]
    );
}

#[test]
fn describes_a_fitted_calibration() {
    let record = piecewise_record();
    let fit = *record.active.fit().unwrap();
    let tare_raw = 5_000;
    let last = Some(LastCommand {
        op_code: 0x7A,
        outcome: Outcome::Applied,
    });
    let details = details(&record, tare_raw, 120.0, last);
    assert_eq!(details.len(), 6 + record.points.len());

    let payloads: Vec<_> = details.iter().map(Detail::payload).collect();
    let kinds: Vec<u8> = payloads.iter().map(|payload| payload[0]).collect();
    assert_eq!(&kinds[..5], [0, 1, 2, 3, 4]);
    assert!(kinds[5..21].iter().all(|kind| *kind == 5));
    assert_eq!(kinds[21], 6);
    assert!(
        payloads
            .iter()
            .all(|payload| payload.len() <= MAX_PAYLOAD_SIZE)
    );

    let curve = &payloads[1];
    assert_eq!(curve.len(), 18);
    assert_eq!(curve[1], Model::Piecewise as u8);
    for (i, coefficient) in fit.coefficients().into_iter().enumerate() {
        assert_eq!(f32_at(curve, 2 + 4 * i), coefficient);
    }
    let coordinates = &payloads[2];
    assert_eq!(f32_at(coordinates, 1), fit.reference_raw);
    assert_eq!(f32_at(coordinates, 5), fit.scale);
    assert_eq!(f32_at(coordinates, 9), fit.x_min);
    assert_eq!(f32_at(coordinates, 13), fit.x_max);

    let diagnostics = record.diagnostics().unwrap();
    let fit_detail = &payloads[3];
    assert_eq!(f32_at(fit_detail, 1), diagnostics.rms_kg);
    assert_eq!(f32_at(fit_detail, 5), diagnostics.max_abs_kg);
    assert_eq!(
        (f32_at(fit_detail, 9), f32_at(fit_detail, 13)),
        fit.load_range_kg()
    );

    let tare = &payloads[4];
    assert_eq!(f32_at(tare, 1), fit.kg(tare_raw as f32 - 120.0));
    assert_eq!(f32_at(tare, 5), 120.0);

    let point = &payloads[5 + 14];
    assert_eq!(point[1], 14);
    let (raw, load) = record.points[14];
    assert_eq!((f32_at(point, 2), f32_at(point, 6)), (raw, load));
    assert_eq!(f32_at(point, 10), fit.kg(raw));
    assert_eq!(payloads[21].as_slice(), [6, 16]);
}

#[test]
fn marks_unknown_values_of_a_factor_calibration() {
    let mut record = CalibrationRecord::DEFAULT;
    record.add_point((1000.0, 0.0));
    let payloads: Vec<_> = details(&record, 0, 0.0, None)
        .iter()
        .map(Detail::payload)
        .collect();

    // Pending: one point is not enough.
    assert_eq!(payloads[0].as_slice(), [0, 1, 0b111, 0, 1, 0, 1, 0, 0, 0]);
    assert_eq!(f32_at(&payloads[1], 2), DEFAULT_FACTOR);
    assert!(
        (1..17)
            .step_by(4)
            .all(|offset| f32_at(&payloads[2], offset).is_nan())
    );
    assert!(
        (1..17)
            .step_by(4)
            .all(|offset| f32_at(&payloads[3], offset).is_nan())
    );
    assert!(f32_at(&payloads[4], 1).is_nan());
    assert!(f32_at(&payloads[5], 10).is_nan(), "no fitted load");
}
