//! BLE payloads that describe the calibration, sent with response code [`RESPONSE_CODE`]. The
//! first payload byte is the kind of detail. Unknown values are NaN, and every payload fits a
//! notification at the minimum ATT MTU of 23 bytes.

use arrayvec::ArrayVec;

use crate::{CalibrationRecord, Error, MAX_POINTS, Model, Outcome};

/// Response code of every calibration detail.
pub const RESPONSE_CODE: u8 = 0x0A;
/// Version of the calibration details, for clients to check the layout.
pub const PROTOCOL_VERSION: u8 = 1;
/// Largest payload: the 20-byte notification of the minimum ATT MTU without the response code and
/// length.
pub const MAX_PAYLOAD_SIZE: usize = 18;
pub type Payload = ArrayVec<u8, MAX_PAYLOAD_SIZE>;

/// Calibration command that the details report the outcome of.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LastCommand {
    pub op_code: u8,
    pub outcome: Outcome,
}

/// One notification of the calibration details.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Detail {
    /// Kind 0: `[version, supported model mask, model, point count, flags, pending error, last
    /// op code, last disposition, last error]`, one byte each. Flags bit 0: the calibration has
    /// a fit, so the coordinates and range are known. Dispositions: 0 applied, 1 stored but
    /// pending, 2 rejected. Codes are 0 for none.
    Info {
        model: Model,
        point_count: u8,
        has_fit: bool,
        pending: Option<Error>,
        last: Option<LastCommand>,
    },
    /// Kind 1: model code, then four `f32` coefficients in the order of
    /// [`crate::Fit::coefficients`]. A calibration without a fit sends its factor first.
    Curve {
        model: Model,
        coefficients: [f32; 4],
    },
    /// Kind 2: reference raw, scale, x min and x max, `f32` each.
    Coordinates {
        reference_raw: f32,
        scale: f32,
        x_min: f32,
        x_max: f32,
    },
    /// Kind 3: RMS and largest absolute calibration-fit residual of the points, and the loads at
    /// the ends of the calibrated range, in kg, `f32` each.
    Fit {
        rms_kg: f32,
        max_abs_kg: f32,
        load_min_kg: f32,
        load_max_kg: f32,
    },
    /// Kind 4: absolute load at the tare in kg, and the zero shift in raw counts, `f32` each.
    /// A reading plus the tare load is the absolute load to compare with the calibrated range.
    Tare { tare_load_kg: f32, zero_shift: f32 },
    /// Kind 5: index (`u8`), raw value, known load and fitted load in kg, `f32` each.
    Point {
        index: u8,
        raw: f32,
        weight_kg: f32,
        fitted_kg: f32,
    },
    /// Kind 6: point count (`u8`). The last detail.
    End { point_count: u8 },
}

/// Bit of each supported model in the info mask.
const SUPPORTED_MODELS: u8 =
    (1 << Model::Linear as u8) | (1 << Model::Quadratic as u8) | (1 << Model::Piecewise as u8);

impl Detail {
    pub fn payload(&self) -> Payload {
        let mut payload = Payload::new();
        let mut put = |kind: u8, bytes: &[u8], values: &[f32]| {
            payload.push(kind);
            payload.extend(bytes.iter().copied());
            for value in values {
                payload.extend(value.to_le_bytes());
            }
        };
        match *self {
            Self::Info {
                model,
                point_count,
                has_fit,
                pending,
                last,
            } => {
                let (op_code, disposition, error) = match last {
                    None => (0, 0, 0),
                    Some(LastCommand { op_code, outcome }) => match outcome {
                        Outcome::Applied => (op_code, 0, 0),
                        Outcome::Pending(error) => (op_code, 1, error as u8),
                        Outcome::Rejected(error) => (op_code, 2, error as u8),
                    },
                };
                put(
                    0,
                    &[
                        PROTOCOL_VERSION,
                        SUPPORTED_MODELS,
                        model as u8,
                        point_count,
                        u8::from(has_fit),
                        pending.map_or(0, |error| error as u8),
                        op_code,
                        disposition,
                        error,
                    ],
                    &[],
                );
            }
            Self::Curve {
                model,
                coefficients,
            } => put(1, &[model as u8], &coefficients),
            Self::Coordinates {
                reference_raw,
                scale,
                x_min,
                x_max,
            } => put(2, &[], &[reference_raw, scale, x_min, x_max]),
            Self::Fit {
                rms_kg,
                max_abs_kg,
                load_min_kg,
                load_max_kg,
            } => put(3, &[], &[rms_kg, max_abs_kg, load_min_kg, load_max_kg]),
            Self::Tare {
                tare_load_kg,
                zero_shift,
            } => put(4, &[], &[tare_load_kg, zero_shift]),
            Self::Point {
                index,
                raw,
                weight_kg,
                fitted_kg,
            } => put(5, &[index], &[raw, weight_kg, fitted_kg]),
            Self::End { point_count } => put(6, &[point_count], &[]),
        }
        payload
    }
}

/// Details that describe the calibration, in the order they are sent.
pub fn details(
    record: &CalibrationRecord,
    tare_raw: i32,
    zero_shift: f32,
    last: Option<LastCommand>,
) -> ArrayVec<Detail, { MAX_POINTS + 6 }> {
    let fit = record.active.fit();
    let point_count = record.points.len() as u8;
    let diagnostics = record.diagnostics();
    let (load_min_kg, load_max_kg) = fit.map_or((f32::NAN, f32::NAN), |fit| fit.load_range_kg());

    let mut details = ArrayVec::new();
    details.push(Detail::Info {
        model: record.model(),
        point_count,
        has_fit: fit.is_some(),
        pending: record.pending,
        last,
    });
    details.push(Detail::Curve {
        model: record.model(),
        coefficients: fit.map_or([record.active.legacy_factor(), 0.0, 0.0, 0.0], |fit| {
            fit.coefficients()
        }),
    });
    details.push(Detail::Coordinates {
        reference_raw: fit.map_or(f32::NAN, |fit| fit.reference_raw),
        scale: fit.map_or(f32::NAN, |fit| fit.scale),
        x_min: fit.map_or(f32::NAN, |fit| fit.x_min),
        x_max: fit.map_or(f32::NAN, |fit| fit.x_max),
    });
    details.push(Detail::Fit {
        rms_kg: diagnostics.map_or(f32::NAN, |diagnostics| diagnostics.rms_kg),
        max_abs_kg: diagnostics.map_or(f32::NAN, |diagnostics| diagnostics.max_abs_kg),
        load_min_kg,
        load_max_kg,
    });
    details.push(Detail::Tare {
        tare_load_kg: fit.map_or(f32::NAN, |fit| fit.kg(tare_raw as f32 - zero_shift)),
        zero_shift,
    });
    for (index, (&(raw, weight_kg), fitted_kg)) in
        record.points.iter().zip(record.fitted_kg()).enumerate()
    {
        details.push(Detail::Point {
            index: index as u8,
            raw,
            weight_kg,
            fitted_kg: fitted_kg.unwrap_or(f32::NAN),
        });
    }
    details.push(Detail::End { point_count });
    details
}
