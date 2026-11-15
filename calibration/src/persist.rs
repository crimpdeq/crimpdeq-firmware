//! Flash record of the calibration, in the 4 KiB `calib` partition.
//!
//! The sector holds a log of [`SLOTS`] versioned records. A write goes to the erased slot after
//! the last written one, without erasing, so a power cut during the write leaves the previous
//! record intact and the reader picks the newest record whose CRC matches. Only when the log is
//! full does the write erase the sector first; a power cut between that erase and the write
//! loses the calibration, and the device starts with the default one.
//!
//! Slot layout, little-endian:
//!
//! | Offset | Size | Field                                                      |
//! | ------ | ---- | ---------------------------------------------------------- |
//! | 0      | 4    | magic `0x43445146`                                         |
//! | 4      | 4    | version, 3                                                 |
//! | 8      | 4    | sequence, increasing with each write                       |
//! | 12     | 4    | payload length                                             |
//! | 16     | n    | payload                                                    |
//! | 16 + n | 4    | CRC-32 (IEEE) of the bytes before it                       |
//!
//! Payload:
//!
//! | Offset | Size | Field                                                      |
//! | ------ | ---- | ---------------------------------------------------------- |
//! | 0      | 1    | 0 for a factor without a fit, 1 for a fit                  |
//! | 1      | 1    | model code                                                 |
//! | 2      | 1    | pending error code, 0 when the points are applied          |
//! | 3      | 1    | point count                                                |
//! | 4      | 16   | coefficients, `f32` each, in the order of [`Fit::coefficients`]; the factor first without a fit |
//! | 20     | 16   | reference raw, scale, x min, x max, `f32` each             |
//! | 36     | 8 n  | points: raw and kg, `f32` each                             |
//!
//! Earlier firmware stored only a linear factor, in a 16-byte version 2 record at the start of
//! the partition: magic, version, factor bits and an XOR checksum. It reads as that factor
//! without points, which converts readings as before.

use arrayvec::ArrayVec;

use crate::{Calibration, CalibrationRecord, Error, Fit, MAX_POINTS, Model, is_valid_factor};

pub const SECTOR_SIZE: usize = 4096;
pub const SLOT_SIZE: usize = 256;
pub const SLOTS: usize = SECTOR_SIZE / SLOT_SIZE;
const MAGIC: u32 = 0x4344_5146;
const VERSION: u32 = 3;
const LEGACY_VERSION: u32 = 2;
const HEADER_SIZE: usize = 16;
const FIXED_PAYLOAD_SIZE: usize = 36;
const POINT_SIZE: usize = 8;
const CRC_SIZE: usize = 4;
const _: () = assert!(
    HEADER_SIZE + FIXED_PAYLOAD_SIZE + MAX_POINTS * POINT_SIZE + CRC_SIZE <= SLOT_SIZE,
    "a record must fit its slot"
);

/// Word-aligned sector image, which the flash driver reads and writes without copying.
#[repr(C, align(4))]
pub struct Sector(pub [u8; SECTOR_SIZE]);

/// Word-aligned slot image.
#[repr(C, align(4))]
pub struct Slot(pub [u8; SLOT_SIZE]);

/// Where the loaded calibration came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Source {
    Record {
        slot: usize,
        sequence: u32,
    },
    /// Factor of earlier firmware.
    Legacy,
    /// Erased sector.
    Empty,
    /// No valid record in a written sector: corrupt, interrupted, or of an unsupported version.
    Invalid,
}

/// Reads the newest valid record, or the default calibration.
pub fn load(sector: &Sector) -> (CalibrationRecord, Source) {
    let newest = slots(sector)
        .enumerate()
        .filter_map(|(slot, bytes)| {
            decode(bytes).map(|(sequence, record)| (slot, sequence, record))
        })
        .max_by_key(|(_, sequence, _)| *sequence);
    if let Some((slot, sequence, record)) = newest {
        return (record, Source::Record { slot, sequence });
    }
    if let Some(factor) = decode_legacy(&sector.0) {
        let record = CalibrationRecord {
            active: Calibration::Factor(factor),
            ..CalibrationRecord::DEFAULT
        };
        return (record, Source::Legacy);
    }
    let source = if sector.0.iter().all(|byte| *byte == 0xFF) {
        Source::Empty
    } else {
        Source::Invalid
    };
    (CalibrationRecord::DEFAULT, source)
}

/// Flash operations that store a record.
pub struct Write {
    /// Erase the sector before writing.
    pub erase: bool,
    /// Offset of the slot in the sector.
    pub offset: usize,
    pub slot: Slot,
}

/// Plans the write of a record after the last written slot, or at the start of an erased sector
/// when the log is full. Slots holding a corrupt or interrupted write are skipped, as NOR flash
/// cannot rewrite them without an erase.
pub fn plan_write(sector: &Sector, record: &CalibrationRecord) -> Write {
    let sequence = slots(sector)
        .filter_map(|bytes| decode(bytes).map(|(sequence, _)| sequence))
        .max()
        .map_or(1, |sequence| sequence.wrapping_add(1));
    let next_slot = slots(sector)
        .rposition(|bytes| bytes.iter().any(|byte| *byte != 0xFF))
        .map_or(0, |last| last + 1);
    let (erase, slot) = if next_slot < SLOTS {
        (false, next_slot)
    } else {
        (true, 0)
    };
    Write {
        erase,
        offset: slot * SLOT_SIZE,
        slot: encode(record, sequence),
    }
}

fn slots(sector: &Sector) -> impl DoubleEndedIterator<Item = &[u8; SLOT_SIZE]> + ExactSizeIterator {
    sector.0.as_chunks::<SLOT_SIZE>().0.iter()
}

/// Encodes a record, padded with erased bytes.
pub fn encode(record: &CalibrationRecord, sequence: u32) -> Slot {
    let mut payload = ArrayVec::<u8, { FIXED_PAYLOAD_SIZE + MAX_POINTS * POINT_SIZE }>::new();
    let (kind, coefficients, coordinates) = match record.active {
        Calibration::Factor(factor) => (0, [factor, 0.0, 0.0, 0.0], [0.0; 4]),
        Calibration::Fitted(fit) => (
            1,
            fit.coefficients(),
            [fit.reference_raw, fit.scale, fit.x_min, fit.x_max],
        ),
    };
    payload.extend([
        kind,
        record.model() as u8,
        record.pending.map_or(0, |error| error as u8),
        record.points.len() as u8,
    ]);
    for value in coefficients.into_iter().chain(coordinates) {
        payload.extend(value.to_le_bytes());
    }
    for (raw, weight) in &record.points {
        payload.extend(raw.to_le_bytes());
        payload.extend(weight.to_le_bytes());
    }

    let mut slot = Slot([0xFF; SLOT_SIZE]);
    let end = HEADER_SIZE + payload.len();
    for (i, word) in [MAGIC, VERSION, sequence, payload.len() as u32]
        .into_iter()
        .enumerate()
    {
        slot.0[4 * i..4 * i + 4].copy_from_slice(&word.to_le_bytes());
    }
    slot.0[HEADER_SIZE..end].copy_from_slice(&payload);
    let crc = crc32(&slot.0[..end]);
    slot.0[end..end + CRC_SIZE].copy_from_slice(&crc.to_le_bytes());
    slot
}

/// Decodes a slot into its sequence and record, or `None` unless it is a valid record of this
/// version. The fit is validated again, so a record that passes the CRC but was written by buggy
/// firmware cannot apply an invalid curve.
pub fn decode(slot: &[u8]) -> Option<(u32, CalibrationRecord)> {
    let word = |offset: usize| {
        Some(u32::from_le_bytes(
            slot.get(offset..offset + 4)?.try_into().ok()?,
        ))
    };
    if word(0)? != MAGIC || word(4)? != VERSION {
        return None;
    }
    let sequence = word(8)?;
    let length = usize::try_from(word(12)?).ok()?;
    let end = HEADER_SIZE.checked_add(length)?;
    if length < FIXED_PAYLOAD_SIZE
        || end + CRC_SIZE > slot.len()
        || word(end)? != crc32(&slot[..end])
    {
        return None;
    }
    let payload = &slot[HEADER_SIZE..end];
    let float = |offset: usize| {
        let value = f32::from_le_bytes(payload[offset..offset + 4].try_into().ok()?);
        value.is_finite().then_some(value)
    };

    let [kind, model, pending, count] = payload[..4] else {
        return None;
    };
    let model = Model::from_code(model)?;
    let pending = match pending {
        0 => None,
        code => Some(Error::from_code(code)?),
    };
    let count = usize::from(count);
    if count > MAX_POINTS || length != FIXED_PAYLOAD_SIZE + count * POINT_SIZE {
        return None;
    }
    let coefficients = [float(4)?, float(8)?, float(12)?, float(16)?];
    let active = match kind {
        0 if model == Model::Linear && is_valid_factor(coefficients[0]) => {
            Calibration::Factor(coefficients[0])
        }
        1 => {
            let fit = Fit::from_coefficients(
                model,
                coefficients,
                float(20)?,
                float(24)?,
                (float(28)?, float(32)?),
            );
            fit.validate().ok()?;
            Calibration::Fitted(fit)
        }
        _ => return None,
    };
    let points = (0..count)
        .map(|i| {
            let offset = FIXED_PAYLOAD_SIZE + i * POINT_SIZE;
            Some((float(offset)?, float(offset + 4)?))
        })
        .collect::<Option<_>>()?;
    Some((
        sequence,
        CalibrationRecord {
            active,
            points,
            pending,
        },
    ))
}

/// Factor of the version 2 record of earlier firmware.
fn decode_legacy(sector: &[u8]) -> Option<f32> {
    let word = |i: usize| u32::from_le_bytes(sector[4 * i..4 * i + 4].try_into().unwrap());
    let (magic, version, value, checksum) = (word(0), word(1), word(2), word(3));
    let valid = magic == MAGIC
        && version == LEGACY_VERSION
        && checksum == magic ^ version ^ value ^ 0xA5A5_5A5A;
    let factor = f32::from_bits(value);
    (valid && is_valid_factor(factor)).then_some(factor)
}

/// CRC-32 (IEEE 802.3), bitwise, as records are small and rarely written.
fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = !0u32;
    for byte in bytes {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            crc = if crc & 1 == 1 {
                (crc >> 1) ^ 0xEDB8_8320
            } else {
                crc >> 1
            };
        }
    }
    !crc
}
