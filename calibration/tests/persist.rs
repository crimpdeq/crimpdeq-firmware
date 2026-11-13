//! Flash record: round trips, the log of slots, migration and damaged records.

mod common;

use common::session_points;
use crimpdeq_calibration::{
    Calibration,
    CalibrationRecord,
    Curve,
    DEFAULT_FACTOR,
    Fit,
    Model,
    persist::{
        SECTOR_SIZE,
        SLOT_SIZE,
        SLOTS,
        Sector,
        Source,
        Write,
        decode,
        encode,
        load,
        plan_write,
    },
};

fn erased() -> Sector {
    Sector([0xFF; SECTOR_SIZE])
}

/// Applies a write with NOR flash semantics: an erase sets every bit, a write only clears bits.
fn apply(sector: &mut Sector, write: &Write, bytes: usize) {
    if write.erase {
        sector.0.fill(0xFF);
    }
    for (target, source) in sector.0[write.offset..write.offset + bytes]
        .iter_mut()
        .zip(&write.slot.0)
    {
        *target &= source;
    }
}

fn store(sector: &mut Sector, record: &CalibrationRecord) -> Write {
    let write = plan_write(sector, record);
    apply(sector, &write, SLOT_SIZE);
    write
}

/// Version 2 record of firmware 3.0: magic, version, factor bits and an XOR checksum.
fn legacy_sector(version: u32, factor: f32) -> Sector {
    let magic = 0x4344_5146u32;
    let value = factor.to_bits();
    let mut sector = erased();
    for (i, word) in [magic, version, value, magic ^ version ^ value ^ 0xA5A5_5A5A]
        .into_iter()
        .enumerate()
    {
        sector.0[4 * i..4 * i + 4].copy_from_slice(&word.to_le_bytes());
    }
    sector
}

fn records() -> Vec<CalibrationRecord> {
    let points = session_points(-3_000.0, -1.0);
    let mut fitted = CalibrationRecord::DEFAULT;
    for point in &points {
        fitted.add_point(*point);
    }
    let mut piecewise = fitted.clone();
    assert_eq!(
        piecewise.select_model(Model::Piecewise),
        crimpdeq_calibration::Outcome::Applied
    );
    let mut quadratic = fitted.clone();
    quadratic.select_model(Model::Quadratic);
    let mut pending = piecewise.clone();
    while pending.pending.is_none() {
        pending.remove_point(0);
    }
    vec![
        CalibrationRecord::DEFAULT,
        CalibrationRecord {
            active: Calibration::Factor(0.0812),
            ..CalibrationRecord::DEFAULT
        },
        fitted,
        quadratic,
        piecewise,
        pending,
    ]
}

#[test]
fn round_trips_every_kind_of_record() {
    for (sequence, record) in records().into_iter().enumerate() {
        let slot = encode(&record, sequence as u32);
        assert_eq!(decode(&slot.0), Some((sequence as u32, record)));
    }
}

#[test]
fn survives_a_reboot() {
    let mut sector = erased();
    assert_eq!(load(&sector), (CalibrationRecord::DEFAULT, Source::Empty));
    let record = records().remove(4);
    store(&mut sector, &record);
    assert_eq!(
        load(&sector),
        (
            record,
            Source::Record {
                slot: 0,
                sequence: 1
            }
        )
    );
}

#[test]
fn migrates_a_factor_of_earlier_firmware() {
    let sector = legacy_sector(2, 0.0923);
    let (record, source) = load(&sector);
    assert_eq!(source, Source::Legacy);
    assert_eq!(record.active, Calibration::Factor(0.0923));
    assert_eq!(record.model(), Model::Linear);
    assert!(record.points.is_empty(), "earlier firmware kept no points");
    assert_eq!(record.pending, None);
    // Readings convert as before: (raw - tare) * factor / 1000.
    assert_eq!(
        record.active.net_kg(512_345, 1_234, 0.0).to_bits(),
        ((512_345 - 1_234) as f32 * 0.0923 / 1000.0).to_bits()
    );

    // The first write keeps the old record, in case it is interrupted.
    let mut sector = sector;
    let write = store(&mut sector, &record);
    assert!(!write.erase);
    assert_eq!(write.offset, SLOT_SIZE);
    assert_eq!(
        load(&sector).1,
        Source::Record {
            slot: 1,
            sequence: 1
        }
    );
}

#[test]
fn ignores_legacy_records_it_cannot_use() {
    // The HX711 front end of version 1, an implausible factor, and a bad checksum.
    let mut bad_checksum = legacy_sector(2, 0.0923);
    bad_checksum.0[12] ^= 1;
    for sector in [
        legacy_sector(1, 0.0923),
        legacy_sector(2, 50.0),
        bad_checksum,
    ] {
        assert_eq!(load(&sector), (CalibrationRecord::DEFAULT, Source::Invalid));
    }
}

#[test]
fn appends_until_the_log_is_full() {
    let records = records();
    let mut sector = erased();
    for i in 0..SLOTS {
        let write = store(&mut sector, &records[i % records.len()]);
        assert!(!write.erase);
        assert_eq!(write.offset, i * SLOT_SIZE);
    }
    let write = store(&mut sector, &records[2]);
    assert!(write.erase);
    assert_eq!(write.offset, 0);
    assert_eq!(
        load(&sector),
        (
            records[2].clone(),
            Source::Record {
                slot: 0,
                sequence: SLOTS as u32 + 1
            }
        )
    );
}

#[test]
fn an_interrupted_write_keeps_the_previous_record() {
    let records = records();
    let mut sector = erased();
    store(&mut sector, &records[3]);

    // The power fails halfway through the next record.
    let write = plan_write(&sector, &records[4]);
    apply(&mut sector, &write, 100);
    assert_eq!(load(&sector).0, records[3]);

    // The next write skips the damaged slot.
    let write = store(&mut sector, &records[5]);
    assert_eq!(write.offset, 2 * SLOT_SIZE);
    assert_eq!(
        load(&sector),
        (
            records[5].clone(),
            Source::Record {
                slot: 2,
                sequence: 2
            }
        )
    );
}

#[test]
fn a_corrupt_record_falls_back_to_an_older_one() {
    let records = records();
    let mut sector = erased();
    store(&mut sector, &records[2]);
    store(&mut sector, &records[4]);
    // A flipped bit in a point of the newest record.
    sector.0[SLOT_SIZE + 60] ^= 0x10;
    assert_eq!(load(&sector).0, records[2]);

    // With every record damaged, the default calibration is used.
    sector.0[60] ^= 0x10;
    assert_eq!(load(&sector), (CalibrationRecord::DEFAULT, Source::Invalid));
}

#[test]
fn an_interrupted_erase_reads_as_the_default() {
    let mut sector = erased();
    store(&mut sector, &records()[4]);
    // Erasing stopped partway, leaving random bits.
    let mut state = 0x1234_5678u32;
    for byte in sector.0.iter_mut().take(1500) {
        state = state.wrapping_mul(1_103_515_245).wrapping_add(12_345);
        *byte &= (state >> 16) as u8 | 0x81;
    }
    assert_eq!(load(&sector), (CalibrationRecord::DEFAULT, Source::Invalid));
}

#[test]
fn rejects_unsupported_and_invalid_records() {
    let record = records().remove(4);
    let valid = encode(&record, 7);

    // A newer version.
    let mut newer = Sector(valid.0.repeat(SLOTS).try_into().unwrap());
    newer.0[4] = 4;
    for slot in 1..SLOTS {
        newer.0[slot * SLOT_SIZE..(slot + 1) * SLOT_SIZE].fill(0xFF);
    }
    assert_eq!(load(&newer).1, Source::Invalid);

    // A valid CRC around a decreasing curve.
    let mut decreasing = record;
    if let Calibration::Fitted(Fit {
        curve: Curve::Piecewise { m1, .. },
        ..
    }) = &mut decreasing.active
    {
        *m1 = -*m1;
    }
    assert!(decode(&encode(&decreasing, 1).0).is_none());
}

#[test]
fn the_default_is_the_factor_of_earlier_firmware() {
    assert_eq!(
        CalibrationRecord::DEFAULT.active,
        Calibration::Factor(DEFAULT_FACTOR)
    );
}
