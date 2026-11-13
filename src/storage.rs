//! Flash storage of the settings that persist across reboots.
//!
//! Each setting has its own partition, so writing one cannot lose the other. The sample rate is a
//! 16-byte record, rewritten in place. The calibration is a log of records in its sector (see
//! [`crimpdeq_calibration::persist`]), so an interrupted write keeps the previous calibration.

use crimpdeq_calibration::{
    CalibrationRecord,
    persist::{self, SECTOR_SIZE, SLOT_SIZE, Sector},
};
use defmt::{Format, error, info, warn};
use esp_bootloader_esp_idf::partitions::{PARTITION_TABLE_MAX_LEN, read_partition_table};
use esp_storage::FlashStorage;

use crate::ads1220::SampleRate;

/// Size of the sample rate record: magic, version, value and checksum, as little-endian u32 words.
const RECORD_SIZE: usize = 16;
/// The last 4 KiB sector of the 4 MiB flash, where earlier firmware stored the calibration.
const CALIBRATION_PARTITION: &str = "calib";
const SAMPLE_RATE_RECORD: Record = Record {
    partition: "settings",
    magic: 0x4344_5352,
    version: 1,
};

/// Settings that the device applies in RAM and persists to flash.
#[derive(Clone, Debug, PartialEq)]
pub struct Settings {
    pub sample_rate: SampleRate,
    pub calibration: CalibrationRecord,
}

impl Settings {
    pub const DEFAULT: Self = Self {
        sample_rate: SampleRate::DEFAULT,
        calibration: CalibrationRecord::DEFAULT,
    };
}

#[derive(Format)]
pub enum StorageError {
    /// The partition table has no usable partition for the record.
    MissingPartition,
    /// Flash access failed.
    Flash,
    /// The written calibration record reads back different.
    Verify,
}

/// A setting stored at the start of its own partition.
struct Record {
    partition: &'static str,
    magic: u32,
    version: u32,
}

impl Record {
    fn checksum(&self, value: u32) -> u32 {
        self.magic ^ self.version ^ value ^ 0xA5A5_5A5A
    }
}

pub struct Storage {
    flash: FlashStorage<'static>,
    /// Flash offsets of the records, `None` when the partition table lacks their partition.
    calibration_offset: Option<u32>,
    sample_rate_offset: Option<u32>,
    /// Image of the calibration sector, kept here rather than on the task stack.
    sector: Sector,
}

impl Storage {
    /// Looks up the record partitions in the partition table.
    pub fn new(mut flash: FlashStorage<'static>) -> Self {
        let mut table_buffer = [0u8; PARTITION_TABLE_MAX_LEN];
        let table = read_partition_table(&mut flash, &mut table_buffer)
            .inspect_err(|e| error!("Failed to read the partition table: {:?}", e))
            .ok();
        let offset = |label: &str, size: usize| {
            let partition = table
                .as_ref()?
                .iter()
                .find(|partition| partition.label_as_str() == label);
            match partition {
                Some(partition) if partition.len() as usize >= size => Some(partition.offset()),
                Some(_) => {
                    error!("Partition {} is too small", label);
                    None
                }
                None => {
                    error!(
                        "Partition {} is missing, flash the partition table in partitions.csv",
                        label
                    );
                    None
                }
            }
        };
        let calibration_offset = offset(CALIBRATION_PARTITION, SECTOR_SIZE);
        let sample_rate_offset = offset(SAMPLE_RATE_RECORD.partition, RECORD_SIZE);

        Self {
            flash,
            calibration_offset,
            sample_rate_offset,
            sector: Sector([0xFF; SECTOR_SIZE]),
        }
    }

    /// Reads the stored settings, with the default for each one that is missing or invalid.
    pub fn read(&mut self) -> Settings {
        let calibration = match self.read_calibration_sector() {
            Ok(()) => {
                let (record, source) = persist::load(&self.sector);
                match source {
                    persist::Source::Invalid => {
                        warn!("No valid calibration record, using the default calibration")
                    }
                    source => info!(
                        "Calibration read from flash ({:?}): {:?}, {} points",
                        source,
                        record.active,
                        record.points.len()
                    ),
                }
                record
            }
            Err(e) => {
                error!("Using the default calibration: {:?}", e);
                CalibrationRecord::DEFAULT
            }
        };
        let sample_rate = match self
            .read_record(self.sample_rate_offset, &SAMPLE_RATE_RECORD)
            .and_then(|sps| SampleRate::from_sps(u16::try_from(sps).ok()?))
        {
            Some(sample_rate) => sample_rate,
            None => {
                info!("Using default sample rate");
                SampleRate::DEFAULT
            }
        };

        Settings {
            sample_rate,
            calibration,
        }
    }

    /// Appends the calibration to its log, and reads it back.
    pub fn write_calibration(&mut self, record: &CalibrationRecord) -> Result<(), StorageError> {
        let offset = self
            .calibration_offset
            .ok_or(StorageError::MissingPartition)?;
        self.read_calibration_sector()?;
        let write = persist::plan_write(&self.sector, record);
        let flash_error = |e| {
            error!("Failed to write the calibration to flash: {:?}", e);
            StorageError::Flash
        };
        if write.erase {
            info!("Calibration log full, erasing its sector");
            self.flash
                .erase(offset, offset + SECTOR_SIZE as u32)
                .map_err(flash_error)?;
        }
        let slot_offset = offset + write.offset as u32;
        self.flash
            .write_nor(slot_offset, &write.slot.0)
            .map_err(flash_error)?;

        self.read_calibration_sector()?;
        let written = &self.sector.0[write.offset..write.offset + SLOT_SIZE];
        if persist::decode(written).is_none_or(|(_, read)| read != *record) {
            return Err(StorageError::Verify);
        }
        Ok(())
    }

    fn read_calibration_sector(&mut self) -> Result<(), StorageError> {
        let offset = self
            .calibration_offset
            .ok_or(StorageError::MissingPartition)?;
        self.flash
            .read_nor(offset, &mut self.sector.0)
            .map_err(|e| {
                error!("Failed to read the calibration from flash: {:?}", e);
                StorageError::Flash
            })
    }

    pub fn write_sample_rate(&mut self, sample_rate: SampleRate) -> Result<(), StorageError> {
        let sps = u32::from(sample_rate.sps());
        self.write_record(self.sample_rate_offset, &SAMPLE_RATE_RECORD, sps)
    }

    /// Reads a stored value, or `None` when the record is missing or corrupt.
    fn read_record(&mut self, offset: Option<u32>, record: &Record) -> Option<u32> {
        let mut bytes = [0; RECORD_SIZE];
        if let Err(e) = self.flash.read(offset?, &mut bytes) {
            error!(
                "Failed to read record {:#x} from flash: {:?}",
                record.magic, e
            );
            return None;
        }

        let words = bytes.as_chunks::<4>().0;
        let [magic, version, value, checksum] =
            core::array::from_fn(|i| u32::from_le_bytes(words[i]));
        if magic != record.magic || version != record.version {
            info!("Record {:#x} missing or version mismatch", record.magic);
            return None;
        }
        if checksum != record.checksum(value) {
            error!("Record {:#x} checksum mismatch", record.magic);
            return None;
        }
        Some(value)
    }

    fn write_record(
        &mut self,
        offset: Option<u32>,
        record: &Record,
        value: u32,
    ) -> Result<(), StorageError> {
        let offset = offset.ok_or(StorageError::MissingPartition)?;
        let words = [record.magic, record.version, value, record.checksum(value)];
        let mut bytes = [0; RECORD_SIZE];
        for (chunk, word) in bytes.as_chunks_mut::<4>().0.iter_mut().zip(words) {
            *chunk = word.to_le_bytes();
        }

        self.flash.write(offset, &bytes).map_err(|e| {
            error!(
                "Failed to write record {:#x} to flash: {:?}",
                record.magic, e
            );
            StorageError::Flash
        })
    }
}
