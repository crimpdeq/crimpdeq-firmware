//! Flash storage of the settings that persist across reboots.
//!
//! Writing a record erases its whole sector, so each setting has its own partition: a power cut
//! while one setting is written cannot lose the other.

use defmt::{Format, error, info};
use esp_bootloader_esp_idf::partitions::{
    PARTITION_TABLE_MAX_LEN,
    PartitionEntry,
    read_partition_table,
};
use esp_hal::{Blocking, flash::Flash};

use crate::ads1220::{DEFAULT_CALIBRATION_FACTOR, SampleRate, is_valid_calibration_factor};

/// Size of a record: magic, version, value and checksum, as little-endian u32 words.
const RECORD_SIZE: usize = 16;
const CALIBRATION_RECORD: Record = Record {
    // The last 4 KiB sector of the 4 MiB flash, where earlier firmware stored the calibration.
    partition: "calib",
    magic: 0x4344_5146,
    // Version 2 invalidates calibration factors saved for the old HX711 front end.
    version: 2,
};
const SAMPLE_RATE_RECORD: Record = Record {
    partition: "settings",
    magic: 0x4344_5352,
    version: 1,
};

/// Settings that the device applies in RAM and persists to flash.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Settings {
    pub sample_rate: SampleRate,
    pub calibration_factor: f32,
}

impl Settings {
    pub const DEFAULT: Self = Self {
        sample_rate: SampleRate::DEFAULT,
        calibration_factor: DEFAULT_CALIBRATION_FACTOR,
    };
}

#[derive(Format)]
pub enum StorageError {
    /// The partition table has no usable partition for the record.
    MissingPartition,
    /// Flash access failed.
    Flash,
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
    flash: Flash<'static, Blocking>,
    /// Partitions of the records, `None` when the partition table lacks them.
    calibration_partition: Option<PartitionEntry>,
    sample_rate_partition: Option<PartitionEntry>,
}

impl Storage {
    /// Looks up the record partitions in the partition table.
    pub fn new(mut flash: Flash<'static, Blocking>) -> Self {
        let mut table_buffer = [0u8; PARTITION_TABLE_MAX_LEN];
        let table = read_partition_table(&mut flash, &mut table_buffer)
            .inspect_err(|e| error!("Failed to read the partition table: {:?}", e))
            .ok();
        let partition = |record: &Record| {
            let partition = table
                .as_ref()?
                .iter()
                .find(|partition| partition.label_as_str() == record.partition);
            match partition {
                Some(partition) if partition.len() >= Flash::SECTOR_SIZE => Some(partition),
                Some(_) => {
                    error!("Partition {} is too small", record.partition);
                    None
                }
                None => {
                    error!(
                        "Partition {} is missing, flash the partition table in partitions.csv",
                        record.partition
                    );
                    None
                }
            }
        };
        let calibration_partition = partition(&CALIBRATION_RECORD);
        let sample_rate_partition = partition(&SAMPLE_RATE_RECORD);

        Self {
            flash,
            calibration_partition,
            sample_rate_partition,
        }
    }

    /// Reads the stored settings, with the default for each one that is missing or invalid.
    pub fn read(&mut self) -> Settings {
        let calibration_factor = match self
            .read_record(self.calibration_partition, &CALIBRATION_RECORD)
            .map(f32::from_bits)
        {
            Some(factor) if is_valid_calibration_factor(factor) => {
                info!("Calibration factor read from flash: {}", factor);
                factor
            }
            _ => {
                info!("Using default calibration factor");
                DEFAULT_CALIBRATION_FACTOR
            }
        };
        let sample_rate = match self
            .read_record(self.sample_rate_partition, &SAMPLE_RATE_RECORD)
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
            calibration_factor,
        }
    }

    pub fn write_calibration_factor(&mut self, factor: f32) -> Result<(), StorageError> {
        self.write_record(
            self.calibration_partition,
            &CALIBRATION_RECORD,
            factor.to_bits(),
        )
    }

    pub fn write_sample_rate(&mut self, sample_rate: SampleRate) -> Result<(), StorageError> {
        let sps = u32::from(sample_rate.sps());
        self.write_record(self.sample_rate_partition, &SAMPLE_RATE_RECORD, sps)
    }

    /// Reads a stored value, or `None` when the record is missing or corrupt.
    fn read_record(&mut self, partition: Option<PartitionEntry>, record: &Record) -> Option<u32> {
        let mut bytes = [0; RECORD_SIZE];
        if let Err(e) = partition?
            .as_flash_region(&mut self.flash)
            .and_then(|mut region| region.read(0, &mut bytes))
        {
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
        partition: Option<PartitionEntry>,
        record: &Record,
        value: u32,
    ) -> Result<(), StorageError> {
        let partition = partition.ok_or(StorageError::MissingPartition)?;
        let words = [record.magic, record.version, value, record.checksum(value)];
        let mut bytes = [0; RECORD_SIZE];
        for (chunk, word) in bytes.as_chunks_mut::<4>().0.iter_mut().zip(words) {
            *chunk = word.to_le_bytes();
        }

        partition
            .as_flash_region(&mut self.flash)
            .and_then(|mut region| {
                region.erase(0, Flash::SECTOR_SIZE)?;
                region.write(0, &bytes)
            })
            .map_err(|e| {
                error!(
                    "Failed to write record {:#x} to flash: {:?}",
                    record.magic, e
                );
                StorageError::Flash
            })
    }
}
