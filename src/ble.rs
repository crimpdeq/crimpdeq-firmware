//! BLE module
//!
//! This module provides the BLE functionality for the Progressor.
//! It includes the BLE advertising data, the GATT server, and the BLE connection.
#![allow(clippy::needless_borrows_for_generic_args)]
use defmt::{debug, info};
use embassy_time::{Duration, with_timeout};
use trouble_host::prelude::*;

use crate::progressor::{CONTROL_POINT_MAX_PAYLOAD_SIZE, DataPoint};

/// Max number of connections
pub const CONNECTIONS_MAX: usize = 1;
/// Max number of L2CAP channels.
pub const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att
/// Size of L2CAP packets
pub const L2CAP_MTU: usize = 255;

/// Time spent advertising at the default 160 ms interval after boot or a disconnect.
const FAST_ADVERTISING_DURATION: Duration = Duration::from_secs(30);
/// Advertising interval once fast advertising ends.
const SLOW_ADVERTISING_INTERVAL: Duration = Duration::from_micros(1_022_500);

/// Name advertised by the device.
pub const DEVICE_NAME: &str = env!("DEVICE_NAME");

/// BLE AD type for flags.
const AD_TYPE_FLAGS: u8 = 0x01;
/// BLE AD type for the complete local name.
const AD_TYPE_COMPLETE_LOCAL_NAME: u8 = 0x09;
/// LE General Discoverable Mode flag.
const FLAG_LE_GENERAL_DISC_MODE: u8 = 0x02;
/// BR/EDR Not Supported flag.
const FLAG_BR_EDR_NOT_SUPPORTED: u8 = 0x04;
/// Maximum size of legacy advertising data.
const MAX_ADVERTISING_DATA_LEN: usize = 31;
/// Size of the advertising data: flags (3 bytes), name header (2 bytes) and name.
const ADVERTISING_DATA_LEN: usize = 5 + DEVICE_NAME.len();
const _: () = assert!(
    ADVERTISING_DATA_LEN <= MAX_ADVERTISING_DATA_LEN,
    "DEVICE_NAME does not fit in the advertising data"
);
/// Progressor BLE advertising data: flags and complete local name.
const ADVERTISING_DATA: [u8; ADVERTISING_DATA_LEN] = {
    let name = DEVICE_NAME.as_bytes();
    let mut data = [0; ADVERTISING_DATA_LEN];
    data[0] = 2;
    data[1] = AD_TYPE_FLAGS;
    data[2] = FLAG_LE_GENERAL_DISC_MODE | FLAG_BR_EDR_NOT_SUPPORTED;
    data[3] = name.len() as u8 + 1;
    data[4] = AD_TYPE_COMPLETE_LOCAL_NAME;
    let mut i = 0;
    while i < name.len() {
        data[5 + i] = name[i];
        i += 1;
    }
    data
};

/// BLE AD type for a complete list of 128-bit service UUIDs.
const AD_TYPE_COMPLETE_128BIT_SERVICE_UUIDS: u8 = 0x07;
/// Progressor service UUID encoded in BLE little-endian order.
const PROGRESSOR_SERVICE_UUID_LE: [u8; 16] = [
    0x57, 0xad, 0xfe, 0x4f, 0xd3, 0x13, 0xcc, 0x9d, 0xc9, 0x40, 0xa6, 0x1e, 0x01, 0x17, 0x4e, 0x7e,
];
/// Progressor BLE Scan Response.
const SCAN_RESPONSE_DATA: &[u8] = &[
    17, // 1 byte type + 16 byte UUID
    AD_TYPE_COMPLETE_128BIT_SERVICE_UUIDS,
    PROGRESSOR_SERVICE_UUID_LE[0],
    PROGRESSOR_SERVICE_UUID_LE[1],
    PROGRESSOR_SERVICE_UUID_LE[2],
    PROGRESSOR_SERVICE_UUID_LE[3],
    PROGRESSOR_SERVICE_UUID_LE[4],
    PROGRESSOR_SERVICE_UUID_LE[5],
    PROGRESSOR_SERVICE_UUID_LE[6],
    PROGRESSOR_SERVICE_UUID_LE[7],
    PROGRESSOR_SERVICE_UUID_LE[8],
    PROGRESSOR_SERVICE_UUID_LE[9],
    PROGRESSOR_SERVICE_UUID_LE[10],
    PROGRESSOR_SERVICE_UUID_LE[11],
    PROGRESSOR_SERVICE_UUID_LE[12],
    PROGRESSOR_SERVICE_UUID_LE[13],
    PROGRESSOR_SERVICE_UUID_LE[14],
    PROGRESSOR_SERVICE_UUID_LE[15],
];

// GATT Server definition
#[gatt_server]
pub struct Server {
    pub progressor: ProgressorService,
}

/// Tindeq Progressor service
#[gatt_service(uuid = "7e4e1701-1ea6-40c9-9dcc-13d34ffead57")]
pub struct ProgressorService {
    /// Data Point - for receiving data from the Progressor
    #[characteristic(uuid = "7e4e1702-1ea6-40c9-9dcc-13d34ffead57", notify)]
    pub data_point: DataPoint,

    /// Control Point - for sending commands to the Progressor
    #[characteristic(
        uuid = "7e4e1703-1ea6-40c9-9dcc-13d34ffead57",
        write,
        write_without_response
    )]
    pub control_point: [u8; CONTROL_POINT_MAX_PAYLOAD_SIZE],
}

/// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
pub async fn advertise<'values, 'server, C: Controller>(
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let advertisement = Advertisement::ConnectableScannableUndirected {
        adv_data: &ADVERTISING_DATA,
        scan_data: SCAN_RESPONSE_DATA,
    };

    debug!("Advertising BLE");
    let advertiser = peripheral
        .advertise(&Default::default(), advertisement)
        .await?;
    let conn = match with_timeout(FAST_ADVERTISING_DURATION, advertiser.accept()).await {
        Ok(conn) => conn?,
        Err(_) => {
            // Dropping the fast advertiser stops it.
            debug!("Switching to slow BLE advertising");
            let params = AdvertisementParameters {
                interval_min: SLOW_ADVERTISING_INTERVAL,
                interval_max: SLOW_ADVERTISING_INTERVAL,
                ..Default::default()
            };
            peripheral
                .advertise(&params, advertisement)
                .await?
                .accept()
                .await?
        }
    };
    let conn = conn.with_attribute_server(server)?;
    info!("BLE connection established");
    Ok(conn)
}
