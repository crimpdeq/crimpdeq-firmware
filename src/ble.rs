/// BLE module
///
/// This module provides the BLE functionality for the Progressor.
/// It includes the BLE advertising data, the GATT server, and the BLE connection.
use defmt::{debug, info};
use trouble_host::{
    advertise::{AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE},
    prelude::*,
};

use crate::progressor::{DataPoint, MAX_CONTROL_PAYLOAD_SIZE};

/// Max number of connections
pub const CONNECTIONS_MAX: usize = 1;
/// Max number of L2CAP channels.
pub const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att
/// Size of L2CAP packets
pub const L2CAP_MTU: usize = 255;

/// Progressor service UUID in little-endian byte order for advertising payloads.
const PROGRESSOR_SERVICE_UUID_LE: [u8; 16] = [
    0x57, 0xad, 0xfe, 0x4f, 0xd3, 0x13, 0xcc, 0x9d, 0xc9, 0x40, 0xa6, 0x1e, 0x01, 0x17, 0x4e, 0x7e,
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
    pub control_point: [u8; MAX_CONTROL_PAYLOAD_SIZE], // Buffer for command data
}

/// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
pub async fn advertise<'values, 'server, C: Controller>(
    name: &'values str,
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut advertising_data = [0u8; 31];
    let advertising_data_len = build_advertising_data(name.as_bytes(), &mut advertising_data)
        .expect("valid advertising data");
    let mut scan_response_data = [0u8; 31];
    let scan_response_data_len =
        build_scan_response_data(&mut scan_response_data).expect("valid scan response data");

    debug!("Advertising BLE");
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &advertising_data[..advertising_data_len],
                scan_data: &scan_response_data[..scan_response_data_len],
            },
        )
        .await?;
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    info!("BLE connection established");
    Ok(conn)
}

fn build_advertising_data(name: &[u8], dest: &mut [u8; 31]) -> Result<usize, ()> {
    AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(name),
        ],
        dest,
    )
    .map_err(|_| ())
}

fn build_scan_response_data(dest: &mut [u8; 31]) -> Result<usize, ()> {
    AdStructure::encode_slice(
        &[AdStructure::ServiceUuids128(&[PROGRESSOR_SERVICE_UUID_LE])],
        dest,
    )
    .map_err(|_| ())
}
