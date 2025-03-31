use arrayvec::ArrayVec;
use defmt::{debug, info};
use trouble_host::{
    advertise::{AD_FLAG_LE_LIMITED_DISCOVERABLE, SIMUL_LE_BR_HOST},
    prelude::*,
};

use crate::progressor::{DataPoint, MAX_PAYLOAD_SIZE};

/// Max number of connections
pub const CONNECTIONS_MAX: usize = 1;
/// Max number of L2CAP channels.
pub const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att
/// Size of L2CAP packets
pub const L2CAP_MTU: usize = 255;

/// Progressor BLE Scanning Response
const SCAN_RESPONSE_DATA: &[u8] = &[
    AD_FLAG_LE_LIMITED_DISCOVERABLE | SIMUL_LE_BR_HOST,
    7_u8, // BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE
    0x57,
    0xad,
    0xfe,
    0x4f,
    0xd3,
    0x13,
    0xcc,
    0x9d,
    0xc9,
    0x40,
    0xa6,
    0x1e,
    0x01,
    0x17,
    0x4e,
    0x7e, //UUID
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
    pub control_point: [u8; MAX_PAYLOAD_SIZE], // Buffer for command data
}

/// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
pub async fn advertise<'a, 'b, C: Controller>(
    peripheral: &mut Peripheral<'a, C>,
    server: &'b Server<'_>,
) -> Result<GattConnection<'a, 'b>, BleHostError<C::Error>> {
    let advertising_data = advertising_data(b"Progressor_7125").expect("Valid advertising data");

    debug!("Advertising BLE");
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: advertising_data.as_slice(),
                scan_data: SCAN_RESPONSE_DATA,
            },
        )
        .await?;
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    info!("BLE connection established");
    Ok(conn)
}

fn advertising_data(name: &[u8]) -> Result<ArrayVec<u8, 27>, ()> {
    // BLE AD type and flag constants
    const AD_TYPE_FLAGS: u8 = 0x01;
    const AD_TYPE_COMPLETE_LOCAL_NAME: u8 = 0x09;
    const FLAG_LE_GENERAL_DISC_MODE: u8 = 0x02;
    const FLAG_BR_EDR_NOT_SUPPORTED: u8 = 0x04;

    // Validate name length
    if name.len() > 24 {
        // Max allowed (27 - 3 bytes for flags)
        return Err(());
    }

    let mut adv_data: ArrayVec<u8, 27> = ArrayVec::new();

    // Add flags (length=2, type, flags)
    adv_data.push(2);
    adv_data.push(AD_TYPE_FLAGS);
    adv_data.push(FLAG_LE_GENERAL_DISC_MODE | FLAG_BR_EDR_NOT_SUPPORTED);

    // Add name (length=name.len()+1, type, name bytes)
    adv_data.push(name.len() as u8 + 1);
    adv_data.push(AD_TYPE_COMPLETE_LOCAL_NAME);
    adv_data.try_extend_from_slice(name).map_err(|_| ())?;

    Ok(adv_data)
}
