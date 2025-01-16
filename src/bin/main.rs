#![no_std]
#![no_main]

use bleps::{
    ad_structure::{
        create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    },
    attribute_server::{AttributeServer, NotificationData, WorkResult},
    gatt, Ble, HciConnector,
};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    entry,
    gpio::{Input, Pull},
    time,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_wifi::{ble::controller::BleConnector, init};

use tindeq::progressor::{DataPoint, ResponseCode, MAX_PAYLOAD_SIZE};

/// Progressor Primary Service UUID
const SERVICE_UUID: &str = "7e4e1701-1ea6-40c9-9dcc-13d34ffead57";
/// Progressor Data Point Characteristic UUID
const DATA_POINT_UUID: &str = "7e4e1702-1ea6-40c9-9dcc-13d34ffead57";
/// Progressor Control Point Characteristic UUID
const CONTROL_POINT_UUID: &str = "7e4e1703-1ea6-40c9-9dcc-13d34ffead57";

extern crate alloc;

const SCAN_RESPONSE_DATA: &[u8] = &[
    18, // Length
    17, 0x07, 0x57, 0xad, 0xfe, 0x4f, 0xd3, 0x13, 0xcc, 0x9d, 0xc9, 0x40, 0xa6, 0x1e, 0x01, 0x17,
    0x4e, 0x7e, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
];

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    esp_println::logger::init_logger_from_env();

    esp_alloc::heap_allocator!(72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let init = init(
        timg0.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    // To be deleted
    let button = Input::new(peripherals.GPIO9, Pull::Down);
    let mut debounce_cnt = 500;
    let mut counter: u8 = 0;

    // Fake data
    let weigth: f32 = 20.4;
    let timestamp: u32 = 123456;
    let var_name = {
        let weigth_bytes = weigth.to_le_bytes();
        let timestamp_bytes = timestamp.to_le_bytes();
        let mut value = [0; 8];
        value[..4].copy_from_slice(&weigth_bytes);
        value[4..].copy_from_slice(&timestamp_bytes);
        value
    };
    let value: [u8; 8] = var_name;
    let mut data: [u8; 10] = [
        0x01, 8, value[0], value[1], value[2], value[3], value[4], value[5], value[6], counter,
    ];

    let mut bluetooth = peripherals.BT;

    let now = || time::now().duration_since_epoch().to_millis();
    loop {
        let connector = BleConnector::new(&init, &mut bluetooth);
        let hci = HciConnector::new(connector, now);
        let mut ble = Ble::new(&hci);

        println!("ble.init: {:?}", ble.init());
        println!(
            "ble.cmd_set_le_advertising_parameters: {:?}",
            ble.cmd_set_le_advertising_parameters()
        );
        // Todo: See diferences between this and the one below
        // println!(
        //     " ble.cmd_set_le_advertising_data: {:?}",
        //     ble.cmd_set_le_advertising_data(
        //         create_advertising_data(&[
        //             AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
        //             AdStructure::CompleteLocalName("Progressor_2639"),
        //         ])
        //         .unwrap()
        //     )
        // );
        println!(
            "{:?}",
            ble.cmd_set_le_advertising_data(bleps::Data::new(&[
                20,
                2,
                0x01,
                0x02 | 0x04,
                ("Progressor_2639".len() + 1) as u8,
                0x9,
                b'P',
                b'r',
                b'o',
                b'g',
                b'r',
                b'e',
                b's',
                b's',
                b'o',
                b'r',
                b'_',
                b'2',
                b'6',
                b'3',
                b'9',
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            ]))
        );

        println!(
            "{:?}",
            ble.cmd_set_le_scan_rsp_data(bleps::Data::new(SCAN_RESPONSE_DATA))
        );
        println!("{:?}", ble.cmd_set_le_advertise_enable(true));

        println!("started advertising");

        let mut data_point_read = |_offset: usize, data: &mut [u8]| {
            data[..20].copy_from_slice(&b"Data Point Read"[..]);
            17
        };

        let mut control_point_write = |_, data: &[u8]| {
            println!("Control Point Received: 0x{:x?}", data);
        };

        let mut service_change_read = |_offset: usize, data: &mut [u8]| {
            data[..20].copy_from_slice(&b"Service Change Read"[..]);
            17
        };

        let device_name = b"Progressor_2639";
        let appearance = b"[0] Unknown";
        let ppcp_val = b"Connection Interval: 50.00ms - 65.00ms, Max Latency:6ms, Suppervision Timeout Multiplier: 400ms";
        let car_val = b"Address resolution supported";
        let ccc_vla = b"Notifications and indications disabled";
        gatt!([
            service {
                uuid: "1800",
                characteristics: [
                    characteristic {
                        uuid: "2a00",
                        value: device_name,
                    },
                    characteristic {
                        uuid: "2a01",
                        value: appearance,
                    },
                    characteristic {
                        uuid: "2a04",
                        value: ppcp_val,
                    },
                    characteristic {
                        uuid: "2aa6",
                        value: car_val,
                    },
                ],
            },
            service {
                uuid: "1801",
                characteristics: [characteristic {
                    uuid: "2a05",
                    read: service_change_read,
                    descriptors: [descriptor {
                        uuid: "2902",
                        value: ccc_vla,
                    },],
                },],
            },
            service {
                uuid: "7e4e1701-1ea6-40c9-9dcc-13d34ffead57",
                characteristics: [
                    characteristic {
                        name: "data_point",
                        uuid: "7e4e1702-1ea6-40c9-9dcc-13d34ffead57",
                        notify: true,
                        read: data_point_read,
                    },
                    characteristic {
                        name: "control_point",
                        uuid: "7e4e1703-1ea6-40c9-9dcc-13d34ffead57",
                        write: control_point_write,
                        // TODO: Is ther a WriteNoResponse?
                    },
                ],
            },
        ]);

        let mut rng = bleps::no_rng::NoRng;
        let mut srv = AttributeServer::new(&mut ble, &mut gatt_attributes, &mut rng);

        loop {
            let mut notification = None;

            if button.is_low() && debounce_cnt > 0 {
                debounce_cnt -= 1;
                if debounce_cnt == 0 {
                    counter += 1;
                    data[9] = counter;

                    let mut cccd = [0u8; 1];
                    if let Some(1) =
                        srv.get_characteristic_value(data_point_notify_enable_handle, 0, &mut cccd)
                    {
                        // if notifications enabled
                        if cccd[0] == 1 {
                            notification = Some(NotificationData::new(data_point_handle, &data));
                        }
                    }
                }
            };

            if button.is_high() {
                debounce_cnt = 500;
            }

            match srv.do_work_with_notification(notification) {
                Ok(res) => {
                    if let WorkResult::GotDisconnected = res {
                        break;
                    }
                }
                Err(err) => {
                    println!("{:?}", err);
                }
            }
        }
    }
}
