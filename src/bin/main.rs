#![no_std]
#![no_main]

use core::cell::RefCell;

use bleps::{
    async_attribute_server::AttributeServer,
    asynch::Ble,
    // ad_structure::{
    //     create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    // },
    attribute_server::NotificationData,
    gatt,
};
use bytemuck::bytes_of;
use embassy_executor::Spawner;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Input, Pull},
    rng::Rng,
    time,
    timer::{systimer::SystemTimer, timg::TimerGroup},
    Config,
};
use esp_println::println;
use esp_wifi::{ble::controller::BleConnector, init, EspWifiController};

use tindeq::progressor::{ControlOpCode, DataPoint, ResponseCode};

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

const SCAN_RESPONSE_DATA: &[u8] = &[
    18, // Length
    17, 0x07, 0x57, 0xad, 0xfe, 0x4f, 0xd3, 0x13, 0xcc, 0x9d, 0xc9, 0x40, 0xa6, 0x1e, 0x01, 0x17,
    0x4e, 0x7e, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
];

const ADVERTISING_DATA: &[u8] = &[
    20, // Length
    2,
    0x01,
    0x02 | 0x04, // LE General Discoverable Mode, BR/EDR Not Supported
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
];

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) -> ! {
    let config = Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_println::logger::init_logger_from_env();

    esp_alloc::heap_allocator!(72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let init = &*mk_static!(
        EspWifiController<'static>,
        init(
            timg0.timer0,
            Rng::new(peripherals.RNG),
            peripherals.RADIO_CLK,
        )
        .unwrap()
    );

    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);

    // To be deleted
    let button = Input::new(peripherals.GPIO9, Pull::Down);
    let pin_ref = RefCell::new(button);
    let pin_ref = &pin_ref;
    let mut debounce_cnt = 500;
    let mut counter: u8 = 0;

    let mut bluetooth = peripherals.BT;
    let connector = BleConnector::new(init, &mut bluetooth);
    let now = || time::now().duration_since_epoch().to_millis();
    let mut ble = Ble::new(connector, now);
    println!("Connector created");

    loop {
        println!("ble.init: {:?}", ble.init().await);
        println!(
            "ble.cmd_set_le_advertising_parameters: {:?}",
            ble.cmd_set_le_advertising_parameters().await
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
            "ble.cmd_set_le_advertising_data: {:?}",
            ble.cmd_set_le_advertising_data(Data::new(ADVERTISING_DATA))
                .await
        );

        println!(
            "ble.cmd_set_le_scan_rsp_data: {:?}",
            ble.cmd_set_le_scan_rsp_data(Data::new(SCAN_RESPONSE_DATA))
                .await
        );
        println!(
            "ble.cmd_set_le_advertise_enable: {:?}",
            ble.cmd_set_le_advertise_enable(true).await
        );

        println!("Started advertising");

        let mut data_point_read = |_offset: usize, data: &mut [u8]| {
            data[..20].copy_from_slice(&b"Data Point Read"[..]);
            17
        };

        let mut control_point_write = |_, data: &[u8]| {
            println!("Control Point Received: 0x{:x?}", data);

            match ControlOpCode::from(data[0]) {
                ControlOpCode::TareScale => {
                    println!("TareScale");
                }
                ControlOpCode::StartMeasurement => {
                    println!("StartMeasurement");
                }
                ControlOpCode::StopMeasurement => {
                    println!("StopMeasurement");
                }
                ControlOpCode::GetAppVersion => {
                    println!("GetAppVersion");
                }
                ControlOpCode::Shutdown => {
                    println!("Shutdown");
                }
                ControlOpCode::SampleBattery => {
                    println!("SampleBattery");
                }
                ControlOpCode::GetProgressorId => {
                    println!("GetProgressorId");
                    // Notify the data_point with the progressor id
                }
            }
        };

        let mut service_change_read = |_offset: usize, data: &mut [u8]| {
            data[..20].copy_from_slice(&b"Service Change Read"[..]);
            17
        };

        // TODO: Avoid using the gatt! macro, replace the uuids with constants and improve the values
        let device_name = b"Progressor_2639";
        let appearance = b"[0] Unknown";
        let ppcp_val = b"Connection Interval: 50.00ms - 65.00ms, Max Latency:6ms, Suppervision Timeout Multiplier: 400ms";
        let car_val = b"Address resolution supported";
        let ccc_vla = b"Notifications and indications disabled";
        gatt!([
            service {
                // Generic Access
                uuid: "1800",
                characteristics: [
                    // Device Name
                    characteristic {
                        uuid: "2a00",
                        value: device_name,
                    },
                    // Appearance
                    characteristic {
                        uuid: "2a01",
                        value: appearance,
                    },
                    // Peripheral Preferred Connection Parameters
                    characteristic {
                        uuid: "2a04",
                        value: ppcp_val,
                    },
                    // Central Address Resolution
                    characteristic {
                        uuid: "2aa6",
                        value: car_val,
                    },
                ],
            },
            // Generic Attribute
            service {
                uuid: "1801",
                characteristics: [
                    // Service Changed
                    characteristic {
                        uuid: "2a05",
                        read: service_change_read,
                        descriptors: [
                            // Client Characteristic Configuration
                            descriptor {
                                uuid: "2902",
                                value: ccc_vla,
                            },
                        ],
                    },
                ],
            },
            service {
                // Progressor Primary
                uuid: "7e4e1701-1ea6-40c9-9dcc-13d34ffead57",
                characteristics: [
                    // Progressor Data Point
                    characteristic {
                        name: "data_point",
                        uuid: "7e4e1702-1ea6-40c9-9dcc-13d34ffead57",
                        notify: true,
                        read: data_point_read,
                    },
                    /// Progressor Control Point
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

        let counter = &RefCell::new(0u8);
        let weigth = &RefCell::new(0.0f32);
        let timestamp = &RefCell::new(0u32);
        let mut notifier = || {
            // TODO how to check if notifications are enabled for the characteristic?
            // maybe pass something into the closure which just can query the characteristic
            // value probably passing in the attribute server won't work?

            async {
                pin_ref.borrow_mut().wait_for_rising_edge().await;
                {
                    // Fake data
                    let mut counter = (&*counter).borrow_mut();
                    *counter += 1;
                    let mut weigth = (&*weigth).borrow_mut();
                    *weigth = *counter as f32 + (*counter as f32 / 10.0);
                    let mut timestamp = (&*timestamp).borrow_mut();
                    *timestamp = time::now().duration_since_epoch().to_micros() as u32;
                }
                let measurement =
                    ResponseCode::WeigthtMeasurement(*weigth.borrow(), *timestamp.borrow());
                let data_point = DataPoint::new(measurement);
                let data = bytes_of(&data_point);
                NotificationData::new(data_point_handle, &data)
            }
        };

        srv.run(&mut notifier).await.unwrap();
        // loop {
        //     let mut notification = None;

        //     if button.is_low() && debounce_cnt > 0 {
        //         debounce_cnt -= 1;
        //         if debounce_cnt == 0 {
        //             // Fake data
        //             let weigth = counter as f32 + (counter as f32 / 10.0);
        //             let timestamp = time::now().duration_since_epoch().to_micros() as u32;
        //             let measurement = ResponseCode::WeigthtMeasurement(weigth, timestamp);
        //             let data_point = DataPoint::new(measurement);
        //             let measurement_data = bytes_of(&data_point);
        //             counter += 1;

        //             let mut cccd = [0u8; 1];
        //             if let Some(1) =
        //                 srv.get_characteristic_value(data_point_notify_enable_handle, 0, &mut cccd)
        //             {
        //                 // if notifications enabled
        //                 if cccd[0] == 1 {
        //                     notification =
        //                         Some(NotificationData::new(data_point_handle, measurement_data));
        //                 }
        //             }
        //         }
        //     };

        //     if button.is_high() {
        //         debounce_cnt = 500;
        //     }

        //     match srv.do_work_with_notification(notification) {
        //         Ok(res) => {
        //             if let WorkResult::GotDisconnected = res {
        //                 break;
        //             }
        //         }
        //         Err(err) => {
        //             println!("{:?}", err);
        //         }
        //     }
        // }
    }
}
