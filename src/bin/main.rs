#![no_std]
#![no_main]

use bleps::{
    ad_structure::{
        create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    },
    async_attribute_server::AttributeServer,
    asynch::Ble,
    attribute_server::NotificationData,
    gatt,
};
use bytemuck::bytes_of;
use core::cell::RefCell;
use critical_section::Mutex;
use defmt::{debug, error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    gpio::{Input, Level, Output, Pull},
    rng::Rng,
    time,
    timer::{systimer::SystemTimer, timg::TimerGroup},
    Config,
};
use esp_println as _;
use esp_wifi::{ble::controller::BleConnector, init, EspWifiController};
use loadcell::{hx711, LoadCell};

use tindeq::progressor::{ControlOpCode, DataPoint, ResponseCode};

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
    17, // AD_FLAG_LE_LIMITED_DISCOVERABLE | SIMUL_LE_BR_HOST
    0x07, 0x57, 0xad, 0xfe, 0x4f, 0xd3, 0x13, 0xcc, 0x9d, 0xc9, 0x40, 0xa6, 0x1e, 0x01, 0x17, 0x4e,
    0x7e, //UUID
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // Padding
];

pub const MEASURE_COMMAND_CHANNEL_SIZE: usize = 50;
pub type DataPointChannel = Channel<NoopRawMutex, DataPoint, MEASURE_COMMAND_CHANNEL_SIZE>;

static WEIGTH_TASK_ENABLED: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    let config = Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

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

    let sck = Output::new(peripherals.GPIO5, Level::Low);
    let dt = Input::new(peripherals.GPIO4, Pull::None);
    let delay = Delay::new();

    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);

    let connector = BleConnector::new(init, peripherals.BT);

    let channel = mk_static!(DataPointChannel, Channel::new());

    spawner.spawn(bt_task(connector, channel)).unwrap();
    spawner
        .spawn(measurement_task(channel, sck, dt, delay))
        .unwrap();

    loop {
        Timer::after(Duration::from_millis(10)).await;
    }
}

#[embassy_executor::task]
async fn bt_task(connector: BleConnector<'static>, channel: &'static DataPointChannel) {
    let now = || time::now().duration_since_epoch().to_millis();
    let mut ble = Ble::new(connector, now);
    loop {
        info!("Starting BLE");
        debug!("Initializing BLE");
        ble.init().await.unwrap();
        debug!("Setting advertising parameters");
        ble.cmd_set_le_advertising_parameters().await.unwrap();
        debug!("Setting advertising data");
        ble.cmd_set_le_advertising_data(
            create_advertising_data(&[
                AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                AdStructure::CompleteLocalName(env!("DEVICE_NAME")),
            ])
            .unwrap(),
        )
        .await
        .unwrap();
        debug!("Setting scan response data");
        ble.cmd_set_le_scan_rsp_data(Data::new(SCAN_RESPONSE_DATA))
            .await
            .unwrap();
        debug!("Setting advertising enable");
        ble.cmd_set_le_advertise_enable(true).await.unwrap();

        info!("Started advertising");

        let mut control_point_write = |_, data: &[u8]| {
            let op_copde = ControlOpCode::from(data[0]);
            info!("Control Point Received: {:?}", op_copde);

            match op_copde {
                ControlOpCode::TareScale => {}
                ControlOpCode::StartMeasurement => {
                    critical_section::with(|cs| {
                        *WEIGTH_TASK_ENABLED.borrow_ref_mut(cs) = true;
                    });
                }
                ControlOpCode::StopMeasurement => {
                    critical_section::with(|cs| {
                        *WEIGTH_TASK_ENABLED.borrow_ref_mut(cs) = false;
                    });
                }
                ControlOpCode::GetAppVersion => {
                    let response =
                        ResponseCode::AppVersion(env!("DEVICE_VERSION_NUMBER").as_bytes());
                    debug!("AppVersion: {:?}", response);
                    let data_point = DataPoint::new(response);
                    if channel.try_send(data_point).is_err() {
                        error!("Failed to send data point");
                    }
                    debug!("Sent GetAppVersion");
                }
                ControlOpCode::Shutdown => {}
                ControlOpCode::SampleBattery => {}
                ControlOpCode::GetProgressorId => {
                    let response = ResponseCode::ProgressorId(env!("DEVICE_ID").parse().unwrap());
                    debug!("ProgressorId: {:?}", response);
                    let data_point = DataPoint::new(response);
                    if channel.try_send(data_point).is_err() {
                        error!("Failed to send data point");
                    }
                    debug!("Sent GetAppVersion");
                }
            }
        };

        // TODO: Are this required, they are not used
        let mut service_change_read = |_, _data: &mut [u8]| 0;
        let mut data_point_read = |_, _data: &mut [u8]| 0;

        // TODO: Avoid using the gatt! macro, replace the uuids with constants and improve the values
        let device_name = env!("DEVICE_NAME").as_bytes();
        let appearance = b"[0] Unknown";
        let ppcp = b"Connection Interval: 50.00ms - 65.00ms, Max Latency:6ms, Suppervision Timeout Multiplier: 400ms";
        let car = b"Address resolution supported";
        let ccc = b"Notifications and indications disabled";
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
                        value: ppcp,
                    },
                    // Central Address Resolution
                    characteristic {
                        uuid: "2aa6",
                        value: car,
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
                                value: ccc,
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
                        // TODO: Is there a WriteNoResponse?
                    },
                ],
            },
        ]);

        let mut rng = bleps::no_rng::NoRng;
        let mut server = AttributeServer::new(&mut ble, &mut gatt_attributes, &mut rng);

        let mut notifier = || async {
            let data_point = channel.receive().await;
            debug!("Notifying data point: {:?}", data_point);
            let data = bytes_of(&data_point);
            NotificationData::new(data_point_handle, data)
        };
        server.run(&mut notifier).await.unwrap();
    }
}

#[embassy_executor::task]
async fn measurement_task(
    channel: &'static DataPointChannel,
    sck: Output<'static>,
    dt: Input<'static>,
    delay: Delay,
) {
    let mut load_sensor = hx711::HX711::new(sck, dt, delay);
    const SAMPLES: usize = 16;
    const CALIBRATION: f32 = 1.26;
    load_sensor.tare(SAMPLES);
    load_sensor.set_scale(CALIBRATION);

    loop {
        let enabled = critical_section::with(|cs| *WEIGTH_TASK_ENABLED.borrow_ref(cs));
        if enabled && load_sensor.is_ready() {
            let mut weigth: f32 = 0.0;
            for _ in 0..20 {
                let reading = load_sensor.read_scaled();
                if let Ok(x) = reading {
                    weigth += x;
                }
            }
            weigth /= 20000.0;
            let timestamp = (time::now().duration_since_epoch()).to_micros() as u32;
            let measurement = ResponseCode::WeigthtMeasurement(weigth, timestamp);
            debug!("Sending measurement: {:?}", measurement);
            let data_point = DataPoint::new(measurement);
            channel.send(data_point).await;
        }
        // On average, 20 measurements take 300 microseconds (0.3ms)
        // Tindeq can sample at 80Hz (12.5ms)
        // So, we can sleep for 10ms
        Timer::after(Duration::from_millis(10)).await;
    }
}
