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
    no_rng::NoRng,
};
use bytemuck::bytes_of;
use core::cell::RefCell;
use critical_section::Mutex;
use defmt::{debug, error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_sync::channel::Channel;
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
// use loadcell::{hx711, LoadCell};

use crimpdeq::{
    hx711::Hx711,
    progressor::{ControlOpCode, DataPoint, DataPointChannel, ResponseCode, SCAN_RESPONSE_DATA},
};

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

/// Status of the weigth measurement task
#[derive(Copy, Debug, Clone, PartialEq)]
enum MeasurementTaskStatus {
    /// Measurements are enabled
    Enabled,
    /// Measurements are disabled
    Disabled,
    /// Taring the scale
    Tare,
}

/// Static tracking the state of the measurement task
static MEASUREMENT_TASK_STATUS: Mutex<RefCell<MeasurementTaskStatus>> =
    Mutex::new(RefCell::new(MeasurementTaskStatus::Disabled));

// Calibration value. Obtained measuring a few known weights and adjusting the value
const CALIBRATION: f32 = 1.26;

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    let config = Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Allocate 72KB of heap memory
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

    // Load cell pins
    let clock_pin = Output::new(peripherals.GPIO5, Level::Low);
    let data_pin = Input::new(peripherals.GPIO4, Pull::None);
    let delay = Delay::new();

    // Initialize embassy
    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);

    // BLE Connector
    let connector = BleConnector::new(init, peripherals.BT);

    // Data point channel
    let channel = mk_static!(DataPointChannel, Channel::new());

    // Spawn tasks
    spawner.spawn(bt_task(connector, channel)).unwrap();
    spawner
        .spawn(measurement_task(channel, clock_pin, data_pin, delay))
        .unwrap();

    // Wait forever
    loop {
        Timer::after(Duration::from_millis(50)).await;
    }
}

#[embassy_executor::task]
async fn bt_task(connector: BleConnector<'static>, channel: &'static DataPointChannel) {
    let now = || time::now().duration_since_epoch().to_millis();
    let mut ble = Ble::new(connector, now);
    loop {
        // Reset the state of the measurement task
        critical_section::with(|cs| {
            *MEASUREMENT_TASK_STATUS.borrow_ref_mut(cs) = MeasurementTaskStatus::Disabled;
        });
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

        // Service/Characteristics Read/Write methods
        let mut control_point_write = |_, data: &[u8]| {
            let op_copde = ControlOpCode::from(data[0]);
            info!("Control Point Received: {:?}", op_copde);

            match op_copde {
                ControlOpCode::TareScale => {}
                ControlOpCode::StartMeasurement => {
                    critical_section::with(|cs| {
                        *MEASUREMENT_TASK_STATUS.borrow_ref_mut(cs) = MeasurementTaskStatus::Tare;
                    });
                }
                ControlOpCode::StopMeasurement => {
                    critical_section::with(|cs| {
                        *MEASUREMENT_TASK_STATUS.borrow_ref_mut(cs) =
                            MeasurementTaskStatus::Disabled;
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

        let mut rng = NoRng;
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
    clock_pin: Output<'static>,
    data_pin: Input<'static>,
    delay: Delay,
) {
    let mut load_cell = Hx711::new(data_pin, clock_pin, delay);
    load_cell.set_scale(CALIBRATION);

    loop {
        let status = critical_section::with(|cs| *MEASUREMENT_TASK_STATUS.borrow_ref(cs));
        if status == MeasurementTaskStatus::Disabled {
            Timer::after(Duration::from_millis(13)).await;
            continue;
        }

        let weigth = if status == MeasurementTaskStatus::Tare {
            load_cell.tare(16).await;
            critical_section::with(|cs| {
                *MEASUREMENT_TASK_STATUS.borrow_ref_mut(cs) = MeasurementTaskStatus::Enabled;
            });
            0.0
        } else {
            load_cell.get_measurement().await
        };

        let timestamp = (time::now().duration_since_epoch()).to_micros() as u32;
        let measurement = ResponseCode::WeigthtMeasurement(weigth, timestamp);
        debug!("Sending measurement: {:?}", measurement);
        let data_point = DataPoint::new(measurement);
        channel.send(data_point).await;
        // On average, measurements take 300 microseconds (0.3ms)
        // Tindeq can receive samples at 80Hz (12.5ms)
        // So, we can sleep for 10ms
        Timer::after(Duration::from_millis(10)).await;
    }
}
