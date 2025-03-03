#![no_std]
#![no_main]

use core::cell::RefCell;

use bleps::{
    ad_structure::{
        create_advertising_data,
        AdStructure,
        BR_EDR_NOT_SUPPORTED,
        LE_GENERAL_DISCOVERABLE,
    },
    async_attribute_server::AttributeServer,
    asynch::Ble,
    attribute_server::NotificationData,
    gatt,
    no_rng::NoRng,
};
use bytemuck::bytes_of;
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
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    rng::Rng,
    time,
    timer::{systimer::SystemTimer, timg::TimerGroup},
    Config,
};
use esp_println as _;
use esp_wifi::{ble::controller::BleConnector, init, EspWifiController};

use crate::{
    hx711::Hx711,
    progressor::{ControlOpCode, DataPoint, DataPointChannel, ResponseCode, SCAN_RESPONSE_DATA},
};

pub mod hx711;
pub mod progressor;

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
    ///
    /// Used in ClimbHarder App
    Tare,
    /// Soft taring the scale (substract the current weight)
    ///
    /// Used in Tindeq App
    SoftTare,
}

/// Static tracking the state of the measurement task
static MEASUREMENT_TASK_STATUS: Mutex<RefCell<MeasurementTaskStatus>> =
    Mutex::new(RefCell::new(MeasurementTaskStatus::Disabled));
/// Static tracking if the device was tared/soft tared
static DEVICE_TARED: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    let config = Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Allocate 72KB of heap memory
    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let esp_wifi_ctrl = &*mk_static!(
        EspWifiController<'static>,
        init(
            timg0.timer0,
            Rng::new(peripherals.RNG),
            peripherals.RADIO_CLK,
        )
        .unwrap()
    );

    // Load cell pins
    let clock_pin = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());
    let data_pin = Input::new(
        peripherals.GPIO4,
        InputConfig::default().with_pull(Pull::None),
    );
    let delay = Delay::new();

    // Initialize embassy
    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);

    // BLE Connector
    let connector = BleConnector::new(esp_wifi_ctrl, peripherals.BT);

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
    let now = || time::Instant::now().duration_since_epoch().as_millis();
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

        // Mark the device as untared
        critical_section::with(|cs| {
            *DEVICE_TARED.borrow_ref_mut(cs) = false;
        });

        // Service/Characteristics Read/Write methods
        let mut control_point_write = |_, data: &[u8]| {
            let op_copde = ControlOpCode::from(data[0]);
            info!("Control Point Received: {:?}", op_copde);

            match op_copde {
                ControlOpCode::TareScale => {
                    critical_section::with(|cs| {
                        *MEASUREMENT_TASK_STATUS.borrow_ref_mut(cs) = MeasurementTaskStatus::Tare;
                    });
                }
                ControlOpCode::StartMeasurement => {
                    let device_tared = critical_section::with(|cs| *DEVICE_TARED.borrow_ref(cs));
                    if device_tared {
                        critical_section::with(|cs| {
                            *MEASUREMENT_TASK_STATUS.borrow_ref_mut(cs) =
                                MeasurementTaskStatus::Enabled;
                        });
                    } else {
                        critical_section::with(|cs| {
                            *(MEASUREMENT_TASK_STATUS).borrow_ref_mut(cs) =
                                MeasurementTaskStatus::SoftTare;
                        });
                    }
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

        let device_name = env!("DEVICE_NAME").as_bytes();
        let appearance = b"[0] Unknown";
        let ppcp = b"Connection Interval: 50.00ms - 65.00ms, Max Latency:6ms, Suppervision Timeout Multiplier: 400ms";
        let car = b"Address resolution supported";
        let ccc = b"Notifications and indications disabled";
        let empty_value = b"";
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
                        value: empty_value,
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
                        value: empty_value,
                    },
                    /// Progressor Control Point
                    characteristic {
                        name: "control_point",
                        uuid: "7e4e1703-1ea6-40c9-9dcc-13d34ffead57",
                        write: control_point_write,
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
    let start_time = (time::Instant::now().duration_since_epoch()).as_micros() as u32;

    loop {
        let status = critical_section::with(|cs| *MEASUREMENT_TASK_STATUS.borrow_ref(cs));
        match status {
            MeasurementTaskStatus::Disabled => {
                Timer::after(Duration::from_millis(10)).await;
                continue;
            }
            MeasurementTaskStatus::Tare | MeasurementTaskStatus::SoftTare => {
                load_cell.tare().await;
                critical_section::with(|cs| {
                    *DEVICE_TARED.borrow_ref_mut(cs) = true;
                    if status == MeasurementTaskStatus::SoftTare {
                        *MEASUREMENT_TASK_STATUS.borrow_ref_mut(cs) =
                            MeasurementTaskStatus::Enabled;
                    }
                });
                Timer::after(Duration::from_millis(10)).await;
                continue;
            }
            MeasurementTaskStatus::Enabled => {}
        }

        let weight = load_cell.read_calibrated().await;
        let timestamp =
            (time::Instant::now().duration_since_epoch()).as_micros() as u32 - start_time;
        let measurement = ResponseCode::WeigthtMeasurement(weight, timestamp);
        debug!("Sending measurement: {:?}", measurement);
        let data_point = DataPoint::new(measurement);
        channel.send(data_point).await;
    }
}
