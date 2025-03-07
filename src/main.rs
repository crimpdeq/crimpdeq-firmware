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
use defmt::{debug, info};
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

// Helper macro for static allocation
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

/// Status of the weight measurement task
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

/// Device state management
struct DeviceState {
    measurement_status: MeasurementTaskStatus,
    tared: bool,
    start_time: u32,
}

/// Static tracking the state of the device
static DEVICE_STATE: Mutex<RefCell<DeviceState>> = Mutex::new(RefCell::new(DeviceState {
    measurement_status: MeasurementTaskStatus::Disabled,
    tared: false,
    start_time: 0,
}));

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    // System initialization
    let config = Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Allocate 72KB of heap memory
    esp_alloc::heap_allocator!(size: 72 * 1024);

    // Initialize BLE controller
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

    // Initialize load cell pins
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

    // Data point channel for communication between tasks
    let channel = mk_static!(DataPointChannel, Channel::new());

    // Spawn tasks
    spawner.spawn(bt_task(connector, channel)).unwrap();
    spawner
        .spawn(measurement_task(channel, clock_pin, data_pin, delay))
        .unwrap();

    // Idle loop
    loop {
        Timer::after(Duration::from_millis(50)).await;
    }
}

#[embassy_executor::task]
async fn bt_task(connector: BleConnector<'static>, channel: &'static DataPointChannel) {
    let now = || time::Instant::now().duration_since_epoch().as_millis();
    let mut ble = Ble::new(connector, now);
    loop {
        // Reset device state on reconnection
        critical_section::with(|cs| {
            let mut state = DEVICE_STATE.borrow_ref_mut(cs);
            state.measurement_status = MeasurementTaskStatus::Disabled;
            state.tared = false;
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
                ControlOpCode::TareScale => {
                    critical_section::with(|cs| {
                        DEVICE_STATE.borrow_ref_mut(cs).measurement_status =
                            MeasurementTaskStatus::Tare;
                    });
                }
                ControlOpCode::StartMeasurement => {
                    critical_section::with(|cs| {
                        let mut state = DEVICE_STATE.borrow_ref_mut(cs);
                        if state.tared {
                            state.measurement_status = MeasurementTaskStatus::Enabled;
                            state.start_time =
                                (time::Instant::now().duration_since_epoch()).as_micros() as u32;
                        } else {
                            state.measurement_status = MeasurementTaskStatus::SoftTare;
                        }
                    });
                }
                ControlOpCode::StopMeasurement => {
                    critical_section::with(|cs| {
                        DEVICE_STATE.borrow_ref_mut(cs).measurement_status =
                            MeasurementTaskStatus::Disabled;
                    });
                }
                ControlOpCode::GetAppVersion => {
                    let response =
                        ResponseCode::AppVersion(env!("DEVICE_VERSION_NUMBER").as_bytes());
                    debug!("AppVersion: {:?}", response);
                    let data_point = DataPoint::from(response);
                    data_point.send(channel);
                }
                ControlOpCode::GetProgressorId => {
                    let response = ResponseCode::ProgressorId(env!("DEVICE_ID").parse().unwrap());
                    debug!("ProgressorId: {:?}", response);
                    let data_point = DataPoint::from(response);
                    data_point.send(channel);
                }
                // Currently unimplemented operations
                ControlOpCode::Shutdown | ControlOpCode::SampleBattery => {}
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

    loop {
        // Get current device state
        let (status, start_time) = critical_section::with(|cs| {
            let state = DEVICE_STATE.borrow_ref(cs);
            (state.measurement_status, state.start_time)
        });

        match status {
            MeasurementTaskStatus::Disabled => {
                // Do nothing when disabled
                Timer::after(Duration::from_millis(10)).await;
            }
            MeasurementTaskStatus::Tare | MeasurementTaskStatus::SoftTare => {
                // Perform taring operation
                load_cell.tare().await;

                critical_section::with(|cs| {
                    let mut state = DEVICE_STATE.borrow_ref_mut(cs);
                    state.tared = true;

                    // For soft tare, immediately enable measurements
                    if status == MeasurementTaskStatus::SoftTare {
                        state.measurement_status = MeasurementTaskStatus::Enabled;
                    }
                });

                Timer::after(Duration::from_millis(10)).await;
            }
            MeasurementTaskStatus::Enabled => {
                // Perform weight measurement and send data
                let weight = load_cell.read_calibrated().await;
                let timestamp =
                    (time::Instant::now().duration_since_epoch()).as_micros() as u32 - start_time;
                let response = ResponseCode::WeigthtMeasurement(weight, timestamp);
                let data_point = DataPoint::from(response);
                data_point.send(channel);
            }
        }
    }
}
