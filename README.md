# Crimpdeq Firmware

[![Rust CI](https://github.com/crimpdeq/crimpdeq-firmware/actions/workflows/rust_ci.yml/badge.svg)](https://github.com/crimpdeq/crimpdeq-firmware/actions/workflows/rust_ci.yml)
[![Documentation](https://img.shields.io/badge/Documentation-Book-orange.svg)](https://book.crimpdeq.com/)


Meet [Crimpdeq](https://crimpdeq.com), a portable digital force sensor designed for climbers, coaches, and therapists to measure and train finger strength, pulling power, and endurance.

Crimpdeq is a fully open-source project: an ESP32-C3 [PCB](https://github.com/crimpdeq/crimpdeq-pcb) with an ADS1220 ratiometric load-cell front end and a MAX17048 fuel gauge, a WH-C100 crane scale, a 3D-printable [case](https://github.com/crimpdeq/crimpdeq-case), and firmware fully written in Rust!


> [!NOTE]
> If you're interested in reproducing this project or giving it a try, please, reach out! You can contact me via email (sergio.gasquez@gmail.com), [Twitter](https://x.com/Sergio_Gasquez) or [Bluesky](https://bsky.app/profile/sergiogasquez.bsky.social).

## Specs

These specs describe PCB V3 with its matching case and firmware.

- Rechargeable battery with USB‑C charging, also while switched off
- Battery level reporting with a MAX17048 fuel gauge
- RGB status LED for connection and low-battery status
- ADS1220 24-bit load-cell ADC
- Communicates via Bluetooth Low Energy (BLE)
- Open-source firmware written in Rust
- Open-source PCB design
- Open-source 3D case design
- Automatic sleep when inactive
- Compatible with Tindeq Progressor app ([Android](https://play.google.com/store/apps/details?id=com.progressor&hl=es_419) | [iOS](https://apps.apple.com/es/app/tindeq-progressor/id1380412428))
- Compatible with Frez app (formerly ClimbHarder) ([Android](https://play.google.com/store/apps/details?id=com.holdtight.climbharder&pcampaignid=web_share) | [iOS](https://apps.apple.com/us/app/climbharder-no-hang-training/id6730120024))
- Sampling frequency: 175 Hz by default, selectable from 20 to 1000 Hz with the Crimpdeq app and kept across reboots
- Design load: 1500 N (≈150 kg), full scale
- Precision:
    - 0.05 kg between 0 and 99 kg
    - 0.1 kg between 100 and 150 kg
- Operating temperature: 0–40 °C
- Dimensions: 97 × 77 × 31 mm
- Uses the [Tindeq Progressor API](https://tindeq.com/progressor_api/)

## Firmware Versions

Each PCB revision needs its own firmware; use the latest release for your revision:

| PCB revision | Firmware                                                                      |
| ------------ | ----------------------------------------------------------------------------- |
| V1           | [`v0.4.0`](https://github.com/crimpdeq/crimpdeq-firmware/releases/tag/v0.4.0) |
| V2           | [`v2.1.0`](https://github.com/crimpdeq/crimpdeq-firmware/releases/tag/v2.1.0) |
| V3           | [`v3.0.0`](https://github.com/crimpdeq/crimpdeq-firmware/releases/tag/v3.0.0) |

The `main` branch targets PCB V3.

## Building and Flashing

The [Crimpdeq Book](https://book.crimpdeq.com/) covers assembly, calibration, charging, and general usage. See [Flash with Web Tools](https://book.crimpdeq.com/firmware/web_tools) to flash a release binary and [Flash with Local Setup](https://book.crimpdeq.com/firmware/local) for prerequisites, flashing, logs, and troubleshooting.

With the stable Rust toolchain, the `riscv32imc-unknown-none-elf` target, and [`probe-rs`](https://probe.rs/) installed:

```bash
cargo build --release                  # Build only
cargo run --release                    # Build, flash, and open the log monitor
DEFMT_LOG=info cargo run --release     # Same, with logs enabled
```

The advertised name, the Progressor ID, and the reported app version come from `DEVICE_NAME`, `DEVICE_ID`, and `DEVICE_VERSION_NUMBER` in [`.cargo/config.toml`](.cargo/config.toml). The calibration factor and sampling rate are stored in the `calib` and `settings` partitions of [`partitions.csv`](partitions.csv), so erasing the flash also erases the calibration.

## Contributing
Contributions are welcome! Feel free to:
- Submit PRs for bug fixes or new features
- Test and report issues
- Suggest improvements to documentation

## Issues
If you encounter any issue or want to leave any feedback, please [open an issue](https://github.com/crimpdeq/crimpdeq-firmware/issues/new)

## License
This repository is licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Kudos
- @bjoernQ for helping me during the development and developing the [`bleps`](https://github.com/bjoernQ/bleps) crate, which the first versions of this firmware were built on.
- [`trouble`](https://github.com/embassy-rs/trouble), the BLE host stack used by this firmware.
- [hangman](https://github.com/kesyog/hangman) for being an improved version of this project and a great source of inspiration for this project.
- Tindeq for having its API public and allowing for projects like this to exist!
