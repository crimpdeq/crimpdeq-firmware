# Crimpdeq Firmware

[![Rust CI](https://github.com/crimpdeq/crimpdeq-firmware/actions/workflows/rust_ci.yml/badge.svg)](https://github.com/crimpdeq/crimpdeq-firmware/actions/workflows/rust_ci.yml)
[![Documentation](https://img.shields.io/badge/Documentation-Book-orange.svg)](https://crimpdeq.github.io/book/)


Meet Crimpdeq, a portable digital force sensor designed for climbers, coaches, and therapists to measure and train finger strength, pulling power, and endurance.

Crimpdeq is a fully open-source project based on an [ESP32-C3](https://github.com/esp-rs/esp-rust-board) and a WH-C100 crane scale, with firmware fully written in Rust!


> [!NOTE]
> If you're interested in reproducing this project or giving it a try, please, reach out! You can contact me via email (sergio.gasquez@gmail.com), [Twitter](https://x.com/Sergio_Gasquez) or [Bluesky](https://bsky.app/profile/sergiogasquez.bsky.social).

## Specs

- Rechargeable battery with USB‑C charging
- Communicates via Bluetooth Low Energy (BLE)
- Open-source firmware written in Rust
- Open-source PCB design
- Automatic sleep when inactive
- Compatible with Tindeq Progressor app ([Android](https://play.google.com/store/apps/details?id=com.progressor&hl=es_419) | [iOS](https://apps.apple.com/es/app/tindeq-progressor/id1380412428))
- Compatible with Frez app (formerly ClimbHarder) ([Android](https://play.google.com/store/apps/details?id=com.holdtight.climbharder&pcampaignid=web_share) | [iOS](https://apps.apple.com/us/app/climbharder-no-hang-training/id6730120024))
- Sampling frequency: 80 Hz
- Design load: 1500 N (≈150 kg), full scale
- Precision:
    - 0.05 kg between 0 and 99 kg
    - 0.1 kg between 100 and 150 kg
- Operating temperature: 0–40 °C
- Dimensions: 80 × 90 × 35 mm
- Uses the [Tindeq Progressor API](https://tindeq.com/progressor_api/)

## Building and Running the Firmware

The [Crimpdeq Book](https://crimpdeq.github.io/book/) covers assembly, calibration, charging, and general usage. For repository-specific instructions, see the [Firmware](https://crimpdeq.github.io/book/firmware.html) chapter for prerequisites, how to build, flash, and run the firmware, how to enable logs, and troubleshooting.

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
- @bjoernQ for helping me during the development and developing the [`bleps`](https://github.com/bjoernQ/bleps) crate, which was fundamental for this project.
- [hangman](https://github.com/kesyog/hangman) for being an improved version of this project and a great source of inspiration for this project.
- Tindeq for having its API public and allowing for projects like this to exist!
