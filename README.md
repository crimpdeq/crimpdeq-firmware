# Crimpdeq

[![Rust CI](https://github.com/crimpdeq/crimpdeq-firmware/actions/workflows/rust_ci.yml/badge.svg)](https://github.com/crimpdeq/crimpdeq-firmware/actions/workflows/rust_ci.yml)
[![Documentation](https://img.shields.io/badge/Documentation-Book-orange.svg)](https://crimpdeq.github.io/book/)

Crimpdeq is a bluetooth dynamometer designed for finger training, powered by an [ESP32-C3](https://github.com/esp-rs/esp-rust-board) and a WH-C100 crane scale, with firmware fully written in Rust!

> [!NOTE]
> If you're interested in reproducing this project or giving it a try, please, reach out! You can contact me via email (sergio.gasquez@gmail.com), [Twitter](https://x.com/Sergio_Gasquez) or [Bluesky](https://bsky.app/profile/sergiogasquez.bsky.social).

## Features
- Open-source firmware written in Rust
- Open-source PCB design
- USB-C rechargeable battery
- Compatible with Tindeq Progressor app ([Android](https://play.google.com/store/apps/details?id=com.progressor&hl=es_419) [iOs](https://apps.apple.com/es/app/tindeq-progressor/id1380412428))
- Compatible with ClimbHarder app ([Android](https://play.google.com/store/apps/details?id=com.holdtight.climbharder&pcampaignid=web_share) [iOs](https://apps.apple.com/us/app/climbharder-no-hang-training/id6730120024))
- Sampling Frequency: 80 Hz
- Design Load: 1500 N (150 kg) (Full Scale)
- Precision:
    - *0.05 kg* between 0 and 99 kg
    - *0.1 kg* between 100 and 150 kg
- Working temperature: 0ºC - 40ºC
- Dimension: 80 mm x 90 mm x 35 mm
- Uses the [Tindeq Progressor API](https://tindeq.com/progressor_api/)

## [Book](https://crimpdeq.github.io/book/)
For detailed guidance on assembly, calibration, and charging of Crimpdeq, refer to the [Crimpdeq book](https://crimpdeq.github.io/book/).

The book covers everything you need to know, from building your own Crimpdeq to firmware installation and PCB details. Below is the list of available sections:

- [Introduction](https://crimpdeq.github.io/book/introduction.html)
- [Making your own Crimpdeq](https://crimpdeq.github.io/book/assembly.html)
- [Calibration](https://crimpdeq.github.io/book/calibration.html)
- [Charging the Battery](https://crimpdeq.github.io/book/battery.html)
- [Firmware](https://crimpdeq.github.io/book/firmware.html)
- [PCB](https://crimpdeq.github.io/book/pcb.html)

## Prototype

Here is how the current prototype looks like:

![Prototype](assets/prototype.png)

## Contributing
Contributions are welcome! Feel free to:
- Submit PRs for bug fixes or new features
- Test and report issues
- Suggest improvements to documentation

## Issues
If you encounter any issue or want to leave any feedback, please [open an issue](https://github.com/SergioGasquez/crimpdeq/issues/new)

## License
This repository is licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Kudos
- @bjoernQ for helping me during the development and developing the [`bleps`](https://github.com/bjoernQ/bleps) crate, which was fundamental for this project.
- [hangman](https://github.com/kesyog/hangman) for being an improved version of this project and a great source of inspiration for this project.
- Tindeq for having its API public and allowing for projects like this to exist!
