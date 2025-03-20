# Crimpdeq

Crimpdeq is a bluetooth dynamometer designed for finger training, powered by an [ESP32-C3](https://github.com/esp-rs/esp-rust-board) and a WH-C100 crane scale, with firmware fully written in Rust!

> [!NOTE]
> If you're interested in reproducing this project or giving it a try, please, reach out! You can contact me via email (sergio.gazquez@gmail.com), [Twitter](https://x.com/Sergio_Gasquez) or [Bluesky](https://bsky.app/profile/sergiogasquez.bsky.social).

## Features
- Open-source frimware wirtten in Rust
- Open-source PCB design
- USB-C rechargeable battery
- Compatible with Tindeq Progressor app ([Android](https://play.google.com/store/apps/details?id=com.progressor&hl=es_419) [iOs](https://apps.apple.com/es/app/tindeq-progressor/id1380412428))
- Compatible with ClimbHarder app ([Android](https://play.google.com/store/apps/details?id=com.holdtight.climbharder&pcampaignid=web_share) [iOs](https://apps.apple.com/us/app/climbharder-no-hang-training/id6730120024))
- Sampling Frequency: 80 Hz
- Design Load: 1500 N (150 kg) (Full Scale)
- Precision:
    - *0.05 kg* between 0 and 99 kg
    - *0.1 kg* between 0 100 kg
- Working temperature: 0ºC - 40ºC
- Dimession: 80 mm x 90 mm x 35 mm
- Uses the [Tindeq Progressor API](https://tindeq.com/progressor_api/)

## [Book](https://sergiogasquez.github.io/crimpdeq/)
For detailed guidance on assembly, calibration, and charging of Crimpdeq, refer to the [Crimpdeq book](https://sergiogasquez.github.io/crimpdeq/).

The book covers everything you need to know, from building your own Crimpdeq to firmware installation and PCB details. Below is the list of available sections:

- [Introduction](https://sergiogasquez.github.io/crimpdeq/introduction.html)
- [Making your own Crimpdeq](https://sergiogasquez.github.io/crimpdeq/assembly.html)
- [Calibration](https://sergiogasquez.github.io/crimpdeq/calibration.html)
- [Charging the Battery](https://sergiogasquez.github.io/crimpdeq/battery.html)
- [Firmware](https://sergiogasquez.github.io/crimpdeq/firmware.html)
- [PCB](https://sergiogasquez.github.io/crimpdeq/pcb.html)

## Prototype

Here is how the current prototype looks like:

![Prototype](assets/prototype.png)

## Issues
If you encounter any issue or want to leave any feedback, please [open an issue](https://github.com/SergioGasquez/crimpdeq/issues/new)

## Kudos
- @bjoernQ for helping me during the development and developing the [`bleps`](https://github.com/bjoernQ/bleps) crate, which is fundamental for this project.
- [hangman](https://github.com/kesyog/hangman) for being an improved version of this project and a great source of inspiration for this project.
- Tindeq for having its API public and allowing for projects like this to exist!
