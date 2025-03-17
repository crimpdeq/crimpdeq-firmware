# Crimpdeq

Crimpdeq is a bluetooth dynamometer designed for finger training, powered by an [ESP32-C3](https://github.com/esp-rs/esp-rust-board) and a [WH-C100 crane scale](https://www.amazon.es/dp/B08133JCM6), with firmware written in Rust!

For detailed guidance on assembly, calibration, and charging of Crimpdeq, refer to the [Crimpdeq book](https://sergiogasquez.github.io/crimpdeq/)

> [!NOTE]
> If you're interested in reproducing this project or giving it a try, please, reach out! You can contact me via email (sergio.gazquez@gmail.com), [Twitter](https://x.com/Sergio_Gasquez) or [Bluesky](https://bsky.app/profile/sergiogasquez.bsky.social).

## Features
- Uses the [Tindeq Progressor API](https://tindeq.com/progressor_api/)
- Compatible with [Tindeq Progressor app](https://play.google.com/store/apps/details?id=com.progressor&hl=es_419)
- Compatible with [Climb Harder app](https://play.google.com/store/apps/details?id=com.holdtight.climbharder&pcampaignid=web_share)
- Sampling Frequency: 80 Hz
- Design Load: 1500 N (150 kg) (Full Scale)
- Precision:
    - *0.05 kg* between 0 and 99 kg
    - *0.1 kg* between 0 100 kg
- Working temperature: 0ºC - 40ºC
- Dimession: 80 mm x 90 mm x 35 mm
- USB-C rechargeable battery

Here is how the current prototype looks like:

![Prototype](assets/prototype.png)

## Kudos
- @bjoernQ for helping me during the development and developing the [`bleps`](https://github.com/bjoernQ/bleps) crate, which is fundamental for this project.
- [hangman](https://github.com/kesyog/hangman) for being an improved version of this project and a great source of inspiration for this project.
- Tindeq for having its API public and allowing for projects like this to exist!
