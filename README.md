# Crimpdeq

Crimpdeq is a bluetooth dynamometer designed for finger training, powered by an [ESP32-C3](https://github.com/esp-rs/esp-rust-board) and a [WH-C100 crane scale](https://www.amazon.es/dp/B08133JCM6), with firmware written in Rust!

> [!NOTE]
> If you're interested in reproducing this project or giving it a try, please, reach out! You can contact me via email (sergio.gazquez@gmail.com), [Twitter](https://x.com/Sergio_Gasquez) or [Bluesky](https://bsky.app/profile/sergiogasquez.bsky.social).

## Features
- Uses the [Tindeq Progressor API](https://tindeq.com/progressor_api/)
- Compatible with [Tindeq Progressor app](https://play.google.com/store/apps/details?id=com.progressor&hl=es_419)
- Compatible with [Climb Harder app](https://play.google.com/store/apps/details?id=com.holdtight.climbharder&pcampaignid=web_share)
-Capable of sampling at up to 80 Hz
## Status
- The firmware is in a usable state, though there is room for improvement. See issues for details.
- The PCB has not yet been produced nor tested.
  - I am not a hardware engineer, so any help or feedback would be greatly appreciated!

Here is how the current prototype looks like:

![Prototype](assets/prototype.png)

## Kudos
- @bjoernQ for helping me during the development and developing the [`bleps`](https://github.com/bjoernQ/bleps) crate, which is fundamental for this project.
- [hangman](https://github.com/kesyog/hangman) for being an improved version of this project and a great source of inspiration for this project.
- Tindeq for having its API public and allowing for projects like this to exist!
