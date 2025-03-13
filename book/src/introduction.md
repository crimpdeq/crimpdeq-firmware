# Crimpdeq

Meet Crimpdeq, an open source alternative to [Tindeq Progressor](https://tindeq.com/product/progressor/), that works with Tindeq and ClimbHarder apps!

The project relies on an ESP32-C3 and uses a HX711 to get the measurements from the crane scale. All the firmware its written in Rust, using `esp-hal` and the anciliary crates.

PCB is almost finished but has not been manufactured yet, note that there may be some differences between the hardware in the PCB and the prototype

## Specs

- Sampling Frequency: 80 Hz
- Design Load: 1500 N (150 kg) (Full Scale)
- Precision:
    - *0.05 kg* between 0 and 99 kg
    - *0.1 kg* between 0 100 kg
- Working temperature: 0ºC - 40ºC
- Dimession: 80 mm x 90 mm x 35 mm

<!-- Note that specs are based on the crane scale that I use -->
