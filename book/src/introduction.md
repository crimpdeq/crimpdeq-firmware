# Crimpdeq

Meet Crimpdeq, an open source alternative to [Tindeq Progressor](https://tindeq.com/product/progressor/), that works with [Tindeq](https://tindeq.com/) and [ClimbHarder](https://climbharder.net/) apps!

The project relies on an ESP32-C3 and uses a HX711 to get the measurements from the crane scale. For more details, see the [Firmware](./firmware.md) chapter

PCB is finished but has not been manufactured yet, hence, not its not tested.

## Specs

- Sampling Frequency: 80 Hz
- Design Load: 1500 N (150 kg) (Full Scale)
- Precision:
    - *0.05 kg* between 0 and 99 kg
    - *0.1 kg* between 0 100 kg
- Working temperature: 0ºC - 40ºC
- Dimension: 80 mm x 90 mm x 35 mm
- USB-C rechargeable battery

> ⚠️ **Note**:  Some of these specs come from the crane scale, if you use a different one, those values might change.
