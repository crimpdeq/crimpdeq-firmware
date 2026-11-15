# Calibration models

`crimpdeq-calibration` holds the calibration of the Crimpdeq load cell: the models, their fits,
the rules for editing calibration points, the flash record and the BLE payloads. It has no
hardware dependencies, so its tests run on the host:

```bash
cargo test-host
```

The firmware fits the calibration and converts every reading; the app only selects a model and
displays what the firmware reports.

## Models

The user selects the model explicitly; the firmware never picks one. Linear is the default and
the model of earlier firmware.

| Model     | Curve                                                    | Load levels needed |
| --------- | -------------------------------------------------------- | ------------------ |
| Linear    | `kg = offset + factor * (raw - reference) / 1000`        | 2                  |
| Quadratic | `kg = a*x² + b*x + c`                                    | 4, one in the middle half of the load span |
| Piecewise | `kg = c + m1*x + (m2 - m1)*max(0, x - breakpoint)`       | 6, three on each side of the breakpoint     |

Loads closer than 2 % of the load span count as one level, so repeated weights do not count twice.
The minimum for each model is one level more than its free parameters, so the residuals can show a
bad fit. The linear minimum, two points, is the bare minimum: add a third load to check it.

### Units and coordinates

- `raw`: ADS1220 code, or an average of codes, before the tare.
- The linear factor is in g per raw count, as in earlier firmware.
- Nonlinear models use `x = (raw - reference_raw) / scale`:
  - `reference_raw` is the average raw value of the zero-load points, the measured unloaded
    reference. Without zero-load points, it is the average raw value of all points.
  - `scale` is the raw span of the points, signed with the wiring polarity, so `x` grows with the
    load for either wiring and the points span at most 1. Centring and scaling keep the
    least-squares fit well conditioned.
  - Coefficients are in kg per unit of `x`; the breakpoint is in `x`.

### Zero constraint

With zero-load points, the curve passes through their average (`c = 0`), like the linear fit of
earlier firmware. Without them, the offset is fitted.

### Fitting

- Linear: the least-squares slope through the zero reference, or through the mean point, exactly
  as earlier firmware computed it.
- Quadratic: least squares of `[x, x²]`, or `[1, x, x²]` without a zero reference.
- Piecewise: for a breakpoint, the segments are a least-squares fit of `[x, hinge]` or
  `[1, x, hinge]`, with `hinge = max(0, x - breakpoint)`, so they stay continuous. The breakpoint
  is searched between the third-lowest and the third-highest load levels: 65 grid candidates,
  then 32 golden-section steps around the best one. The work is fixed, so a fit takes a bounded
  time on the device.

### Validation

A fit is rejected, with its reason, when:

- a point or coefficient is not finite;
- there are too few points or load levels, or no mid-range level for the quadratic model;
- the points do not determine the curve, such as equal raw values for different loads;
- the sensitivity, `dkg/draw` in g per count, at either end of the calibrated range is more than
  10 times above or below the default (0.109 g per count), for either polarity;
- the curve decreases at either end of the range, so it is not monotonic;
- the sensitivities at the two ends differ by more than 1.5 times;
- for the piecewise model, the segment slopes differ by less than 1 %, so the breakpoint is
  arbitrary, or the best breakpoint lies at the end of the search, with too few levels on a side.

A quadratic slope is linear in `x`, and past the range the curve keeps its end slopes, so checking
the slopes at both ends proves the curve increasing for every raw value.

### Extrapolation

Outside the calibrated range, the raw range of the points of the fit, the curve continues as a
straight line with the slope at the end of the range. A quadratic cannot turn back or grow faster
than calibrated; linear and piecewise curves are straight there already. The details report the
range so the app can flag extrapolated readings.

## Tare and zero

The calibration reference and the runtime tare are separate:

- **Tare** (`0x64`) records the current raw value. Readings are
  `f(raw - shift) - f(tare - shift)`: the load on top of the load at the tare. The curve stays at
  the raw values where it was calibrated, so taring under a preload keeps the breakpoint and
  slopes at the loads where they were measured. Evaluating the curve at `raw - tare` would move
  them by the preload.
- **Set zero** (`0x7B`) is for an unloaded load cell whose baseline drifted. It tares, and sets
  `shift` so the curve reads 0 kg at the current raw value: the curve moves with the drift. It is
  rejected when the reading is more than 1 kg from the calibrated zero, which is a load rather than
  drift; a rejected zero keeps the previous tare. The shift is not stored: it resets at boot and
  whenever the calibration changes.

A linear curve gives the same reading either way, so linear readings are exactly
`(raw - tare) * factor / 1000`, as in earlier firmware. The power-on tare is a plain tare.

## Editing points

- Adding or removing a point refits the model in use.
- Selecting a model fits it to the stored points. When it does not fit, nothing changes and the
  reason is reported: the model in use is always the model reported to the user.
- A point that leaves the fit short of data is stored, and the previous calibration stays in use
  until the points can be fit. The details report the stored points as pending, with the reason.
- A point that makes a fit of applied points invalid is discarded. Points that are already pending
  were never validated together, so a new point joins them.
- A removal is always kept. When the remaining points cannot be fit, the previous calibration stays
  in use and the points are pending.
- Reset restores the default linear factor and deletes the points.

## Flash record

The record of `persist` is versioned (version 3) and holds the model, coefficients, reference,
scale, calibrated range, pending reason and points, protected by a CRC-32. The `calib` partition's
sector holds a log of 16 records of 256 bytes: a write appends after the last written slot without
erasing, so an interrupted write keeps the previous record, and the reader takes the newest record
whose CRC matches. Only a full log erases the sector before writing; a power cut in that window
loses the calibration and the device starts with the default. Every record read is validated again,
so a corrupt or unsupported record, or one of a newer version, never applies a curve. The sample
rate stays in its own partition.

Earlier firmware stored only a linear factor, in a 16-byte version 2 record. It migrates to that
linear factor without points or calibrated range, which converts readings exactly as before. The
first new record goes to the second slot, so the old record stays until the log wraps.

## BLE

New opcodes on the control point, after the existing `0x78`:

| Opcode | Command                    | Payload     | Response                           |
| ------ | -------------------------- | ----------- | ---------------------------------- |
| `0x79` | Get calibration details    | none        | calibration details                |
| `0x7A` | Set calibration model      | model (`u8`) | calibration details               |
| `0x7B` | Set zero                   | none        | calibration details                |

Model codes: 0 linear, 1 quadratic, 2 piecewise.

The details are notifications with the new response code `0x0A`; the first payload byte is the
kind. Every payload fits the 20-byte notification of the minimum ATT MTU. Multi-byte values are
little-endian `f32`; NaN is unknown.

| Kind | Payload                                                                                     |
| ---- | ------------------------------------------------------------------------------------------- |
| 0    | Info: version (1), supported model mask, model, point count, flags (bit 0: fitted from points, so the range is known), pending error, last op code, last disposition, last error (`u8` each) |
| 1    | Curve: model (`u8`), four coefficients: linear `[factor, offset]`, quadratic `[a, b, c]`, piecewise `[c, m1, m2, breakpoint]` |
| 2    | Coordinates: reference raw, scale, x min, x max                                             |
| 3    | Fit: RMS and largest absolute residual of the points, loads at the ends of the range, in kg |
| 4    | Tare: absolute load at the tare in kg, zero shift in raw counts                             |
| 5    | Point: index (`u8`), raw, known kg, fitted kg                                                |
| 6    | End: point count (`u8`)                                                                       |

Dispositions: 0 applied, 1 stored but pending, 2 rejected. Error codes are those of
[`Error`](src/lib.rs). The last command is the last of `0x69`, `0x74`, `0x78`, `0x7A` and `0x7B`
since boot, so a client adding a point with `0x69` reads its outcome with `0x79`. A reading plus
the load at the tare is the absolute load to compare with the calibrated range.

The residuals are calibration-fit residuals: how well the curve follows the points it was fit to,
not the accuracy at other loads.

### Compatibility

- Measurement packets (`0x01`) are unchanged: every client receives readings converted by the
  model in use.
- Existing commands keep their replies. `0x72` sends the calibration factor (`0x05`) and the
  stored points (`0x06`); the factor is NaN for a nonlinear model, which no factor describes.
  `0x69` and `0x78` follow the editing rules above and send the factor and points when they store
  a change; `0x78` then answers `0x09` with whether the point was removed. `0x74` restores the
  default linear calibration. Points now survive a reboot.
- Firmware before these commands ignores `0x79`–`0x7B`, so an app that gets no details uses the
  linear-only commands and offers only the linear model.
- The details describe the device state; a client reads them after each connection instead of
  assuming its previous selection.
