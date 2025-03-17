# Calibration

1. Comment the `tare` calls from the code
2. Set the calibration factor to `1.0` and calibration offset to `0.0`
3. Measure how much it measures with no weight attached
4. Measure how much it measures with a known weight attached
   1. This known weight should be a value greater than the maximum weight that you are trying to apply. Using something like 80 kg should be enough
5. Calculate the new calibration offset and factor:
6. To calibrate a scale sensor using two known points, use the linear equation:
    ```
    m = (y_2 - y_1) / (x_2 - x_1)
    b = y_1 - m * x_1
    ```
    Where:
    - `y1` is the actual weight (kg) with no weigth attached (should be `0`)
    - `y2` is the actual weight (kg) of the known weight
    - `x1` is the sensor reading with no weigth attached
    - `x2` is the sensor reading with known weight
    - `m` is the calibration factor
    - `b` is the calibration offset
7. Update those values in the code:
   1. Open the `hx711.rs` file and update the `CALIBRATION_FACTOR` and `CALIBRATION_OFFSET` values
8. Rebuild and upload the code:
    ```bash
    cargo run --release
    ```
9. Verify that it now measures properly

For an example of calibration, see the following [Pull Request](https://github.com/SergioGasquez/crimpdeq/pull/11).
