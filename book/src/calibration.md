# Calibration

1. Using an IDE or text editor, open the project
2. Reset the calibration factor and calibration offset
   1. Open the `src/hx711.rs` file
   2. Set `CALIBRATION_FACTOR` value to `1.0`
   3. Set `CALIBRATION_OFFSET` value to `0.0`
3. Upload the code with the new values:
   1. Connect your device via USB-C
   2. In a terminal, run:
        ```bash
        cargo run --release
        ```
4. Connect your device to the Tindeq or ClimbHarder app and go to the Live Data/Live View
5. Measure how much it measures with no weight attached
6. Measure how much it measures with a known weight attached
   1. This known weight should be a value greater than the maximum weight that you are trying to apply. Using something like 80 kg should be enough
7. Calculate the new calibration offset and factor:
   1. To calibrate a scale sensor using two known points, use the linear equation:
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
8.  Update those values in the code:
    1. Open the `src/hx711.rs` file
    2. Update the `CALIBRATION_FACTOR` to the calculated `m` value
    3. Update the `CALIBRATION_OFFSET` to the calculated `b` value
    4. Upload the code with the new values:
        1. Connect your device via USB-C
        2. In a terminal, run:
            ```bash
            cargo run --release
            ```
9.  Verify that it now measures properly

For an example of calibration, see the following [Pull Request](https://github.com/SergioGasquez/crimpdeq/pull/11).
