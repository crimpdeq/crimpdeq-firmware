# Calibration
1. Get the hex value of a known weigth:
   - This known weight should be a value greater than the maximum weight that you are trying to apply. Using something like 80 kg should be enough.
   1. Go to [Floating Point to Hex Converter](https://gregstoll.com/~gregstoll/floattohex/)
   2. Use the *Single-precision* floating point converter
   3. Add your known weigth in the "Float value"
   4. Press "Convert to hex"
   5. Save the calculated "Hex value"
   - Eg: Using a known weigth of 75.3, gives a hex value of 0x4296999a
2. Download and install the nRF Connect app
   - [Android](https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp&hl=es_419)
   - [iOS](https://apps.apple.com/es/app/nrf-connect-for-mobile/id1054362403)
   - [Desktop](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-Desktop/Download#infotabs): Windows, Linux and macOS versions are available.
3. Connect your Crimpdeq with nRF Connect:
   1. Open the app
   2. Scan for devices
   3. It should be listed on the Scanner tab as "Progressor_7125"
   4. Click "Connect"
   5. Go to its tab

   ![nrF Discovered](./assets/Screenshot_1.png)
4. Once connected, you should see the different serives and characteristics. Cick the "Unkown Service" to expand its characteristics
   ![Services](./assets/Screenshot_2.png)
5. Hang your Crimpdeq with no weigth
6. Send to the `7e4e1703-1ea6-40c9-9dcc-13d34ffead57` characteristic a `7300000000` value:
   - You can send commands by pressing the Up Arrow icon on the characteristic and filling the fields as in the screenshot:
   ![Send weigth](./assets/Screenshot_3.png)
7. Hang the known weigth to your Crimpdeq
8. Append `73` to the hex value calculated in step 1: `73<your_hex_result>`
   - Eg: For 75.3 kg (0x4296999a) that would be: `734296999a`
9. Send that value to the  `7e4e1703-1ea6-40c9-9dcc-13d34ffead57` characteristic

