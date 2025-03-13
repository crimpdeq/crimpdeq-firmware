# Making your own Crimpdeq

1. Required Materials
    - [ESP32-C3-DevKit-RUST-1](https://github.com/esp-rs/esp-rust-board?tab=readme-ov-file#where-to-buy)
        - Other ESP32 devices can be used, but you need to figure out how to charge the battery
    - [Battery Holder](https://es.aliexpress.com/item/1005006283753220.html?spm=a2g0o.order_list.order_list_main.5.4779194d1mFZpd&gatewayAdapt=glo2esp)
    - [18650 Battery](https://es.aliexpress.com/item/1005007923191656.html?spm=a2g0o.order_list.order_list_main.11.4779194d1mFZpd&gatewayAdapt=glo2esp)
      - Other batteries migth also work, as long as they can power the device
    - [Crane Scale](https://es.aliexpress.com/item/1005002719645426.html?spm=a2g0o.order_list.order_list_main.17.4779194d1mFZpd&gatewayAdapt=glo2esp) or [Amazon alternative](https://www.amazon.es/dp/B08133JCM6)
      - Other crane-scales migth also work
    - [HX711](https://www.amazon.es/dp/B0DJX8BPQL)
2. Disassemble the Crane Scale
    <!-- Add photo -->
    1. Desolder the battery connections.
    2. esolder the four wires of the load cell (`E-`, `S-`, `S+` and `E+`) from the PCB
    3. Unscrew and remove the PCB along with the display.
4. Soldering Instructions
    1. Modify the HX711 Module
        <!-- Inser photo with the pinout -->
       1. Break the track of `RATE` pin,
       2. Verify with a polymeter that GND and the RATE pin are not connected anymore
            - Make sure that you dont break the next connection
       3. Solder the `RATE` to `VDD` pin
       4. Verify with a polymeter
        <!-- Add photo -->
        <!-- Add note that this can be skipped, but the sample rate will be 10Hz -->
    2. Connect the Crane Scale to the HX711:
      - Solder the 4 wires of the crane scale to the HX711. Usually the colors are:

        | **HX711 Pin** | **Load Cell Pin** | **Description**                    |
        | ------------- | ----------------- | ---------------------------------- |
        | E+            | E+ (Red)          | Excitation positive (to load cell) |
        | E-            | E- (Black)        | Excitation negative (to load cell) |
        | S+            | S+ (Green)        | Signal positive (from load cell)   |
        | S-            | S- (White)        | Signal negative (from load cell)   |

        - Note that sometimes the `S` pins are refred as `A`
    3. Connect the HX711 to the ESP32-C3-DevKit-RUST-1 devkit:

     | **HX711 Pin** | **ESP32-C3 Pin** | **Description**                |
     | ------------- | ---------------- | ------------------------------ |
     | VCC           | 3.3V             | Power supply (3.3V)            |
     | GND           | GND              | Ground                         |
     | DT (Data)     | GPIO4            | Data output from HX711         |
     | SCK (Clock)   | GPIO5            | Clock signal for communication |

    <!-- Add a note saying that they should verify all the connections with polymeter -->
5. Adapt the Scale Case:
   1. Create space for the USB connector
       - I did this by placing the devkit, marking the space that i needed with a pen and then, heating a knife and melting the case.
   2. Install the battery holder:
      1. Glue, with some silicone, the battery holder, make sure to leave the lid for the original batteries  of the scale open, as there is a hole for which you need to intrudece the two wire of the battery holder
      2. Solder those wires to the `B+` and `B-` pins of the ESP32-C3-DevKit-RUST-1
   3. Close the case
6. Upload the fimrware:
   - You may need to recalibrate

