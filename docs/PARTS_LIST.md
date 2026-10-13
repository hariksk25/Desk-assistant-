# Desk Plane Radar v6 - parts to buy and files to print

## Print (Bambu A1 Mini, PLA, 0.2 mm, 4 walls, no supports)

| File | Qty | Colour | Orientation |
|---|---|---|---|
| v6_carrier_white.stl | 1 | white or natural PLA (it is the ring diffuser) | front face down |
| v6_cover_black.stl | 1 | black (or any) | front face down |
| v6_bezel_ring_black.stl | 2 | red PLA (accent, matches the knobs) | flat, pegs up |
| v6_body.stl | 1 | same as the cover | open side down |
| v6_knob_red.stl | 2 | red PLA, 0.12 mm layers | top face down, no supports |
| v6_tft_retainer.stl | 2 | any colour (hidden) | flat |

Print the white carrier first and test-fit the two round displays, both OLEDs and one LED ring before printing the rest.

## Electronics

| # | Part | Spec that matters | Qty | Approx Rs | Where |
|---|---|---|---|---|---|
| 1 | ESP32-S3-DevKitC-1 N16R8 | WROOM-1-N16R8 (16 MB flash, 8 MB PSRAM), 44 pins, USB-C | 1 | 850 | the AYWHP listing you found, or robu.in |
| 2 | 1.28" GC9A01 round TFT, 7-pin round module | same board as the one you have (round PCB, two holes on the header tab) | 1 | 500 | Amazon |
| 3 | 0.96" SSD1306 OLED 128x64 I2C | second one; must be settable to address 0x3D (jumper or resistor on the back) | 1 | 330 | Amazon / robu.in |
| 4 | WS2812B LED ring, 24 LEDs, 66 mm OD / 52 mm ID | one per dial | 2 | 600 | robu.in / Amazon |
| 5 | KY-040 rotary encoder module (second one) | roof, above the left dial | 1 | 60 | robu.in / Amazon |
| 7 | 220 ohm resistor | one in series with each ring data line | 2 | 5 | ordered |
| 8 | 1000 uF 6.3 V electrolytic capacitor | across 5 V and GND at each ring | 2 | 20 | ordered |
| 9 | Dupont F-F 10 cm and 20 cm, a few F-M | | 1 set | 150 | Amazon |
| 10 | USB-C charger 5 V 2 A and a cable | plugs straight into the S3 through the facet in the left end | 1 | 0 | have |
| | Reused from the current build | second GC9A01, first OLED, KY-040 encoder, passive piezo | | | |
| | Total new | | | ~2,700 | |

## Hardware (M2)

| Item | Qty | Use |
|---|---|---|
| M2 x 4 mm brass heat-set inserts, 3.5 mm OD (robu.in R182371, 10 pcs, buy 2 packs) | 12 | 4 body bosses, 4 cover inserts in the carrier, 4 display bosses |
| M2 x 8 socket head | 4 | carrier to body, hidden under the cover |
| M2 x 6 wafer head | 20 | 4 cover, 4 round displays, 8 OLEDs, 4 display retainer bars (self-tapping into 1.7 mm holes) |
| Rubber bumpers, 8 mm, adhesive | 2 | under the pods on the sloped bottom |

## Pin map (ESP32-S3)

| Function | GPIO |
|---|---|
| TFT SCK / MOSI | 12 / 11 |
| TFT #1 CS / TFT #2 CS | 10 / 16 |
| TFT DC / RST (shared) | 13 / 14 |
| I2C SDA / SCL (OLED 0x3C, OLED 0x3D) | 8 / 9 |
| Encoder 1 (right) CLK / DT / SW | 4 / 5 / 6 |
| Encoder 2 (left) CLK / DT / SW | 15 / 17 / 18 |
| LED ring 1 data | 47 via 220 ohm |
| LED ring 2 data | 21 via 220 ohm |
| Piezo | 7 |
| 5 V rail | S3 5V pin: rings (1000 uF at each ring), piezo |
| 3V3 | both TFTs, both OLEDs, both encoders |

Do not use GPIO 0, 3, 45, 46 (strapping), 19, 20 (USB), 26 to 37 (flash and PSRAM).
