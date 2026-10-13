# MakerWorld listing (draft)

Copy the sections below into the MakerWorld upload form. Replace the GitHub link once the repo is public.

## Title

Desk Plane Radar: twin-dial ESP32-S3 flight radar, clock and desk assistant

## Short description

A desk gadget that shows the live air traffic around your home airport on a round display, with the aircraft's airline logo, route and approach on a second dial, LED rings, two knobs and a web page. Also a clock with 17 faces, weather, calendar, Pomodoro timer and more. Printed on an A1 Mini.

## Description

**What it is.** Two 1.28" round displays, two small OLEDs, two 24-LED rings and two knobs in a printed case, driven by an ESP32-S3. The left dial is a radar of the ADS-B traffic around the airport you choose (data from adsb.lol and friends, no receiver needed); the right dial shows the aircraft the radar is following: airline logo, flight number, route, aircraft type, and a side view of its approach to the runway. The rings sweep with the radar and mark each aircraft's bearing.

**What else it does.** Clock with 17 faces (the rings become the hands); weather, air quality and an indoor sensor; your calendar with chimes before meetings and an on-air light when your mic is live (via a small Mac helper); a Pomodoro timer with a focus log to Google Sheets; markets, news, media keys. Everything is set from the device (press both knobs for SETTINGS) or from the web page it serves.

**Printing.** Seven parts, no supports needed except where noted in the profile. Body and cover in black, carrier in white, bezel rings in black, knobs in red (or any colour). Fits the A1 Mini bed; about 2 h 30 for the body.

**Electronics.** See the parts list and the two wiring diagrams in the repo. No resistors or capacitors; the rings run from the board's 5 V pin at a capped brightness. The front panel is soldered as one harness of 21 wires; the panel-only diagram gives the soldering order.

**Firmware.** Open source (MIT) on GitHub: <GITHUB LINK>. Built with arduino-cli through a Makefile, flashed over USB once, then over WiFi. Two small config files hold your WiFi and your place.

**Difficulty.** Intermediate: soldering 50-odd joints, one afternoon of assembly, no SMD.

## Parts (summary)

| Qty | Part |
|---|---|
| 1 | ESP32-S3 N16R8 dev board with microSD slot |
| 2 | GC9A01 1.28" round TFT, 7 pin |
| 2 | SSD1306 0.96" OLED 128 x 64 (yellow / blue) |
| 2 | WS2812B 24-LED ring |
| 2 | KY-040 rotary encoder |
| 1 | BMP280 module |
| 1 | passive piezo 12 mm |
| 1 | microSD card (any size) |
| 1 | USB-C cable (90 degree), 5 V 2 A supply |
| | hookup wire, heat-shrink or tape, M2 screws |

## Print settings

| Setting | Value |
|---|---|
| Printer | Bambu Lab A1 Mini |
| Layer height | 0.2 mm |
| Walls | 3 |
| Infill | 15 % gyroid |
| Supports | as in the profile |
| Material | PLA (PETG for the body if the desk gets sun) |

## Assembly in short

1. Print all parts; test-fit the displays in the carrier.
2. Solder the front panel as one harness (panel diagram, soldering order on it), tape every joint.
3. Wire the harness to the board (full diagram), add the BMP280 and the piezo.
4. Flash the firmware (README), check both dials and both OLEDs on the web page, then close the case.
5. Optional: put airline logos on the SD card, install the Mac helper, set up the focus log sheet.

## Tags

esp32, esp32-s3, flight radar, ads-b, aviation, desk gadget, clock, gc9a01, ws2812b, pomodoro, weather station, bambu a1 mini

## Licence suggestion

Model: CC BY-NC-SA 4.0 (remix with credit, non-commercial). Firmware: MIT (in the repo).
