# v6 firmware brief (for Claude Code) - 12 Oct 2026

The v6 hardware is **built and wired (phase 1)**. Your job: write the v6 firmware, bring every part up one at a time, then port the v5 apps to the twin-dial UI. This file is the spec. HANDOFF.md has history; CLAUDE.md has the rules. Wiring picture: `docs/images/wiring_v6_phase1.png` (source `tools/wiring_v6_phase1.py`).

"Left" and "right" always mean as seen from the FRONT. The left dial is on the USB side.

## 1. Hardware on the bench

| Part | Detail |
|---|---|
| MCU | ESP32-S3 N16R8 (16 MB flash, 8 MB octal PSRAM), "S3-CAM" style board: two USB-C ports, camera connector (unused, keep it empty), on-board microSD slot |
| Round displays x2 | GC9A01 1.28" 240 x 240, 7-pin (VCC GND SCL SDA RES DC CS), datasheet R123209. Mounted with the pin tab UP, both need 180 degree rotation |
| OLEDs x2 | SSD1306 0.96" 128 x 64 I2C, yellow/blue (top 16 rows yellow in the panel's native orientation). Mounted UPSIDE DOWN under each round display: rotate 180; the yellow band then sits at the BOTTOM of the window. Both 0x3C (the address resistor was never moved): left on I2C bus 0 (8 / 9), right alone on bus 1 (1 / 2, `Wire1`). Changed 12 Oct |
| OLED visible window | about 24.3 x 12.6 mm through the bezel. The logical TOP 3 to 4 rows (after rotation) sit at the window edge next to the round display: keep them free of essential content until checked on the device |
| LED rings x2 | 24 x WS2812B each (GRB), 85 / 72 mm. Pixel 0 position and direction unknown: make offset + direction settings and a calibration screen |
| BMP280 module | 4-pin (VIN GND SCL SDA), VIN on 3V3. Address 0x76 or 0x77 (scan both). Chip ID 0x58 = BMP280 (temp + pressure), 0x60 = BME280 (adds humidity): support both |
| Encoders x2 | Pins labelled S1A (= CLK / A), S1B (= DT / B), SW, GND, VCC on 3V3. Detent encoders, typically 2 or 4 counts per detent: measure and make it a constant |
| Piezo | Passive 12 mm buzzer on GPIO 7 through 100 ohm (LEDC tone) |
| microSD | 16 GB Class 10 inserted, FAT32. SD_MMC 1-bit on CMD 38 / CLK 39 / D0 40 (**confirm at bring-up**; fall back to SPI mode only if SD_MMC fails) |
| Power | USB-C 5 V 2 A. Rings on the 5V pin, everything else 3V3. Ring data has 330 ohm series resistors and bulk capacitance; **no level shifter**: if the first LED flickers, report it (owner fix: 1N4007 in the ring 5V feed, or 74AHCT125 later) |

## 2. Pin map (phase 1, final)

| GPIO | Function | Notes |
|---|---|---|
| 12 / 11 | SPI SCK / MOSI | shared by both GC9A01 |
| 38 / 14 | TFT DC / RST | shared; reset both at boot. DC was 13, moved 12 Oct: 13 is the microSD MISO |
| 16 / 41 | TFT CS left / right (`PANEL_SWAP` in config_local.h flips them) | left was 10: the left panel never answered on GPIO 10 on this board (new wire tried, board pulls GPIO 10 high). Moved 12 Oct |
| 8 / 9 | I2C bus 0 SDA / SCL | left OLED 0x3C (right with `PANEL_SWAP`), BMP280 0x76/0x77. 400 kHz |
| 1 / 2 | I2C bus 1 SDA / SCL (`Wire1`) | right OLED 0x3C alone (left with `PANEL_SWAP`). 400 kHz. Added 12 Oct |
| 4 / 5 / 6 | left encoder A / B / SW (right with `PANEL_SWAP`) | SW: INPUT_PULLUP |
| 15 / 17 / 18 | right encoder A / B / SW (left with `PANEL_SWAP`) | |
| 47 / 21 | LED ring left / right data | RMT |
| 7 | Piezo | phase 2: becomes I2S DIN for the amp |
| 10 / 11 / 12 / 13 | microSD CS / MOSI / SCK / MISO | on board, SPI mode (found by `firmware/sd_probe`, 12 Oct); shares SCK / MOSI with the displays |
| Reserved phase 2 | 39 / 40 (I2S BCLK / LRCLK), 7 (I2S DIN, replaces the piezo), 42 / 48 (LD2410 OUT or UART), I2C bus 0 for BH1750 / SHT30 | do not use now. Changed 12 Oct |
| Never | 0, 3, 45, 46 (strapping), 19 / 20 (native USB), 26 to 37 (flash + PSRAM), 43 / 44 (UART0 log) | camera pins overlap our map: never plug a camera |

## 3. Toolchain

| Item | Value |
|---|---|
| Sketch | `firmware/desk_plane_radar_v6/desk_plane_radar_v6.ino` (new; v5 stays untouched for the old device) |
| Board | `esp32:esp32:esp32s3`, core 3.3.x. Make targets `compile6 / upload6 / log6` (see Makefile; verify option keys with `arduino-cli board details -b esp32:esp32:esp32s3` and fix the FQBN if a key differs) |
| Options | Flash 16 MB, PSRAM OPI, partition `app3M_fat9M_16MB`, USB CDC on boot enabled, USB mode hardware CDC |
| Port | `/dev/cu.usbmodem*` on the native USB port, or `/dev/cu.wchusbserial*` on the UART port. If upload fails: hold BOOT, tap RST, release BOOT |
| Secrets | `make secrets6` copies `firmware/desk_plane_radar_v5/secrets.h` into the v6 folder (gitignored). Same hard rules as v5: never print, log or commit it |
| Libraries | as v5 plus Adafruit NeoPixel, Adafruit BMP280 + Adafruit BME280 (or a small register driver), ESP32Encoder (PCNT hardware counting). SD_MMC is in the core. `make deps` installs them |
| Version | start at `6.0.0`; bump the patch digit before every flash (boot screen proves the build) |

## 4. Order of work

Each step: compile, say what you are flashing, flash, capture `make log6`, confirm the acceptance line, then move on. Stop and report anything that fails twice.

| Step | Build | Accept when |
|---|---|---|
| 0 | `6.0.x` hardware test sketch (keep it as a "DIAG" app later): boot banner with heap / PSRAM / flash size | log shows PSRAM about 8 MB, flash 16 MB, reset reason |
| 1 | I2C scan | log lists 0x3C and 0x76 or 0x77 on bus 0, 0x3C on bus 1 |
| 2 | Both OLEDs, rotation 180, test text with a border box | left shows "LEFT", right "RIGHT", upright, yellow band at the bottom. Ask the owner whether the top rows are visible |
| 3 | Left round display, then right: colour bars, a circle at r 119, "LEFT" / "RIGHT", rotation 180 | both upright, no tearing, no interference between the two |
| 4 | Rings: chase one pixel around each ring slowly | owner reports where pixel 0 is and the direction; store as `RING_OFFSET[2]`, `RING_DIR[2]` |
| 5 | Encoders: counts and clicks in the log and on the OLEDs | clockwise = +1 per detent on both; clicks debounced |
| 6 | Piezo: short two-tone chime | audible, no stuck tone |
| 7 | BMP280 / BME280: read every 2 s | sane temperature and about 910 hPa (Bengaluru, about 900 m) |
| 8 | microSD: mount, card size, write + read `/log/boot.txt` | log shows the card size and the read-back matches |
| 9 | WiFi, NTP, mDNS `deskradar.local`, OTA, web log | `make weblog` works; OTA flash works |
| 10 | Port v5 (section 5) as `6.1.0` | every v5 app works on both dials |
| 11 | Rings + OLED pages per app, BMP280 on the weather and system pages | as the table in section 6 |
| 12 | SD phase 1: crash log, focus log backup, `/config.json` overrides (`docs/SD_CARD_PLAN.md`) | files appear on the card |
| 13 | Fix the v5 open issues that still apply (HANDOFF section 6: market % sign, NIFTY stuck, routeset 403) | verified against Google / live data |

## 5. Porting v5

| Topic | v5 | v6 |
|---|---|---|
| Rendering | one GC9A01, drawn in 3 horizontal bands with a 256-colour palette (RAM limit) | two full 240 x 240 RGB565 canvases in PSRAM (about 115 KB each). Draw the whole frame, then flush. Drop the band/palette workarounds (smooth gradients are fine now) but keep paint functions deterministic per frame |
| Two displays, one bus | n/a | both GC9A01 share SCK/MOSI/DC/RST with their own CS. Verify Arduino_GFX can run two panel objects on one SPI host; if not, use one bus object and drive CS manually. Initialise both after one shared reset pulse |
| OLED | one SSD1306 | two Adafruit_SSD1306 objects, both 0x3C, on `Wire` and `Wire1`, rotation 2 |
| Input | one KY-040 with tap / double tap / hold gestures | two encoders via ESP32Encoder (PCNT). Gesture code per knob |
| Memory | TLS needs a 40 KB contiguous block; heap was the main stability risk | put canvases, JSON documents, caches and logs in PSRAM (`ps_malloc`, ArduinoJson PSRAM allocator). Internal heap stays for WiFi/TLS. Log `largest free block` internal and PSRAM at boot and on the SYSTEM page |
| Fonts | big fonts digits only | can add full big fonts (letters) now; flash is 16 MB |
| Tasks | `fetchTask` on core 0 for network, UI in `loop()`, shared data under `dataMutex` | same split. SD access from one task only, never inside paint functions |
| Keep unchanged | Pomodoro / Focus Pro logic, focus log protocol and NVS keys, alerts, settings web page fields | only adapt drawing and input |

## 6. UI model (owner decision, 10 Oct)

No always-on radar. One app is active; **both dials show that app**: left dial = main view, right dial = detail of the same app. Each OLED supports the dial above it. Rings follow the app.

| App | Left dial | Right dial | OLED left / right | Rings |
|---|---|---|---|---|
| Radar | map with aircraft | selected aircraft: type, altitude, speed, route | callsign, distance / origin to destination | left: bearing of each aircraft; right: distance of the selected one |
| Arrivals | arrivals board | next arrival, ETA countdown | count next hour / runway | countdown |
| Approach | runway approach view | descent profile | callsign / altitude, speed | distance to touchdown |
| Replay | replay map | timeline (right knob scrubs) | time / count | progress |
| Clock | time | date, next alarm, second zone | day / alarm | seconds / hour |
| Weather | now | hourly forecast | humidity (BME only), wind / AQI; indoor temp + pressure from the BMP280 | AQI colour; blue when rain is near |
| Pomodoro | session countdown | daily goal, streak, week | task / sessions today | session progress / daily goal |
| Markets | SENSEX / NIFTY | intraday chart | change % / previous close | green or red with the move |
| News | headline | next headline | source / age | quiet |
| Cricket | score | batting / bowling or run rate | overs / RR | wickets or RR |
| F1 | next race countdown | standings | circuit / session | countdown |
| Stats, Network | today | 7-day chart | key values | quiet |
| System, Display, Location | settings list | live preview | hint / value | preview |

| Control | Action |
|---|---|
| Left knob turn | change app (both dials follow) |
| Right knob turn | scroll inside the app |
| Right knob click | select / act (pick aircraft, refresh, snooze) |
| Left knob click | back to the app's home view |
| Double click either | start / pause Pomodoro |
| Hold left 2 s | menu / settings (as v5 hold) |
| Alerts | take over both dials briefly, then return |

Rings: brightness cap 20 percent by default (setting), off in sleep hours, per-channel correction start R 0.7 G 1.0 B 0.5 (tune by eye).

## 7. Definition of done for each release

Compiles clean, flashed, 2 minutes of log captured with no resets, heap numbers recorded, HANDOFF.md version table and open issues updated, owner told what changed and what to look at on the device.
