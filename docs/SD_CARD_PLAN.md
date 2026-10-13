# v6 microSD plan (ESP32-S3 N16R8 board with SD slot)

Status: plan only, not built. Applies to the v6 firmware (S3), behind `#define USE_SD 1`.

## 0. Confirm the board first

The official ESP32-S3-DevKitC-1 has no SD slot, so the owner's board is a variant (usually the "ESP32-S3 WROOM N16R8 CAM" style board with a camera connector and microSD). Before any code or case work:

| Check | Why |
|---|---|
| SD pins from the board's silkscreen / seller pinout | common layout is SDMMC 1-bit **CMD 38, CLK 39, D0 40**; verify, do not assume |
| Board outline, hole positions, USB-C location(s) | the v6 case tray assumes DevKitC-1 (69.9 x 25.5 mm, USB on the short end). A CAM board differs: update `BOARD_L/BOARD_W/TRAY_POS/USB_*` in `case_v6/make_case_v6.py`, add an SD access slot, run `make case` |
| Camera pins | unused (no camera), so they stay free for our pin map |

## 1. Pin map change (SD on 38/39/40 collides with the old plan)

| Signal | Old plan | New plan |
|---|---|---|
| SD CMD / CLK / D0 | n/a | 38 / 39 / 40 (confirm) |
| I2S BCLK / LRCLK / DIN (MAX98357A) | 40 / 41 / 42 | **41 / 42 / 7** (GPIO 7 is free because the I2S amp replaces the piezo) |
| LD2410 UART RX / TX | 38 / 39 | **1 / 2** |
| Spare | | 48 (onboard RGB LED on many boards) |
| Unchanged | SPI 11/12, TFT CS 10/16, DC 13, RST 14, I2C 8/9, encoders 4/5/6 and 15/17/18, rings 47/21 | |
| Never use | 0, 3, 45, 46 (strapping), 19/20 (USB), 26 to 37 (flash/PSRAM on N16R8), 43/44 (serial log) | |

Update `tools/wiring_v6.py` and HANDOFF.md section 4 once confirmed.

## 2. Interface

SD_MMC in 1-bit mode (faster than SPI, no extra SPI bus, no clash with the shared display bus):

```
#include <SD_MMC.h>
SD_MMC.setPins(39 /*CLK*/, 38 /*CMD*/, 40 /*D0*/);
bool sdOk = SD_MMC.begin("/sdcard", true /*1-bit*/);
```

Card: 4 to 32 GB, FAT32. The firmware must run normally with no card (every feature falls back to online lookups or is hidden).

## 3. Card layout

| Path | Content | Written by |
|---|---|---|
| `/db/aircraft.bin` | sorted fixed-size records: ICAO hex (3 B) + registration + type + operator/flags, about 15 to 20 MB | Mac script, copied to the card |
| `/db/routes.bin` | sorted records: callsign + origin/destination ICAO codes | Mac script |
| `/db/version.txt` | build date of both DBs | Mac script |
| `/sounds/*.wav` | 16 kHz mono clips: chime, ding-dong, Pomodoro bell, spoken lines ("Session done") | owner |
| `/img/` | airline logos, aircraft silhouettes, weather icons as raw RGB565 | Mac script |
| `/fonts/` | large anti-aliased fonts (letters too) for the PSRAM framebuffer | `tools/gen_fonts.py` |
| `/log/focus.csv` | every Pomodoro session (backup of the Google log) | device |
| `/log/flights/YYYY-MM-DD.csv` | aircraft seen: hex, callsign, type, closest distance, time | device, batched each minute |
| `/log/sensors/YYYY-MM.csv` | temp, humidity, pressure, light every 5 min | device |
| `/log/crash.txt` | reset reason + last log ring on boot after a crash | device |
| `/config.json` | optional overrides (WiFi, tokens, settings) read at boot | owner |

## 4. Build order

| Phase | Feature | Done when |
|---|---|---|
| 1 | Mount, status on the SYSTEM page, `/log/crash.txt`, `/log/focus.csv` | card info shows; a session appears in focus.csv |
| 2 | Aircraft DB lookup (binary search, about 20 reads) replacing hexdb.io; route DB replacing adsb.lol `routeset` (403 today) | type/registration/route show with WiFi lookups disabled |
| 3 | WAV playback through the I2S amp | Pomodoro and alert sounds play from the card |
| 4 | Logos, icons, big fonts | arrivals board shows airline logos |
| 5 | Flight and sensor history pages | "seen today / this month", 7-day temperature graph |
| 6 | `/config.json` overrides | WiFi change without reflashing |

DB sources to evaluate (check licences): tar1090-db (aircraft by ICAO hex), VRS standing data (routes). Builder script goes in `tools/build_sd_db.py`, output to a local `sd_card/` folder (gitignored) that the owner copies to the card.

## 5. Rules

- Read-mostly. Batch writes (once a minute), always close files, never write every frame.
- All SD access from one task (the UI task, or a dedicated low-priority task), never inside paint functions.
- Card errors must not block the UI: time out, mark the card offline, retry later.
- Never store secrets on the card unless the owner chooses `/config.json`; never log them.
