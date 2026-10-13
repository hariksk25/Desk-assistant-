# Desk Plane Radar. v5 = classic ESP32 WROOM-32 (old device), v6 = ESP32-S3 N16R8 twin dial. Run from the repo root.
SKETCH := firmware/desk_plane_radar_v5
FQBN   := esp32:esp32:esp32:PartitionScheme=min_spiffs
BUILD  := build
PORT   ?= $(firstword $(wildcard /dev/cu.usbserial-* /dev/cu.wchusbserial* /dev/cu.SLAB_USBtoUART /dev/cu.usbmodem*))
HOST   ?= deskradar.local
# v6 (ESP32-S3 N16R8). Verify keys with: arduino-cli board details -b esp32:esp32:esp32s3
SKETCH6 := firmware/desk_plane_radar_v6
FQBN6   := esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=cdc,USBMode=hwcdc
BUILD6  := build6
PORT6   ?= $(firstword $(wildcard /dev/cu.usbmodem* /dev/cu.wchusbserial* /dev/cu.usbserial-*))
# v6 hostname while the v5 device still answers deskradar.local (no comment on the value line: make keeps the spaces)
HOST6   ?= deskradar6.local
SECS   ?= 60

.PHONY: deps compile upload ota log weblog port secrets-check case secrets6 compile6 upload6 ota6 log6 weblog6 port6

deps:            ## one-time: ESP32 core + libraries
	arduino-cli config init --overwrite
	arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
	arduino-cli core update-index
	arduino-cli core install esp32:esp32
	arduino-cli lib install "Adafruit GFX Library" "Adafruit SSD1306" "GFX Library for Arduino" "ArduinoJson" "WiFiManager" "Adafruit NeoPixel" "Adafruit BMP280 Library" "Adafruit BME280 Library" "ESP32Encoder"
	python3 -m pip install --user pyserial trimesh manifold3d shapely numpy pillow

secrets-check:
	@test -f $(SKETCH)/secrets.h || (echo "Missing $(SKETCH)/secrets.h: cp $(SKETCH)/secrets.example.h $(SKETCH)/secrets.h and fill it in"; exit 1)

compile: secrets-check
	arduino-cli compile --fqbn $(FQBN) --output-dir $(BUILD) $(SKETCH)

upload: compile   ## USB flash (first time, or when OTA says "Update.begin failed")
	arduino-cli upload -p $(PORT) --fqbn $(FQBN),UploadSpeed=115200 --input-dir $(BUILD) $(SKETCH)

ota: compile     ## flash over WiFi (curl needs -H "Expect:")
	curl -H "Expect:" -F "fw=@$(BUILD)/desk_plane_radar_v5.ino.bin" http://$(HOST)/update

log:             ## serial log for SECS seconds (default 60)
	python3 scripts/serial_log.py $(PORT) $(SECS)

weblog:          ## the device's 3 KB log ring over WiFi
	curl -s http://$(HOST)/log

port:
	@echo $(PORT)

case:            ## regenerate the v6 STLs and run fit/clearance checks + renders
	python3 case_v6/make_case_v6.py
	python3 case_v6/check_case.py
	cd case_v6 && python3 render_section.py && python3 render_final.py && python3 render_layout.py

# ---- v6 (ESP32-S3) ----
secrets6:        ## copy the v5 secrets into the v6 sketch (gitignored)
	@test -f $(SKETCH6)/secrets.h || cp $(SKETCH)/secrets.h $(SKETCH6)/secrets.h
	@test -f $(SKETCH6)/secrets.h && echo "v6 secrets.h present"

compile6: secrets6
	arduino-cli compile --fqbn $(FQBN6) --output-dir $(BUILD6) $(SKETCH6)

upload6: compile6   ## USB flash. If it fails: hold BOOT, tap RST, release BOOT, retry
	arduino-cli upload -p $(PORT6) --fqbn $(FQBN6) --input-dir $(BUILD6) $(SKETCH6)

ota6: compile6
	curl -4 -H "Expect:" -F "fw=@$(BUILD6)/desk_plane_radar_v6.ino.bin" http://$(HOST6)/update   # -4: the .local IPv6 lookup costs 5 s on this Mac

weblog6:         ## the v6 log ring (16 KB) over WiFi; does not reset the board like log6 does
	curl -4 -s http://$(HOST6)/log

log6:            ## serial log for SECS seconds (opening the port resets the S3 on macOS)
	python3 scripts/serial_log.py $(PORT6) $(SECS)

port6:
	@echo $(PORT6)
