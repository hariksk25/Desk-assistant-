// SD slot probe for the v6 board (12 Oct 2026). Which pins is the on-board microSD slot wired to?
// Hypothesis: SPI mode on the S3 default FSPI pins (CS 10, MOSI 11, SCK 12, MISO 13): GPIO 10 has a board
// pull-up, and the left TFT failed only while its CS was on GPIO 10 (the card would drive GPIO 13 = TFT DC).
// Both round displays stay deselected (CS 41 / 16 high). Repeats every 15 s so a late serial log sees it.
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <SD_MMC.h>

const int CS_L = 41, CS_R = 16;

void pullTest() {
  const int pins[] = {0, 3, 10, 11, 12, 13, 14, 38, 39, 40, 42, 45, 46, 48};
  Serial.print("board pull-ups (reads 1 with the internal pull-down on):");
  for (int p : pins) { pinMode(p, INPUT_PULLDOWN); delay(2); Serial.printf(" %d=%d", p, digitalRead(p)); pinMode(p, INPUT); }
  Serial.println();
}

void listAndTest(fs::FS &fs, const char *how) {
  File f = fs.open("/sd_probe.txt", FILE_WRITE);
  bool w = f && f.print("desk radar v6 sd probe\n") > 0; if (f) f.close();
  f = fs.open("/sd_probe.txt", FILE_READ); String r = f ? f.readStringUntil('\n') : ""; if (f) f.close();
  Serial.printf("  %s: write %s, read back \"%s\"\n", how, w ? "ok" : "FAILED", r.c_str());
  File root = fs.open("/"); int n = 0;
  for (File e = root.openNextFile(); e && n < 8; e = root.openNextFile(), n++) Serial.printf("    /%s%s\n", e.name(), e.isDirectory() ? "/" : "");
}

bool trySpi(int sck, int miso, int mosi, int cs, uint32_t hz) {
  SPIClass *spi = new SPIClass(FSPI);
  spi->begin(sck, miso, mosi, cs);
  bool ok = SD.begin(cs, *spi, hz);
  Serial.printf("SPI mode SCK %d MISO %d MOSI %d CS %d @%lu kHz: %s", sck, miso, mosi, cs, (unsigned long)(hz / 1000), ok ? "MOUNTED" : "no card\n");
  if (ok) {
    const char *t[] = {"none", "MMC", "SD", "SDHC", "unknown"}; uint8_t ct = SD.cardType();
    Serial.printf(", %s, %llu MB, FAT %llu MB\n", t[ct < 4 ? ct : 4], SD.cardSize() >> 20, SD.totalBytes() >> 20);
    listAndTest(SD, "SPI");
    SD.end();
  }
  spi->end(); delete spi;
  return ok;
}

bool tryMmc(int clk, int cmd, int d0) {
  if (!SD_MMC.setPins(clk, cmd, d0)) { Serial.printf("SD_MMC CLK %d CMD %d D0 %d: setPins failed\n", clk, cmd, d0); return false; }
  bool ok = SD_MMC.begin("/sdcard", true, false, 400);
  Serial.printf("SD_MMC 1-bit CLK %d CMD %d D0 %d @400 kHz: %s", clk, cmd, d0, ok ? "MOUNTED" : "no card\n");
  if (ok) { Serial.printf(", %llu MB\n", SD_MMC.cardSize() >> 20); listAndTest(SD_MMC, "SD_MMC"); }
  SD_MMC.end();
  return ok;
}

void probe() {
  Serial.println("\n==== SD PROBE ====");
  pullTest();
  bool found = trySpi(12, 13, 11, 10, 400000);
  if (found) trySpi(12, 13, 11, 10, 20000000);
  else {
    found |= trySpi(11, 13, 12, 10, 400000);     // clock and data-in swapped
    found |= trySpi(12, 13, 11, 10, 100000);     // slower
    found |= trySpi(14, 13, 11, 10, 400000);     // clock on 14 (the TFT reset line)
  }
  const int p[6][3] = {{39, 38, 40}, {38, 39, 40}, {39, 40, 38}, {40, 39, 38}, {38, 40, 39}, {40, 38, 39}};
  for (auto &c : p) found |= tryMmc(c[0], c[1], c[2]);
  Serial.println(found ? "==== card found ====" : "==== no card on any tried wiring ====");
}

void setup() {
  pinMode(CS_L, OUTPUT); digitalWrite(CS_L, HIGH);
  pinMode(CS_R, OUTPUT); digitalWrite(CS_R, HIGH);
  Serial.begin(115200); Serial.setTxTimeoutMs(0);
  unsigned long t0 = millis(); while (!Serial && millis() - t0 < 3000) delay(10);
  probe();
}

void loop() {
  static unsigned long last = millis();
  if (millis() - last > 15000) { last = millis(); probe(); }
  delay(20);
}
