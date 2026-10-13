"""Desk Plane Radar v6, phase 1 wiring diagram (ESP32-S3 N16R8 with on-board microSD).
Phase 1: 2 x GC9A01 round TFT, 2 x SSD1306 0.96" OLED, 2 x 24-LED WS2812B ring, BMP280 4-pin module, passive piezo, 2 x KY-040.
Phase 2 (not drawn, pins reserved): MAX98357A amp + speaker, LiPo + charger/boost, light / presence sensors.
13 Oct 2026 (as built after the panel rebuild): TFT CS left 16 / right 41, left OLED alone on I2C bus 1 (GPIO 1 / 2),
right OLED + BMP280 on bus 0 (8 / 9), left encoder 15 / 17 / 18, right encoder 4 / 5 / 6. Rings unchanged (47 left, 21 right).
Run: python3 tools/wiring_v6_phase1.py [out.png]"""
import sys, os
LS = int(os.environ.get("LS", 128)); RS = int(os.environ.get("RS", 128))
LP = int(os.environ.get("LP", 80)); RP = int(os.environ.get("RP", 65))
from PIL import Image, ImageDraw, ImageFont

OUT = sys.argv[1] if len(sys.argv) > 1 else "docs/images/wiring_v6_phase1.png"
W, H = 2000, 1720
img = Image.new("RGB", (W, H), "white"); d = ImageDraw.Draw(img)
_FONTS = [("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"),
          ("/System/Library/Fonts/Supplemental/Arial.ttf", "/System/Library/Fonts/Supplemental/Arial Bold.ttf")]   # Linux, macOS
_REG, _BOLD = next(f for f in _FONTS if os.path.exists(f[0]))
F = lambda s: ImageFont.truetype(_REG, s)
FB = lambda s: ImageFont.truetype(_BOLD, s)
C = {"5V": "#d62728", "3V3": "#ff7f0e", "GND": "#222222", "SCK": "#1f77b4", "MOSI": "#17becf", "DC": "#9467bd",
     "RST": "#8c564b", "CS": "#e377c2", "SDA": "#2ca02c", "SCL": "#006400", "SDA2": "#2ca02c", "SCL2": "#006400", "ENC": "#7f7f7f", "DATA": "#bcbd22", "PZ": "#555555", "NC": "#bbbbbb"}

d.text((30, 18), "Desk Plane Radar v6  -  PHASE 1 wiring  (ESP32-S3 N16R8, on-board microSD)", font=FB(30), fill="black")
d.text((30, 60), "USB-C (90 degree cable, left side) powers everything. Rings take 5 V from the 5V pin, everything else 3V3. All GNDs common. "
       "Left / right = as seen from the FRONT.", font=F(17), fill="#444")
d.text((30, 86), "Phase 2 (not drawn, pins kept free): I2S amp + 3 W speaker (39 / 40 / 7, replaces the piezo), LiPo + charger / boost, "
       "light sensor (I2C bus 0), presence sensor on 42 / 48.", font=F(17), fill="#444")

# ---- board
bx0, by0, bx1, by1 = 830, 170, 1170, 1335
d.rounded_rectangle((bx0, by0, bx1, by1), 14, fill="#e8f0e8", outline="black", width=3)
d.text((bx0 + 22, by0 + 14), "ESP32-S3 N16R8", font=FB(22), fill="black")
d.text((bx0 + 22, by0 + 44), "(board with microSD slot)", font=F(15), fill="#333")
d.rounded_rectangle((bx0 + 136, by1 - 128, bx1 - 14, by1 - 14), 8, fill="#d9d9d9", outline="#555", width=2)
d.text((bx0 + 150, by1 - 118), "microSD (on board, SPI)", font=FB(15), fill="black")
d.text((bx0 + 150, by1 - 92), "CS 10  MISO 13", font=F(14), fill="black")
d.text((bx0 + 150, by1 - 66), "SCK 12, MOSI 11 shared", font=F(14), fill="#444"); d.text((bx0 + 150, by1 - 44), "with the displays", font=F(14), fill="#444")

pinsL = [("3V3", "3V3"), ("GND", "GND"), ("12  SCK", "SCK"), ("11  MOSI", "MOSI"), ("38  DC", "DC"), ("14  RST", "RST"),
         ("16  CS left", "CS"), ("41  CS right", "CS"), ("1   SDA (L OLED)", "SDA2"), ("2   SCL (L OLED)", "SCL2"),
         ("8   SDA", "SDA"), ("9   SCL", "SCL")]
pinsR = [("5V", "5V"), ("GND", "GND"), ("4   ENC-R CLK", "ENC"), ("5   ENC-R DT", "ENC"), ("6   ENC-R SW", "ENC"),
         ("15  ENC-L CLK", "ENC"), ("17  ENC-L DT", "ENC"), ("18  ENC-L SW", "ENC"), ("47  RING-L DIN", "DATA"),
         ("21  RING-R DIN", "DATA"), ("7   PIEZO", "PZ")]
PL, PR = {}, {}
for i, (lab, net) in enumerate(pinsL):
    y = by0 + LS + i * LP; PL[lab] = (bx0, y)
    d.ellipse((bx0 - 8, y - 8, bx0 + 8, y + 8), fill=C[net]); d.text((bx0 + 16, y - 10), lab, font=FB(16), fill="black")
for i, (lab, net) in enumerate(pinsR):
    y = by0 + RS + i * RP; PR[lab] = (bx1, y)
    d.ellipse((bx1 - 8, y - 8, bx1 + 8, y + 8), fill=C[net])
    tw = d.textlength(lab, font=FB(16)); d.text((bx1 - 18 - tw, y - 10), lab, font=FB(16), fill="black")

def module(x, y, w, title, pins, side, sub=None):
    h = 46 + len(pins) * 26 + (22 if sub else 0)
    d.rounded_rectangle((x, y, x + w, y + h), 10, fill="#f4f4f4", outline="black", width=2)
    d.text((x + 10, y + 8), title, font=FB(16), fill="black")
    if sub: d.text((x + 10, y + 30), sub, font=F(13), fill="#555")
    out = {}
    for i, (lab, net) in enumerate(pins):
        py = y + 46 + (22 if sub else 0) + i * 26
        px = x + w if side == "L" else x
        d.ellipse((px - 6, py - 6, px + 6, py + 6), fill=C[net])
        if side == "L":
            tw = d.textlength(lab, font=F(15)); d.text((px - 14 - tw, py - 9), lab, font=F(15), fill="black")
        else:
            d.text((px + 14, py - 9), lab, font=F(15), fill="black")
        out[lab] = (px, py)
    return out

SEGS = []
def seg(pts, net, w=3):
    d.line(pts, fill=C[net], width=w)
    for a, b in zip(pts[:-1], pts[1:]): SEGS.append((a, b, net))
def overlaps():   # parallel segments of different nets closer than 12 px with overlapping spans
    bad = []
    for i, (a, b, n) in enumerate(SEGS):
        for (c, e, m) in SEGS[i + 1:]:
            if n == m: continue
            if a[1] == b[1] and c[1] == e[1] and abs(a[1] - c[1]) < 12:
                lo, hi = max(min(a[0], b[0]), min(c[0], e[0])), min(max(a[0], b[0]), max(c[0], e[0]))
                if hi - lo > 2: bad.append(("H", round(a[1]), round(c[1]), n, m, round(lo), round(hi)))
            if a[0] == b[0] and c[0] == e[0] and abs(a[0] - c[0]) < 10:
                lo, hi = max(min(a[1], b[1]), min(c[1], e[1])), min(max(a[1], b[1]), max(c[1], e[1]))
                if hi - lo > 2: bad.append(("V", round(a[0]), round(c[0]), n, m, round(lo), round(hi)))
    return bad
def dot(p, net): d.ellipse((p[0] - 5, p[1] - 5, p[0] + 5, p[1] + 5), fill=C[net])

# ---- left modules
tft = [("VCC", "3V3"), ("GND", "GND"), ("SCL", "SCK"), ("SDA", "MOSI"), ("RES", "RST"), ("DC", "DC"), ("CS", "CS")]
oled = [("GND", "GND"), ("VCC", "3V3"), ("SCL", "SCL"), ("SDA", "SDA")]
bmp = [("VIN", "3V3"), ("GND", "GND"), ("SCL", "SCL"), ("SDA", "SDA")]
TL = module(60, 140, 330, "Round TFT, LEFT dial", tft, "L", "GC9A01, 7 pin")
TR = module(60, 420, 330, "Round TFT, RIGHT dial", tft, "L", "GC9A01, 7 pin")
OL = module(60, 700, 330, "OLED, RIGHT dial  (0x3C, bus 0)", oled, "L", "SSD1306 0.96, shares bus 0 with the BMP280")
OR = module(60, 900, 330, "OLED, LEFT dial  (0x3C, bus 1)", oled, "L", "own I2C bus on GPIO 1 / 2, see note 2")
BM = module(60, 1100, 330, "BMP280, 4 pin  (0x76 / 0x77)", bmp, "L", "VIN to 3V3; address set on the board")

# power buses (left): 3V3 x=430, GND x=460
X3, XG = 430, 462
lmods = [TL, TR, OL, OR, BM]
y3 = [m[k][1] for m in lmods for k in m if k in ("VCC", "VIN")]; yg = [m[k][1] for m in lmods for k in m if k == "GND"]
seg([(X3, 112), (X3, max(y3))], "3V3", 5); seg([(XG, min(yg)), (XG, max(yg))], "GND", 5)
seg([(X3, PL["3V3"][1]), PL["3V3"]], "3V3", 5); dot((X3, PL["3V3"][1]), "3V3")
seg([(XG, PL["GND"][1]), PL["GND"]], "GND", 5); dot((XG, PL["GND"][1]), "GND")
for m in lmods:
    for k, v in m.items():
        if k in ("VCC", "VIN"): seg([v, (X3, v[1])], "3V3"); dot((X3, v[1]), "3V3")
        if k == "GND": seg([v, (XG, v[1])], "GND"); dot((XG, v[1]), "GND")
# shared SPI lines
for net, lab, xb, pin in (("SCK", "SCL", 500, "12  SCK"), ("MOSI", "SDA", 530, "11  MOSI"), ("DC", "DC", 560, "38  DC"), ("RST", "RES", 590, "14  RST")):
    ys = [TL[lab][1], TR[lab][1], PL[pin][1]]
    seg([(xb, min(ys)), (xb, max(ys))], net)
    for m in (TL, TR): seg([m[lab], (xb, m[lab][1])], net); dot((xb, m[lab][1]), net)
    seg([(xb, PL[pin][1]), PL[pin]], net); dot((xb, PL[pin][1]), net)
seg([TL["CS"], (625, TL["CS"][1]), (625, PL["16  CS left"][1]), PL["16  CS left"]], "CS")
seg([TR["CS"], (650, TR["CS"][1]), (650, PL["41  CS right"][1]), PL["41  CS right"]], "CS")
# I2C bus 0: right OLED + BMP280 on 8 / 9 (13 Oct rebuild)
for net, lab, xb, pin in (("SDA", "SDA", 700, "8   SDA"), ("SCL", "SCL", 735, "9   SCL")):
    ys = [OL[lab][1], BM[lab][1], PL[pin][1]]
    seg([(xb, min(ys)), (xb, max(ys))], net)
    for m in (OL, BM): seg([m[lab], (xb, m[lab][1])], net); dot((xb, m[lab][1]), net)
    seg([(xb, PL[pin][1]), PL[pin]], net); dot((xb, PL[pin][1]), net)
# I2C bus 1: left OLED alone on 1 / 2 (plain wires, crossing bus 0 without joining it)
for lab, net, xb, pin in (("SDA", "SDA2", 770, "1   SDA (L OLED)"), ("SCL", "SCL2", 800, "2   SCL (L OLED)")):
    seg([OR[lab], (xb, OR[lab][1]), (xb, PL[pin][1]), PL[pin]], net)

# ---- right modules
enc = [("VCC", "3V3"), ("GND", "GND"), ("S1A  (CLK)", "ENC"), ("S1B  (DT)", "ENC"), ("SW", "ENC")]
ring = [("5V", "5V"), ("GND", "GND"), ("DIN", "DATA")]
EL = module(1620, 140, 330, "Encoder, LEFT knob", enc, "R", "VCC to 3V3 (NOT 5V)")
ER = module(1620, 370, 330, "Encoder, RIGHT knob", enc, "R", "VCC to 3V3 (NOT 5V)")
RL = module(1620, 610, 330, "LED ring, LEFT dial", ring, "R", "24 x WS2812B, wired direct")
RR = module(1620, 800, 330, "LED ring, RIGHT dial", ring, "R", "24 x WS2812B, wired direct")
PZ = module(1620, 990, 330, "Passive piezo", [("+", "PZ"), ("-", "GND")], "R", "12 mm buzzer")

# 3V3 to the encoders: from the left bus over the top
X3R, XGR, X5 = 1580, 1550, 1520
seg([(X3, 112), (X3R, 112), (X3R, ER["VCC"][1])], "3V3", 5)
for m in (EL, ER): seg([m["VCC"], (X3R, m["VCC"][1])], "3V3"); dot((X3R, m["VCC"][1]), "3V3")
# GND bus right
gy = [m["GND"][1] for m in (EL, ER, RL, RR)] + [PZ["-"][1]]
seg([(XGR, min(gy)), (XGR, max(gy))], "GND", 5)
seg([PR["GND"], (XGR, PR["GND"][1])], "GND", 5); dot((XGR, PR["GND"][1]), "GND")
for m in (EL, ER, RL, RR): seg([m["GND"], (XGR, m["GND"][1])], "GND"); dot((XGR, m["GND"][1]), "GND")
seg([PZ["-"], (XGR, PZ["-"][1])], "GND"); dot((XGR, PZ["-"][1]), "GND")
# 5V bus to the rings
seg([PR["5V"], (X5, PR["5V"][1]), (X5, RR["5V"][1])], "5V", 5)
for m in (RL, RR): seg([m["5V"], (X5, m["5V"][1])], "5V", 4); dot((X5, m["5V"][1]), "5V")
# encoder signals
xs = iter(range(1250, 1500, 26))
for m, pre in ((EL, "ENC-L"), (ER, "ENC-R")):
    for lab, pn in (("S1A  (CLK)", "CLK"), ("S1B  (DT)", "DT"), ("SW", "SW")):
        pin = [p for p in PR if p.endswith(f"{pre} {pn}")][0]; xm = next(xs)
        seg([PR[pin], (xm, PR[pin][1]), (xm, m[lab][1]), m[lab]], "ENC")

def resistor(xm, y, label):   # small resistor box centred on a horizontal wire

    d.rectangle((xm - 22, y - 9, xm + 22, y + 9), fill="white", outline="black", width=2)
    d.text((xm - 24, y - 30), label, font=FB(14), fill="black")

def cap(x, ytop, ybot, label):
    d.line([(x, ytop), (x, ytop + 14)], fill=C["5V"], width=3); d.line([(x - 14, ytop + 14), (x + 14, ytop + 14)], fill="black", width=3)
    d.line([(x - 14, ytop + 22), (x + 14, ytop + 22)], fill="black", width=3); d.line([(x, ytop + 22), (x, ybot)], fill=C["GND"], width=3)
    d.text((x + 18, ytop + 4), label, font=FB(13), fill="black")

# ring data lines with 330 ohm near the ring, cap across 5V/GND at each ring
for m, pin, xm in ((RL, "47  RING-L DIN", 1440), (RR, "21  RING-R DIN", 1466)):
    seg([PR[pin], (xm, PR[pin][1]), (xm, m["DIN"][1]), m["DIN"]], "DATA")
    # 12 Oct (owner): built with no series resistor and no bulk capacitor on the rings
# piezo
seg([PR["7   PIEZO"], (1492, PR["7   PIEZO"][1]), (1492, PZ["+"][1]), PZ["+"]], "PZ")
# 12 Oct (owner): piezo wired straight to GPIO 7, no series resistor

# ---- notes
ny = 1370
d.line([(30, ny - 14), (W - 30, ny - 14)], fill="#999", width=2)
notes = [
    ("1", "Displays share SPI (SCK 12, MOSI 11, DC 38, RST 14), CS left 16, right 41 (13 Oct rebuild). The on-board microSD is on the same bus: CS 10, MISO 13."),
    ("2", "Both OLEDs are 0x3C, so each has its own I2C bus: RIGHT OLED + BMP280 on 8 / 9, LEFT OLED alone on 1 / 2 (13 Oct rebuild). BMP280: 0x76 or 0x77 (firmware scans both)."),
    ("3", "Rings: wired direct (no 330 ohm, no 1000 uF as built, 12 Oct); firmware brightness cap 20 percent (SETTINGS > DISPLAY > ring brightness)."),
    ("", "    If the first LED flickers (3.3 V data into 5 V LEDs): put a 1N4007 diode in each ring's 5V feed, or add the 74AHCT125 later."),
    ("4", "Encoders on 3V3 only (the S3 pins are not 5 V tolerant). SW uses the S3 internal pull-up. Piezo: direct to GPIO 7 (no series resistor), minus to GND."),
    ("5", "Do not use: 0, 3, 45, 46 (boot pins), 19 / 20 (USB), 26 to 37 (flash + PSRAM), 10 / 13 (microSD CS / MISO), 43 / 44 (serial log)."),
    ("", "    Reserved for phase 2: 39 / 40 / 7 I2S amp BCLK / LRCLK / DIN (7 moves from the piezo), 42 or 48 presence sensor, light sensor on I2C bus 0."),
    ("6", "Do not plug a camera into the board's camera connector: its pins are shared with the encoders, displays and I2C above."),
    ("7", "Power: 5 V 2 A USB charger. Budget: S3 + WiFi 0.3 A, displays + OLEDs 0.2 A, 48 LEDs at the 20 percent cap about 0.6 A = about 1.1 A."),
]
for i, (n, t) in enumerate(notes):
    d.text((40, ny + i * 34), n, font=FB(17), fill="black"); d.text((70, ny + i * 34), t, font=F(17), fill="black")
# legend
lx = 1250; ly = 125
for i, (k, lab) in enumerate((("5V", "5V"), ("3V3", "3V3"), ("GND", "GND"), ("SDA", "I2C"), ("SCK", "SPI"), ("DATA", "LED data"))):
    d.line([(lx + i * 58, ly), (lx + i * 58 + 22, ly)], fill=C[k], width=6); d.text((lx + i * 58, ly + 6), lab, font=F(11), fill="#333")
for b in overlaps(): print("CLASH", b)
img.save(OUT); print(OUT)
