"""Desk Plane Radar v6: FRONT PANEL wiring only (13 Oct 2026 rebuild, as wired: left / right swapped for TFT CS, OLED bus and encoders; rings unchanged).
Parts on the panel: 2 x GC9A01 round TFT (7 pin), 2 x SSD1306 0.96" OLED, 2 x 24-LED WS2812B ring, 2 x KY-040 encoder.
Not on the panel (not drawn): ESP32-S3 board with the microSD, BMP280, piezo.
Run: python3 tools/wiring_v6_panel.py [out.png]     (writes docs/images/wiring_v6_panel.png)"""
import sys, os
from PIL import Image, ImageDraw, ImageFont

OUT = sys.argv[1] if len(sys.argv) > 1 else "docs/images/wiring_v6_panel.png"
W, H = 2000, 1640
img = Image.new("RGB", (W, H), "white"); d = ImageDraw.Draw(img)
_FONTS = [("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"),
          ("/System/Library/Fonts/Supplemental/Arial.ttf", "/System/Library/Fonts/Supplemental/Arial Bold.ttf")]
_REG, _BOLD = next(f for f in _FONTS if os.path.exists(f[0]))
F = lambda s: ImageFont.truetype(_REG, s)
FB = lambda s: ImageFont.truetype(_BOLD, s)
C = {"5V": "#d62728", "3V3": "#ff7f0e", "GND": "#222222", "SCK": "#1f77b4", "MOSI": "#17becf", "DC": "#9467bd", "RST": "#8c564b",
     "CS": "#e377c2", "SDA": "#2ca02c", "SCL": "#006400", "DATA": "#bcbd22", "ENC": "#7f7f7f"}

d.text((30, 18), "Desk Plane Radar v6  -  FRONT PANEL wiring  (panel parts only, one harness to the ESP32-S3)", font=FB(30), fill="black")
d.text((30, 60), "Front view. Every pad shows the wire that must reach it: net, and the ESP32-S3 GPIO it ends on. "
       "Chain the shared nets on the panel so only ONE wire per net leaves the panel.", font=F(17), fill="#444")
d.text((30, 86), "Rings: 5 V from the board's 5V pin. Everything else 3V3. All GNDs are one net. No resistors, no capacitors "
       "(as built). Encoder VCC to 3V3, never 5 V.", font=F(17), fill="#444")

# ---- panel outline
px0, py0, px1, py1 = 40, 120, 1960, 1130
d.rounded_rectangle((px0, py0, px1, py1), 24, fill="#f7f7fb", outline="#333", width=3)
d.text((px0 + 16, py0 + 10), "FRONT PANEL (as seen from the front; left dial = left)", font=FB(18), fill="#333")

def pad_list(x, y, title, sub, pads, colour_title="#000"):
    """A component box with its pads; each pad: (pad name, net, wire text)."""
    w, h = 400, 48 + 26 * len(pads) + (20 if sub else 0)
    d.rounded_rectangle((x, y, x + w, y + h), 10, fill="white", outline="black", width=2)
    d.text((x + 12, y + 8), title, font=FB(17), fill=colour_title)
    if sub: d.text((x + 12, y + 30), sub, font=F(13), fill="#555")
    for i, (pad, net, wire) in enumerate(pads):
        yy = y + 48 + (20 if sub else 0) + i * 26
        d.ellipse((x + 16, yy - 7, x + 30, yy + 7), fill=C[net])
        d.text((x + 40, yy - 9), pad, font=FB(15), fill="black")
        d.text((x + 150, yy - 9), wire, font=F(15), fill=C[net] if net not in ("GND",) else "#222")
    return (x, y, x + w, y + h)

# ---- dial graphics (ring, TFT, OLED, knob) for each side
def dial(cx, cy, label):
    d.ellipse((cx - 150, cy - 150, cx + 150, cy + 150), outline="#bcbd22", width=14)       # ring
    d.ellipse((cx - 112, cy - 112, cx + 112, cy + 112), fill="#111", outline="#555", width=3)  # TFT
    d.text((cx - 36, cy - 12), label, font=FB(22), fill="white")
    d.rectangle((cx - 80, cy + 172, cx + 80, cy + 232), fill="#111", outline="#555", width=3)   # OLED
    d.rectangle((cx - 76, cy + 176, cx + 76, cy + 190), fill="#8a7a00")                        # yellow band (mounted rotated: appears at the top)
    d.text((cx - 30, cy + 200), "OLED", font=F(15), fill="white")
    d.ellipse((cx - 34, cy + 262, cx + 34, cy + 330), fill="#444", outline="#222", width=3)     # knob
    d.text((cx - 24, cy + 286), "knob", font=F(15), fill="white")

dial(560, 330, "LEFT"); dial(1440, 330, "RIGHT")
d.text((440, 150), "left TFT  +  left ring  +  left OLED  +  left knob", font=F(14), fill="#666")
d.text((1300, 150), "right TFT  +  right ring  +  right OLED  +  right knob", font=F(14), fill="#666")

# ---- pad lists: TFTs beside the dials (outer side), OLED / ring / encoder below
tftL = [("VCC", "3V3", "3V3  (chain to the right TFT)"), ("GND", "GND", "GND  (chain)"), ("SCL", "SCK", "SCK  GPIO 12  (chain)"),
        ("SDA", "MOSI", "MOSI  GPIO 11  (chain)"), ("RES", "RST", "RST  GPIO 14  (chain)"), ("DC", "DC", "DC  GPIO 38  (chain)"),
        ("CS", "CS", "CS  GPIO 16   (own wire)")]
tftR = [("VCC", "3V3", "3V3  (from the left TFT)"), ("GND", "GND", "GND  (from the left TFT)"), ("SCL", "SCK", "SCK  GPIO 12  (from left)"),
        ("SDA", "MOSI", "MOSI  GPIO 11  (from left)"), ("RES", "RST", "RST  GPIO 14  (from left)"), ("DC", "DC", "DC  GPIO 38  (from left)"),
        ("CS", "CS", "CS  GPIO 41   (own wire)")]
pad_list(60, 490, "Round TFT, LEFT  (GC9A01, 7 pads)", "6 of 7 pads are shared with the right TFT", tftL)
pad_list(1540, 490, "Round TFT, RIGHT  (GC9A01, 7 pads)", "only CS is its own wire", tftR)

oledL = [("GND", "GND", "GND"), ("VCC", "3V3", "3V3"), ("SCL", "SCL", "SCL  GPIO 2   (I2C bus 1)"), ("SDA", "SDA", "SDA  GPIO 1   (I2C bus 1)")]
oledR = [("GND", "GND", "GND"), ("VCC", "3V3", "3V3"), ("SCL", "SCL", "SCL  GPIO 9   (I2C bus 0)"), ("SDA", "SDA", "SDA  GPIO 8   (I2C bus 0)")]
pad_list(480, 720, "OLED, LEFT  (SSD1306, addr 0x3C)", "bus 1: its OWN two wires, nothing else on them", oledL)
pad_list(1120, 720, "OLED, RIGHT  (SSD1306, addr 0x3C)", "bus 0, shared with the BMP280 inside the case", oledR)

ringL = [("5V", "5V", "5V   (board 5V pin)"), ("GND", "GND", "GND"), ("DIN", "DATA", "DATA  GPIO 47   (into DIN, not DOUT)")]
ringR = [("5V", "5V", "5V   (chain from the left ring)"), ("GND", "GND", "GND"), ("DIN", "DATA", "DATA  GPIO 21   (into DIN, not DOUT)")]
pad_list(60, 900, "LED ring, LEFT  (24 x WS2812B)", "pixel 0 and direction are set in firmware (RING_OFFSET)", ringL)
pad_list(1540, 900, "LED ring, RIGHT  (24 x WS2812B)", "DIN is the pad the arrows point AWAY from", ringR)

encL = [("VCC / +", "3V3", "3V3  (NOT 5 V)"), ("GND", "GND", "GND"), ("S1A / CLK", "ENC", "GPIO 15"), ("S1B / DT", "ENC", "GPIO 17"), ("SW", "ENC", "GPIO 18")]
encR = [("VCC / +", "3V3", "3V3  (NOT 5 V)"), ("GND", "GND", "GND"), ("S1A / CLK", "ENC", "GPIO 4"), ("S1B / DT", "ENC", "GPIO 5"), ("SW", "ENC", "GPIO 6")]
pad_list(480, 900, "Encoder, LEFT knob  (KY-040)", None, encL)
pad_list(1120, 900, "Encoder, RIGHT knob  (KY-040)", None, encR)

# ---- harness: the wires that leave the panel
hx0, hy0 = 40, 1160
d.rounded_rectangle((hx0, hy0, 1960, 1620), 18, fill="#eef4ee", outline="#333", width=3)
d.text((hx0 + 16, hy0 + 10), "HARNESS: 21 wires leave the panel. Label both ends. One wire per row.", font=FB(20), fill="black")
rows = [
    ("5V",   "board 5V",   "both rings (chain on the panel)",                         "red"),
    ("3V3",  "board 3V3",  "both TFTs, both OLEDs, both encoders (chain on the panel)", "orange"),
    ("GND",  "board GND",  "everything (one net; chain on the panel)",                 "black"),
    ("SCK",  "GPIO 12",    "both TFTs (SCL pads)",                                    "blue"),
    ("MOSI", "GPIO 11",    "both TFTs (SDA pads)",                                    "light blue"),
    ("DC",   "GPIO 38",    "both TFTs",                                               "purple"),
    ("RST",  "GPIO 14",    "both TFTs (RES pads)",                                    "brown"),
    ("CS",   "GPIO 16",    "LEFT TFT only",                                           "pink"),
    ("CS",   "GPIO 41",    "RIGHT TFT only",                                          "pink + mark"),
    ("SDA",  "GPIO 1",     "LEFT OLED",                                               "green"),
    ("SCL",  "GPIO 2",     "LEFT OLED",                                               "dark green"),
    ("SDA",  "GPIO 8",     "RIGHT OLED",                                              "green + mark"),
    ("SCL",  "GPIO 9",     "RIGHT OLED",                                              "dark green + mark"),
    ("DATA", "GPIO 47",    "LEFT ring DIN",                                           "yellow"),
    ("DATA", "GPIO 21",    "RIGHT ring DIN",                                          "yellow + mark"),
    ("ENC",  "GPIO 15 / 17 / 18",    "LEFT knob CLK / DT / SW",                          "grey x3"),
    ("ENC",  "GPIO 4 / 5 / 6", "RIGHT knob CLK / DT / SW",                         "grey x3 + mark"),
]
cols = [hx0 + 20, hx0 + 120, hx0 + 320, hx0 + 820]
d.text((cols[0], hy0 + 44), "net", font=FB(15), fill="#333"); d.text((cols[1], hy0 + 44), "board end", font=FB(15), fill="#333")
d.text((cols[2], hy0 + 44), "panel end", font=FB(15), fill="#333"); d.text((cols[3], hy0 + 44), "suggested colour", font=FB(15), fill="#333")
for i, (net, bend, pend, col) in enumerate(rows):
    yy = hy0 + 70 + i * 21
    d.ellipse((cols[0] - 2, yy + 2, cols[0] + 10, yy + 14), fill=C[net])
    d.text((cols[0] + 18, yy), net, font=FB(14), fill="black"); d.text((cols[1], yy), bend, font=F(14), fill="black")
    d.text((cols[2], yy), pend, font=F(14), fill="black"); d.text((cols[3], yy), col, font=F(14), fill="#555")
# notes column
nx = hx0 + 1080
notes = ["Soldering order that avoids the faults we had:",
         "1. Power first: 3V3 + GND chain to both TFTs, both OLEDs, both encoders; 5V + GND to both rings.",
         "2. TFT bus chain: SCK, MOSI, DC, RST from the left TFT pads to the right TFT pads (short links).",
         "3. The two CS wires: 16 to the LEFT TFT, 41 to the RIGHT TFT. A loose CS = blank dial or the",
         "   other dial's picture flickering through (that was the 12 Oct fault).",
         "4. OLEDs: left on 1 / 2, right on 8 / 9 (with the BMP280). Keep them on separate buses: both are 0x3C.",
         "5. Ring data into DIN (arrows point away from DIN). A dark ring = DIN, 5V or GND on that ring.",
         "6. Encoders last. Tape every joint; strain-relieve the harness where it leaves the panel.",
         "",
         "Test after each step with the device on: the web page (deskradar6.local) shows both dials and",
         "both OLEDs live; deskradar6.local/i2c lists 0x3C on bus 0 and 0x3C on bus 1."]
for i, t in enumerate(notes): d.text((nx, hy0 + 44 + i * 24), t, font=FB(15) if i == 0 else F(15), fill="#222")

os.makedirs(os.path.dirname(OUT), exist_ok=True)
img.save(OUT); print("wrote", OUT)
