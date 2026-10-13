"""Airline logos for the v6 right dial, as files for the microSD card.

Reads the IATA codes from the AIRLINES table in the v6 sketch, downloads each logo once from pics.avs.io
(transparent PNG, cached in assets/airline_logos/), trims the transparent margin, fits it into LOGO_W x LOGO_H
and lifts dark parts toward white so they stay visible on the black dial (owner, 12 Oct: no white card).
Writes one file per airline to sd_card/deskradar/logos/<IATA>.lgo and assets/airline_logos/preview_dark.png.

.lgo format (little endian): "LGO1", uint16 w, uint16 h, w*h uint16 RGB565, w*h uint8 alpha.
Run from the repo root: python3 tools/make_airline_logos.py
Upload to the device: tools/sd_upload.sh sd_card/deskradar/logos (or copy the folder onto the card).
"""
import os, re, struct, urllib.request
from PIL import Image

SKETCH = "firmware/desk_plane_radar_v6/desk_plane_radar_v6.ino"
OUT_DIR = "sd_card/deskradar/logos"
CACHE = "assets/airline_logos"
LOGO_W, LOGO_H = 150, 40
URL = "https://pics.avs.io/240/80/{}.png"
EXCLUDE = {"BZ"}   # pics.avs.io returns BlueBird Airways for BZ, not Blue Dart: keep the colour badge
DARK = 110         # luminance below this is lifted toward white (navy / black lettering)

src = open(SKETCH).read()
table = src[src.index("const AirlineRow AIRLINES[]"):src.index("#define NUM_AIRLINES")]
codes = []
for icao, iata, name in re.findall(r'\{"(\w*)","(\w*)","([^"]*)",0x[0-9A-Fa-f]+\}', table):
    if iata and iata not in codes and iata not in EXCLUDE:
        codes.append(iata)
os.makedirs(CACHE, exist_ok=True); os.makedirs(OUT_DIR, exist_ok=True)

def fetch(code):
    path = os.path.join(CACHE, code + ".png")
    if not os.path.exists(path):
        req = urllib.request.Request(URL.format(code), headers={"User-Agent": "DeskRadar logo builder"})
        try:
            open(path, "wb").write(urllib.request.urlopen(req, timeout=15).read())
        except Exception as e:
            print(f"{code}: download failed ({e})"); return None
    try:
        return Image.open(path).convert("RGBA")
    except Exception as e:
        print(f"{code}: not an image ({e})"); return None

def lift(im):
    """dark opaque pixels toward white, smoothly, so navy text reads on black; colours keep their hue"""
    px = im.load()
    for y in range(im.height):
        for x in range(im.width):
            r, g, b, a = px[x, y]
            if not a: continue
            if r > 110 and r > 1.5 * g and r > 1.5 * b: continue   # reds / oranges already read well on black
            L = 0.299 * r + 0.587 * g + 0.114 * b
            if L < DARK:
                t = (DARK - L) / DARK                       # 1 for black, 0 at the threshold
                k = 0.35 + 0.65 * t                         # how far toward white
                px[x, y] = (int(r + (255 - r) * k), int(g + (255 - g) * k), int(b + (255 - b) * k), a)
    return im

def fit(im):
    bbox = im.getchannel("A").point(lambda v: 255 if v > 24 else 0).getbbox()
    if not bbox: return None
    im = im.crop(bbox)
    k = min(LOGO_W / im.width, LOGO_H / im.height)
    im = im.resize((max(1, round(im.width * k)), max(1, round(im.height * k))), Image.LANCZOS)
    return lift(im)

def rgb565(r, g, b): return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)

logos = []
for code in codes:
    im = fetch(code)
    im = fit(im) if im else None
    if im is None: print(f"{code}: skipped"); continue
    w, h = im.size; data = list(im.getdata())
    blob = b"LGO1" + struct.pack("<HH", w, h)
    blob += struct.pack("<%dH" % (w * h), *[rgb565(r, g, b) for r, g, b, a in data])
    blob += bytes(a for r, g, b, a in data)
    open(os.path.join(OUT_DIR, code + ".lgo"), "wb").write(blob)
    logos.append((code, im, len(blob)))

cols = 4
sheet = Image.new("RGB", (cols * (LOGO_W + 20) + 10, ((len(logos) + cols - 1) // cols) * (LOGO_H + 20) + 10), (0, 0, 0))
for i, (c, im, n) in enumerate(logos):
    x, y = 10 + (i % cols) * (LOGO_W + 20), 10 + (i // cols) * (LOGO_H + 20)
    sheet.paste(im, (x + (LOGO_W - im.width) // 2, y + (LOGO_H - im.height) // 2), im)
sheet.save(os.path.join(CACHE, "preview_dark.png"))
print(f"{len(logos)} of {len(codes)} airlines, {sum(n for _, _, n in logos) // 1024} KB -> {OUT_DIR}/")
