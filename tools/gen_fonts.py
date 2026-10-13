#!/usr/bin/env python3
"""Generate Adafruit-GFX-format (GFXfont) 1-bit fonts from TrueType files.
Output: C header text with PROGMEM arrays, plus a preview PNG."""
from PIL import Image, ImageDraw, ImageFont
import sys

G = "/usr/share/fonts/truetype/google-fonts/"
D = "/usr/share/fonts/truetype/dejavu/"
C = "/usr/share/fonts/truetype/crosextra/"

FONTS = [
    # name,            file,                      px,  chars,                 threshold
    ("F_BIG104", G + "Poppins-Bold.ttf",    104, " 0123456789",                    110),
    ("F_BIG60",  G + "Poppins-Bold.ttf",     60, " 0123456789:.,-+%\x7f",          110),
    ("F_BIG36",  G + "Poppins-Bold.ttf",     36, " 0123456789:.,-+%\x7f",          110),
    ("F_MID22",  G + "Poppins-Medium.ttf",   22, [chr(c) for c in range(32, 128)], 110),
    ("F_MID22B", G + "Poppins-Bold.ttf",     22, [chr(c) for c in range(32, 128)], 110),
    ("F_SM13",   D + "DejaVuSansCondensed-Bold.ttf", 13, [chr(c) for c in range(32, 128)], 96),
    ("F_SM13R",  D + "DejaVuSansCondensed.ttf",      13, [chr(c) for c in range(32, 128)], 90),
]

def render_glyph(font, ch, thr):
    if ch == "\x7f": ch = "\u00b0"   # code 127 = degree sign
    # baseline-anchored bbox
    bbox = font.getbbox(ch, anchor="ls")
    x0, y0, x1, y1 = bbox
    w, h = x1 - x0, y1 - y0
    adv = int(round(font.getlength(ch)))
    if w <= 0 or h <= 0 or ch == " ":
        return (b"", 0, 0, adv, 0, 0, None)
    im = Image.new("L", (w, h), 0)
    ImageDraw.Draw(im).text((-x0, -y0), ch, font=font, fill=255, anchor="ls")
    px = im.load()
    bits = []
    for yy in range(h):
        for xx in range(w):
            bits.append(1 if px[xx, yy] >= thr else 0)
    # trim empty rows/cols (threshold may shrink the box)
    rows = [any(bits[yy*w:(yy+1)*w]) for yy in range(h)]
    cols = [any(bits[yy*w+xx] for yy in range(h)) for xx in range(w)]
    if not any(rows):
        return (b"", 0, 0, adv, 0, 0, None)
    ry0, ry1 = rows.index(True), len(rows) - 1 - rows[::-1].index(True)
    cx0, cx1 = cols.index(True), len(cols) - 1 - cols[::-1].index(True)
    nw, nh = cx1 - cx0 + 1, ry1 - ry0 + 1
    nbits = [bits[yy*w+xx] for yy in range(ry0, ry1+1) for xx in range(cx0, cx1+1)]
    data = bytearray()
    acc = 0; n = 0
    for b in nbits:
        acc = (acc << 1) | b; n += 1
        if n == 8: data.append(acc); acc = 0; n = 0
    if n: data.append(acc << (8 - n))
    xoff = x0 + cx0
    yoff = y0 + ry0   # negative = above baseline
    return (bytes(data), nw, nh, adv, xoff, yoff, (nbits, nw, nh))

def build(name, path, px, chars, thr):
    font = ImageFont.truetype(path, px)
    chars = sorted(set(chars), key=ord)
    first, last = ord(chars[0]), ord(chars[-1])
    bitmaps = bytearray(); glyphs = []
    previews = {}
    for code in range(first, last + 1):
        ch = chr(code)
        if ch in chars:
            data, w, h, adv, xo, yo, pv = render_glyph(font, ch, thr)
        else:
            data, w, h, adv, xo, yo, pv = b"", 0, 0, 0, 0, 0, None
        glyphs.append((len(bitmaps), w, h, adv, xo, yo, ch))
        bitmaps += data
        if pv: previews[ch] = pv
    asc, desc = font.getmetrics()
    yadv = int(round((asc + desc) * 1.0))
    out = []
    out.append(f"// {name}: {path.split('/')[-1]} {px}px, {len(chars)} glyphs, {len(bitmaps)} bytes")
    out.append(f"const uint8_t {name}_Bitmaps[] PROGMEM = {{")
    for i in range(0, len(bitmaps), 24):
        out.append("  " + ", ".join(f"0x{b:02X}" for b in bitmaps[i:i+24]) + ",")
    out.append("};")
    out.append(f"const GFXglyph {name}_Glyphs[] PROGMEM = {{")
    for (off, w, h, adv, xo, yo, ch) in glyphs:
        cm = ch if ch.isprintable() and ch not in "\\" and ch != "\x7f" else ("deg" if ch == "\x7f" else "?")
        out.append(f"  {{{off:5d}, {w:3d}, {h:3d}, {adv:3d}, {xo:4d}, {yo:4d}}},   // '{cm}'")
    out.append("};")
    out.append(f"const GFXfont {name} PROGMEM = {{(uint8_t *){name}_Bitmaps, (GFXglyph *){name}_Glyphs, 0x{first:02X}, 0x{last:02X}, {yadv}}};")
    out.append("")
    return "\n".join(out), glyphs, previews, len(bitmaps)

def preview(all_fonts, path):
    S = 2
    rows = []
    for name, glyphs, pv in all_fonts:
        sample = "14:48 25,412.30" if "BIG" in name else "AI504 DEL>BLR 27C Partly cloudy FL350 09/27 abc xyz"
        x = 0; imgs = []
        for ch in sample:
            g = [g for g in glyphs if g[6] == ch]
            if not g: continue
            off, w, h, adv, xo, yo, _ = g[0]
            imgs.append((x, ch, g[0], pv.get(ch)))
            x += adv
        rows.append((name, x, imgs))
    H = sum(80 if "BIG6" in n else 50 if "BIG3" in n else 32 for n, _, _ in rows)
    W = max(x for _, x, _ in rows) + 10
    im = Image.new("L", (W, H + 10), 0); px = im.load()
    y = 5
    for name, _, imgs in rows:
        base = y + (64 if "BIG6" in name else 40 if "BIG3" in name else 24)
        for x, ch, g, pv in imgs:
            if not pv: continue
            bits, w, h = pv
            for yy in range(h):
                for xx in range(w):
                    if bits[yy*w+xx]:
                        X, Y = x + g[4] + xx, base + g[5] + yy
                        if 0 <= X < W and 0 <= Y < H + 10: px[X, Y] = 255
        y += 80 if "BIG6" in name else 50 if "BIG3" in name else 32
    im = im.resize((W*S, (H+10)*S), Image.NEAREST)
    im.save(path)

if __name__ == "__main__":
    hdr = ["// ---------------------------------------------------------------------",
           "// FONTS (generated by gen_fonts.py from Poppins / DejaVu, 1-bit GFXfont format)",
           "// ---------------------------------------------------------------------"]
    allf = []; total = 0
    for name, path, px, chars, thr in FONTS:
        txt, glyphs, pv, nbytes = build(name, path, px, chars, thr)
        hdr.append(txt); allf.append((name, glyphs, pv)); total += nbytes
        print(name, nbytes, "bytes")
    open("fonts.h", "w").write("\n".join(hdr))
    preview(allf, "fonts_preview.png")
    print("total bitmap bytes", total)
