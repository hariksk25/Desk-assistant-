#!/usr/bin/env python3
"""Photo-style renders of the v6 sub-dial case with the real components (z-buffer rasterizer, no GPU needed).
Writes renders/final_hero.png, renders/final_front.png, renders/final_exploded.png. Run after make_case_v6.py."""
import os, sys, numpy as np, trimesh
from PIL import Image, ImageDraw, ImageFont, ImageFilter
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import make_case_v6 as C
STL = os.path.join(HERE, "stl"); OUT = os.path.join(HERE, "renders"); os.makedirs(OUT, exist_ok=True)
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"; FONTB = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
def F(sz, b=False): return ImageFont.truetype(FONTB if b else FONT, sz)

def load(n): return trimesh.load(os.path.join(STL, f"v6_{n}.stl"))
def T(m, t=(0, 0, 0), R=None):
    m = m.copy()
    if R is not None: m.apply_transform(R)
    m.apply_translation(t); return m
def cylm(r, z0, z1, x, y, s=96):
    c = trimesh.creation.cylinder(radius=r, height=z1 - z0, sections=s); c.apply_translation((x, y, (z0 + z1) / 2)); return c
def boxm(sx, sy, z0, z1, x, y):
    b = trimesh.creation.box((sx, sy, z1 - z0)); b.apply_translation((x, y, (z0 + z1) / 2)); return b
def annm(od, idr, z0, z1, x, y):
    return cylm(od / 2, z0, z1, x, y, 128).difference(cylm(idr / 2, z0 - 1, z1 + 1, x, y, 128), engine="manifold")

# ---------------------------------------------------------------- parts (assembly coordinates)
WHITE, BLACK, BEZEL, BODY, RED = (236, 234, 226), (34, 35, 39), (44, 45, 50), (38, 39, 44), (200, 38, 38)
PCB_G, PCB_B, GLASS = (24, 110, 60), (25, 70, 150), (12, 14, 18)
def case_parts(explode=0.0, body=True):
    e = explode
    parts = ([(T(load("body"), (0, 0, -6 * e)), BODY)] if body else []) + [(T(load("carrier_white"), (0, 0, 0)), WHITE),
             (T(load("cover_black"), (0, 0, 3 * e)), BLACK)]
    bz = load("bezel_ring_black")
    for (cx, cy) in (C.POD1, C.POD2): parts.append((T(bz, (cx, cy, C.PLATE_T + 4 * e)), BEZEL))
    if getattr(C, 'BEZEL_PIP', None):
        pp = load('pip_red')
        for (cx, cy) in (C.POD1, C.POD2): parts.append((T(pp, (cx, cy + C.BEZEL_PIP[2], C.PLATE_T + 4 * e + 0.01)), RED))
    if not body: return parts
    kn = load("knob_red"); Rk = trimesh.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0])   # knob axis z -> +y
    for kx in C.KNOB_X: parts.append((T(kn, (kx, C.H + 1.5 + 2 * e, C.KNOB_Z - 6 * e), Rk), RED))
    return parts
def component_parts(explode=0.0):
    e = explode; P = []
    zr0, zr1 = C.RING_SEAT_Z - C.RING_PCB_T, C.RING_SEAT_Z
    for (cx, cy) in (C.POD1, C.POD2):                                    # LED rings
        dz = -2 * e
        P.append((annm(C.RING_OD, C.RING_ID, zr0 + dz, zr1 + dz, cx, cy), PCB_G))
        for k in range(24):
            a = 2 * np.pi * k / 24; r = (C.RING_OD + C.RING_ID) / 4
            P.append((boxm(5, 5, zr1 + dz, zr1 + C.RING_LED_H + dz, cx + r * np.sin(a), cy + r * np.cos(a)), (245, 245, 240)))
    for (dx, dy) in C.DISP:                                              # round TFT modules
        dz = -1 * e
        zp = C.GLASS_POCKET_T - C.TFT_FRONT_T
        P.append((cylm(C.TFT_MOD_R, zp - C.TFT_PCB_T + dz, zp + dz, dx, dy), PCB_B))
        P.append((boxm(C.TFT_TAB[0], C.TFT_TAB[1] - 14.0, zp - C.TFT_PCB_T + dz, zp + dz, dx, dy + (C.TFT_TAB[1] + 14.0) / 2), PCB_B))   # tab
        P.append((boxm(17.8, 2.5, zp - C.TFT_PCB_T - 2.5 + dz, zp - C.TFT_PCB_T + dz, dx, dy + C.TFT_TAB[1] - 1.76), (20, 20, 22)))      # header
        P.append((cylm(C.TFT_BL[0] / 2, zp + dz, C.GLASS_POCKET_T + dz, dx, dy), GLASS))
    for (ox, oy), (sx, sy) in zip(C.OLEDS, C.OLED_GLASS):                  # 0.96" OLEDs
        dz = -3 * e
        P.append((boxm(*C.OLED91_BOARD[:2], zr0 - C.OLED91_BOARD[2] + dz, zr0 + dz, ox, oy), PCB_B))
        P.append((boxm(*C.OLED91_SCREEN, zr0 + dz, zr0 + C.OLED91_GLASS_T + dz, sx, sy), GLASS))
    return P

# ---------------------------------------------------------------- z-buffer rasterizer (orthographic)
def rot(ax, deg):
    a = np.deg2rad(deg); c, s = np.cos(a), np.sin(a)
    return {"x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]), "y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[ax]
class Cam:
    def __init__(self, Rm, W, H, scale, centre): self.R, self.W, self.H, self.s, self.c = Rm, W, H, scale, np.asarray(centre)
    def proj(self, p):
        v = (np.asarray(p) - self.c) @ self.R.T
        return np.stack([self.W / 2 + v[..., 0] * self.s, self.H / 2 - v[..., 1] * self.s, v[..., 2]], -1)
def raster(cam, items, bg=(20, 22, 26), glow=None):
    W, H = cam.W, cam.H
    zb = np.full((H, W), -1e9); img = np.zeros((H, W, 3)); img[:] = bg
    L1 = np.array([-0.35, 0.55, 0.75]); L1 /= np.linalg.norm(L1); L2 = np.array([0.6, 0.2, 0.5]); L2 /= np.linalg.norm(L2)
    for mesh, col in items:
        v = cam.proj(mesh.vertices); tri = v[mesh.faces]
        nw = mesh.face_normals @ cam.R.T
        sh = 0.30 + 0.55 * np.clip(nw @ L1, 0, 1) + 0.25 * np.clip(nw @ L2, 0, 1)
        cols = np.tile(np.asarray(col, float), (len(tri), 1))
        emis = np.zeros(len(tri), bool)
        if glow is not None and col == WHITE: cols, emis = glow(mesh, cols)
        for i in np.where(nw[:, 2] > 0.0)[0]:
            t = tri[i]; x0, x1 = int(max(0, np.floor(t[:, 0].min()))), int(min(W - 1, np.ceil(t[:, 0].max())))
            y0, y1 = int(max(0, np.floor(t[:, 1].min()))), int(min(H - 1, np.ceil(t[:, 1].max())))
            if x1 < x0 or y1 < y0: continue
            xs, ys = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
            (ax, ay, az), (bx_, by, bz), (cx, cy, cz) = t
            d = (by - cy) * (ax - cx) + (cx - bx_) * (ay - cy)
            if abs(d) < 1e-9: continue
            w1 = ((by - cy) * (xs - cx) + (cx - bx_) * (ys - cy)) / d; w2 = ((cy - ay) * (xs - cx) + (ax - cx) * (ys - cy)) / d; w3 = 1 - w1 - w2
            m = (w1 >= -1e-6) & (w2 >= -1e-6) & (w3 >= -1e-6)
            if not m.any(): continue
            z = w1 * az + w2 * bz + w3 * cz; sub = zb[y0:y1 + 1, x0:x1 + 1]; m &= z > sub
            sub[m] = z[m]; img[y0:y1 + 1, x0:x1 + 1][m] = np.clip(cols[i] * (1.0 if emis[i] else sh[i]), 0, 255)
    return img.astype(np.uint8), zb

# LED glow on the diffuser band: faces of the white carrier front skin inside the glow annulus get lit
GLOW_COLS = [(255, 170, 60)] * 24
def make_glow(cols24):
    def g(mesh, cols):
        c = mesh.triangles_center; out = cols.copy(); em = np.zeros(len(cols), bool)
        for (px, py) in (C.POD1, C.POD2):
            dx, dy = c[:, 0] - px, c[:, 1] - py; r = np.hypot(dx, dy)
            m = (r > C.BEZEL_OD / 2 - 1) & (r < C.GLOW_OD / 2 + 0.5) & (c[:, 2] > C.PLATE_T - 0.05)
            ang = (np.degrees(np.arctan2(dx, dy)) + 360) % 360
            k = np.round(ang / 15).astype(int) % 24; off = np.abs(ang - np.round(ang / 15) * 15) / 7.5   # 0 at an LED, 1 between
            base = np.array([cols24[j] for j in k[m]], float); inten = 0.75 + 0.25 * (1 - off[m])
            out[m] = np.clip(0.30 * 255 + 0.80 * base * inten[:, None], 0, 255); em |= m
        return out, em
    return g

# ---------------------------------------------------------------- screen textures, pasted onto their apertures (orthographic => affine)
def radar_tex(n=520):
    im = Image.new("RGB", (n, n), (6, 14, 24)); d = ImageDraw.Draw(im); c = n / 2
    for r in (0.92, 0.66, 0.40, 0.14): d.ellipse([c - r * c, c - r * c, c + r * c, c + r * c], outline=(30, 120, 80), width=3)
    d.line([c, 0, c, n], fill=(24, 80, 56), width=2); d.line([0, c, n, c], fill=(24, 80, 56), width=2)
    for k in range(40):                                                   # sweep wedge
        a0 = -60 + k * 1.5; d.pieslice([c - 0.92 * c, c - 0.92 * c, c + 0.92 * c, c + 0.92 * c], a0, a0 + 1.6, fill=(10, int(30 + k * 3), int(20 + k * 1.6)))
    for (x, y, lab) in ((0.30, -0.35, "IGO6E"), (-0.45, 0.20, "AIC804"), (0.55, 0.42, "QP1372")):
        px, py = c + x * c, c + y * c; d.polygon([(px, py - 11), (px - 8, py + 8), (px + 8, py + 8)], fill=(255, 210, 70)); d.text((px + 12, py - 8), lab, font=F(22, True), fill=(230, 230, 230))
    d.ellipse([c - 6, c - 6, c + 6, c + 6], fill=(255, 80, 80))
    d.text((c, n * 0.80), "3 FLIGHTS  12 km", font=F(26, True), fill=(140, 220, 170), anchor="mm")
    return im
def clock_tex(n=520):
    im = Image.new("RGB", (n, n), (8, 8, 12)); d = ImageDraw.Draw(im); c = n / 2
    for k in range(60):
        a = np.deg2rad(k * 6); r0 = 0.86 if k % 5 else 0.78
        d.line([c + r0 * c * np.sin(a), c - r0 * c * np.cos(a), c + 0.92 * c * np.sin(a), c - 0.92 * c * np.cos(a)], fill=(230, 120, 40) if k < 41 else (70, 70, 80), width=4 if k % 5 == 0 else 2)
    d.text((c, c - 18), "24:17", font=F(108, True), fill=(250, 250, 250), anchor="mm")
    d.text((c, c + 70), "FOCUS  2 / 4", font=F(30, True), fill=(250, 170, 70), anchor="mm")
    return im
def oled_tex(lines, w=420, h=220):
    im = Image.new("RGB", (w, h), (0, 0, 0)); d = ImageDraw.Draw(im)
    d.text((w / 2, h * 0.36), lines[0], font=F(64, True), fill=(120, 190, 255), anchor="mm")           # blue rows
    d.text((w / 2, h * 0.87), lines[1], font=F(40, True), fill=(255, 214, 60), anchor="mm"); return im   # yellow band, bottom 16 rows (OLED mounted upside down)
def paste_plane(img, cam, tex, centre, size, z, mask="rect"):
    """Paste tex onto the plane rectangle (centre, size) at height z. Orthographic projection of a plane is affine."""
    (x, y), (w, h) = centre, size
    P = cam.proj(np.array([[x - w / 2, y + h / 2, z], [x + w / 2, y + h / 2, z], [x - w / 2, y - h / 2, z]]))[:, :2]
    tw, th = tex.size; A = np.array([[(P[1] - P[0])[0] / tw, (P[2] - P[0])[0] / th, P[0][0]], [(P[1] - P[0])[1] / tw, (P[2] - P[0])[1] / th, P[0][1]], [0, 0, 1]])
    Ai = np.linalg.inv(A); coeffs = tuple(Ai[:2].ravel())
    warped = tex.transform(img.size, Image.AFFINE, coeffs, resample=Image.BICUBIC)
    m = Image.new("L", tex.size, 0); dm = ImageDraw.Draw(m)
    (dm.ellipse if mask == "circle" else dm.rectangle)([0, 0, tw - 1, th - 1], fill=255)
    img.paste(warped, (0, 0), m.transform(img.size, Image.AFFINE, coeffs, resample=Image.BILINEAR))

def glow_tex(cols24, n=600):
    """LED glow seen through the 1 mm white skin: RGBA texture over the square (GLOW_OD) centred on the pod."""
    im = Image.new("RGB", (n, n), (0, 0, 0)); d = ImageDraw.Draw(im); c = n / 2; s = n / C.GLOW_OD
    r = (C.RING_OD + C.RING_ID) / 4 * s
    for k in range(24):
        a = 2 * np.pi * k / 24; x, y = c + r * np.sin(a), c - r * np.cos(a); col = cols24[k]
        d.ellipse([x - 16, y - 16, x + 16, y + 16], fill=col)
    im = im.filter(ImageFilter.GaussianBlur(9))
    a = np.asarray(im, float); a = 0.55 * 240 + 0.45 * np.clip(a * 2.2, 0, 255)      # white skin, lit from behind
    hot = Image.new("L", (n, n), 0); dh = ImageDraw.Draw(hot)
    for k in range(24):
        ang = 2 * np.pi * k / 24; x, y = c + r * np.sin(ang), c - r * np.cos(ang); dh.ellipse([x - 7, y - 7, x + 7, y + 7], fill=255)
    hot = np.asarray(hot.filter(ImageFilter.GaussianBlur(5)), float)[..., None] / 255
    a = a * (1 - 0.6 * hot) + 255 * 0.6 * hot
    return Image.fromarray(np.clip(a, 0, 255).astype(np.uint8))
def paste_depth(img, zb, cam, tex, centre, size, z, tol=0.25, shape="ring"):
    """Like paste_plane, but only where the z-buffer shows that plane (keeps occlusion by the cover and bezel)."""
    (x, y), (w, h) = centre, size
    P = cam.proj(np.array([[x - w / 2, y + h / 2, z], [x + w / 2, y + h / 2, z], [x - w / 2, y - h / 2, z]]))
    tw, th = tex.size; A = np.array([[(P[1] - P[0])[0] / tw, (P[2] - P[0])[0] / th, P[0][0]], [(P[1] - P[0])[1] / tw, (P[2] - P[0])[1] / th, P[0][1]], [0, 0, 1]])
    Ai = np.linalg.inv(A); coeffs = tuple(Ai[:2].ravel())
    warped = tex.transform(img.size, Image.AFFINE, coeffs, resample=Image.BICUBIC)
    H, W = zb.shape; ys, xs = np.mgrid[0:H, 0:W] + 0.5
    u = Ai[0, 0] * xs + Ai[0, 1] * ys + Ai[0, 2]; v = Ai[1, 0] * xs + Ai[1, 1] * ys + Ai[1, 2]     # texture coords
    zp = P[0][2] + (P[1][2] - P[0][2]) * u / tw + (P[2][2] - P[0][2]) * v / th                      # plane depth per pixel
    rr = np.hypot(u / tw - 0.5, v / th - 0.5) * w
    m = np.abs(zb - zp) < tol
    if shape == "ring": m &= (rr > C.BEZEL_OD / 2 - 0.2) & (rr < C.GLOW_OD / 2 + 0.2)
    elif shape == "circle": m &= rr < w / 2
    else: m &= (u >= 0) & (u < tw) & (v >= 0) & (v < th)
    img.paste(warped, (0, 0), Image.fromarray((m * 255).astype(np.uint8)))
def scene(cam, explode=0.0, comps=False, screens=True, bg=(20, 22, 26), body=True):
    items = case_parts(explode, body) + (component_parts(explode) if comps else [])
    arr, zb = raster(cam, items, bg=bg)
    im = Image.fromarray(arr)
    leds = [[(255, 150, 40)] * 16 + [(60, 60, 70)] * 8, [(70, 190, 255)] * 24]
    for k, (cx, cy) in enumerate((C.POD1, C.POD2)):
        if explode == 0: paste_depth(im, zb, cam, glow_tex(leds[k]), (cx, cy), (C.GLOW_OD, C.GLOW_OD), C.PLATE_T)
    tex = [radar_tex(), clock_tex()]; ol = [("3 FLT", "IGO6E 12km"), ("FOCUS", "ends 11:40")]
    if screens and explode:                       # exploded: screens sit on their own glass, depth tested
        zr0 = C.RING_SEAT_Z - C.RING_PCB_T
        for k, (dx, dy) in enumerate(C.DISP): paste_depth(im, zb, cam, tex[k], (dx, dy), (C.TFT_WIN_D, C.TFT_WIN_D), C.GLASS_POCKET_T - 1 * explode, shape="circle")
        for k, (sx, sy) in enumerate(C.OLED_SCR): paste_depth(im, zb, cam, oled_tex(ol[k]), (sx, sy), C.OLED91_WIN, zr0 + C.OLED91_GLASS_T - 3 * explode, shape="rect")
    elif screens:
        for k, (dx, dy) in enumerate(C.DISP): paste_plane(im, cam, tex[k], (dx, dy), (C.TFT_WIN_D, C.TFT_WIN_D), C.PLATE_T + 4 * explode, "circle")
        for k, (sx, sy) in enumerate(C.OLED_SCR): paste_plane(im, cam, oled_tex(ol[k]), (sx, sy), C.OLED91_WIN, C.PLATE_T + C.COVER_T + 7 * explode)
    return im

def finish(im, ss, title, sub=None, dark=False):
    im = im.resize((im.width // ss, im.height // ss), Image.LANCZOS); d = ImageDraw.Draw(im)
    d.text((28, 22), title, font=F(30, True), fill=(30, 32, 36) if dark else (240, 240, 240))
    if sub: d.text((28, 62), sub, font=F(18), fill=(70, 74, 82) if dark else (170, 175, 185))
    return im

if __name__ == "__main__":
    SS = 2; ctr = (90, 50, -16)
    tilt = rot("x", -C.TILT)                                               # stands on the desk: front leans back 30 degrees
    # hero: three quarter, slightly above
    cam = Cam(rot("x", 18) @ rot("y", 28) @ tilt, 1600 * SS, 1100 * SS, 6.2 * SS, ctr)
    finish(scene(cam), SS, "Desk Plane Radar v6  ·  twin sub-dial", "183 x 94 mm figure-8 body wraps the front unit, 30 degree lean, USB-C on the left side").save(os.path.join(OUT, "final_hero.png"))
    # straight front (face plane)
    cam = Cam(np.eye(3), 1600 * SS, 1100 * SS, 7.5 * SS, (90, 62, 0))
    finish(scene(cam), SS, "Front, face on", "Round TFT dead centre, 0.96 in OLED inside the ring at 6 o'clock").save(os.path.join(OUT, "final_front.png"))
    # exploded from the back (how it is assembled: face down), body lifted off
    cam = Cam(rot("x", 30) @ rot("y", 180 - 42), 1600 * SS, 1200 * SS, 5.2 * SS, (90, 50, -22))
    im = scene(cam, explode=13.0, comps=True, screens=False, body=False)
    im = finish(im, SS, "Assembly layers, seen from the back (body lifted off)", "white carrier with the pits  ->  round TFT  ->  LED ring (1.2 mm PCB)  ->  0.96 in OLED on the ring back")
    im.save(os.path.join(OUT, "final_exploded.png"))
    # assembled back, no body: everything seated
    cam = Cam(rot("x", 32) @ rot("y", 180 - 30), 1600 * SS, 1100 * SS, 6.4 * SS, (90, 50, -2))
    finish(scene(cam, comps=True, screens=False, body=False), SS, "Seated, seen from the back (body off)", "ring back flush with the carrier floor, OLED board lying across the ring, TFT in its pocket").save(os.path.join(OUT, "final_back_seated.png"))
    # rear three quarter, on the desk: USB-C socket in the back wall
    cam = Cam(rot("x", 22) @ rot("y", 180 + 30) @ tilt, 1600 * SS, 1100 * SS, 6.2 * SS, ctr)
    finish(scene(cam, screens=False), SS, "Rear, on the desk", "Speaker grille behind the right dial (away from the USB side), vents over the S3").save(os.path.join(OUT, "final_rear.png"))
    # side, low: the body wraps the carrier edge, only the front glow ring is white
    cam = Cam(rot("x", 8) @ rot("y", 62) @ tilt, 1600 * SS, 1100 * SS, 6.0 * SS, ctr)
    finish(scene(cam), SS, "Side view", "Body sleeve runs up to the cover front: no white carrier edge from the side").save(os.path.join(OUT, "final_side.png"))
    # body alone, front three quarter: the pocket the front unit drops into
    cam = Cam(rot("x", 35) @ rot("y", 25), 1600 * SS, 1100 * SS, 6.0 * SS, (90, 55, -16))
    arr, _ = raster(cam, [(load("body"), BODY)], bg=(205, 208, 214))
    finish(Image.fromarray(arr), SS, "Back case (body)", "1.2 mm sleeve, 5 mm pocket, 2 waist lugs for the front M2 screws, 4 x M3 from the back", dark=True).save(os.path.join(OUT, "final_body.png"))
    print("ok")
