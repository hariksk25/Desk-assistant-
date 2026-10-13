#!/usr/bin/env python3
"""Checks the generated v6 STLs: watertight, fits the A1 Mini bed (rotated 45 deg), and component envelopes
(LED rings, OLED boards, ESP32-S3) do not collide with the printed parts. Also renders PNG views to renders/check_*.png.
Run after make_case_v6.py:  python3 case_v6/check_case.py      (exit code 1 on any FAIL)
Add an envelope here whenever you add or move a component in make_case_v6.py."""
import os, sys, numpy as np, trimesh
from shapely.geometry import MultiPoint
from shapely import affinity
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import make_case_v6 as C
from PIL import Image, ImageDraw, ImageFont

STL = os.environ.get("V6_OUT", os.path.join(HERE, "stl"))
BED, MARGIN = 180.0, 3.0
fails = []
def check(ok, msg): print(("PASS  " if ok else "FAIL  ") + msg); (None if ok else fails.append(msg))
PARTS = ["carrier_white", "cover_black", "body", "bezel_ring_black", "knob_red"] + (["pip_red"] if getattr(C, "BEZEL_PIP", None) else []) + (["clamp_plate"] if getattr(C, "CLAMP", False) and getattr(C, "SUBDIAL", False) else []) + ([] if getattr(C, "SUBDIAL", False) else ["tft_retainer"])
P = {n: trimesh.load(os.path.join(STL, f"v6_{n}.stl")) for n in PARTS}

for n, m in P.items(): check(m.is_watertight, f"{n} watertight ({len(m.faces)} faces, {np.round(m.extents, 1)} mm)")
if getattr(C, "BEZEL_PIP", None):   # plug vs slot: 0.1 per side
    bz, pp = P["bezel_ring_black"], P["pip_red"].copy(); pp.apply_translation((0, C.BEZEL_PIP[2], 0))
    check(bz.intersection(pp, engine="manifold").volume < 0.01, f"red pip plug fits its bezel slot (plug {np.round(P['pip_red'].extents, 2)} mm)")
for n in ["carrier_white", "cover_black", "body"]:
    hull = MultiPoint(P[n].vertices[:, :2]).convex_hull
    best = min((max(affinity.rotate(hull, a).bounds[2] - affinity.rotate(hull, a).bounds[0], affinity.rotate(hull, a).bounds[3] - affinity.rotate(hull, a).bounds[1]), a) for a in range(0, 91, 5))
    check(best[0] <= BED - MARGIN, f"{n} fits the bed: {best[0]:.1f} mm at {best[1]} deg (limit {BED - MARGIN:.0f})")

def ann(od, idr, z0, z1, cx, cy):
    a = trimesh.creation.cylinder(radius=od / 2, height=z1 - z0, sections=128); b = trimesh.creation.cylinder(radius=idr / 2, height=z1 - z0 + 1, sections=128)
    r = a.difference(b, engine="manifold"); r.apply_translation((cx, cy, (z0 + z1) / 2)); return r
def boxz(sx, sy, z0, z1, cx, cy):
    b = trimesh.creation.box((sx, sy, z1 - z0)); b.apply_translation((cx, cy, (z0 + z1) / 2)); return b
zb = -C.DEPTH + C.WALL
ENV = {}
def cylz(r, z0, z1, cx, cy):
    c = trimesh.creation.cylinder(radius=r, height=z1 - z0, sections=128); c.apply_translation((cx, cy, (z0 + z1) / 2)); return c
E = 0.02   # keep envelopes off coplanar seating faces (touching is fine, overlap is not)
if getattr(C, "SUBDIAL", False):
    # layered sub-dial: ring in a pit (back flush with the carrier back floor), OLED board lying on the ring's back,
    # its glass poking forward through the ring's hole, round display module above centre
    zr0, zr1 = C.RING_SEAT_Z - C.RING_PCB_T, C.RING_SEAT_Z            # ring PCB -3.7..-2.5
    for i, (cx, cy) in enumerate((C.POD1, C.POD2)):
        ENV[f"LED ring PCB {i + 1}"] = ann(C.RING_OD, C.RING_ID, zr0 + E, zr1 - E, cx, cy)
        ENV[f"LED ring LEDs {i + 1}"] = ann(C.RING_OD - 0.2, 73.4, zr1 + E, zr1 + C.RING_LED_H, cx, cy)   # LEDs anywhere out to the ring edge
    for i, ((ox, oy), (px, py), (gx, gy)) in enumerate(zip(C.OLEDS, C.OLED_PINS, C.OLED_GLASS)):
        zob = zr0 - C.OLED91_BOARD[2]
        ENV[f"OLED91 board {i + 1}"] = boxz(*C.OLED91_BOARD[:2], zob, zr0 - E, ox, oy)
        ENV[f"OLED rear parts {i + 1}"] = boxz(C.OLED91_BOARD[0] - 2 * getattr(C, "CLAMP_OLED_STRIP", 1.0) - 0.2, C.OLED91_BOARD[1], zob - 1.5, zob, ox, oy)      # datasheet: 1.5 max; FPC fold + pads included
        ENV[f"OLED header {i + 1}"] = boxz(C.OLED_PIN_SPAN + 2.6, 2.6, zob - 2.0 - 6.0 - 3.0, zob, px, py)            # plastic 2 + pins 6 + wire ends
        ENV[f"OLED pin stubs {i + 1}"] = boxz(C.OLED_PIN_SPAN + 1.2, 1.4, zr0 + E, zr0 + 2.3, px, py)   # 4 pins + solder fillets               # 0.5 pin end + solder on the front
        ENV[f"OLED91 glass {i + 1}"] = boxz(*C.OLED91_SCREEN, zr0 + E, zr0 + C.OLED91_GLASS_T, gx, gy)
    for i, (dx, dy) in enumerate(C.DISP):
        zf, zp = C.GLASS_POCKET_T, C.GLASS_POCKET_T - C.TFT_FRONT_T                      # glass front, PCB front (datasheet R123209)
        ENV[f"TFT module {i + 1}"] = cylz(C.TFT_MOD_R, zp - C.TFT_PCB_T + E, zp - E, dx, dy).union(
            boxz(C.TFT_TAB[0], C.TFT_TAB[1] - 14.0, zp - C.TFT_PCB_T + E, zp - E, dx, dy + (C.TFT_TAB[1] + 14.0) / 2), engine="manifold")   # PCB + tab
        ENV[f"TFT glass {i + 1}"] = cylz(C.TFT_BL[0] / 2, zp + E, zf - E, dx, dy).union(
            boxz(22.0, 4.0, zp + E, zf - E, dx, dy + C.TFT_BL[1] - 2.0), engine="manifold")                                   # glass + FPC ledge
        ENV[f"TFT rear SMD {i + 1}"] = cylz(17.5, zp - C.TFT_PCB_T - C.TFT_SMD_H, zp - C.TFT_PCB_T - E, dx, dy)
        ENV[f"TFT header {i + 1}"] = boxz(17.5, 3.5, zf - 12.0, zp - C.TFT_PCB_T, dx, dy + C.TFT_TAB[1] - 1.76)              # 7 pins behind the tab
else:
    for i, (cx, cy) in enumerate((C.POD1, C.POD2)): ENV[f"LED ring {i + 1}"] = ann(C.RING_OD, C.RING_ID, -(C.RING_GAP + 3.5), -C.RING_GAP, cx, cy)   # ring sits on shelves at z -11.5, face at -8
    for i, (ox, oy) in enumerate(C.OLEDS): ENV[f"OLED board {i + 1}"] = boxz(27.0, 27.0, -4.6, -3.0, ox, oy)
if getattr(C, "SUBDIAL", False):   # component vs component (they share the same layers)
    for a, b in [("OLED91 glass 1", "LED ring PCB 1"), ("OLED91 glass 1", "TFT module 1"), ("OLED91 board 1", "TFT module 1"),
                 ("OLED91 board 1", "LED ring PCB 1"), ("TFT module 1", "LED ring PCB 1"), ("OLED pin stubs 1", "TFT module 1"), ("OLED91 glass 1", "TFT rear SMD 1"), ("OLED pin stubs 1", "TFT rear SMD 1"), ("OLED header 1", "TFT header 1"), ("OLED rear parts 1", "LED ring PCB 1"), ("OLED pin stubs 1", "LED ring PCB 1"), ("OLED header 1", "LED ring PCB 1"), ("TFT glass 1", "LED ring PCB 1")]:
        v = ENV[a].intersection(ENV[b], engine="manifold").volume
        check(v < 0.5, f"{a} vs {b}: overlap {v:.2f} mm3")
    zf = C.RING_SEAT_Z - C.RING_PCB_T + C.OLED91_GLASS_T
    print(f"INFO  OLED glass front z {zf:.2f}, display PCB back z {C.GLASS_POCKET_T - C.TFT_MOD_T:.2f}, rear SMD to z {C.GLASS_POCKET_T - C.TFT_MOD_T - C.TFT_SMD_H:.2f} (inside r 17.5)")
if getattr(C, "SUBDIAL", False):   # seating checks: parts pushed 0.3 mm forward must hit their ledge (proves they are held, not floating)
    for nm, e in [("LED ring", ann(C.RING_OD, C.RING_ID, C.RING_SEAT_Z - C.RING_PCB_T + 0.3, C.RING_SEAT_Z + 0.3, *C.POD1)),
                  ("OLED91 board", boxz(*C.OLED91_BOARD[:2], C.RING_SEAT_Z - C.RING_PCB_T - C.OLED91_BOARD[2] + 0.3, C.RING_SEAT_Z - C.RING_PCB_T + 0.3, *C.OLEDS[0])), ("OLED91 glass", boxz(*C.OLED91_SCREEN, C.RING_SEAT_Z - C.RING_PCB_T + 0.3, C.RING_SEAT_Z - C.RING_PCB_T + C.OLED91_GLASS_T + 0.3, *C.OLED_GLASS[0]))]:
        held = P["carrier_white"].intersection(e, engine="manifold").volume
        if nm == "OLED91 glass":   # the glass is held at its front by the bezel's black skirt
            for (bcx, bcy) in (C.POD1,):
                bz = P["bezel_ring_black"].copy(); bz.apply_translation((bcx, bcy, C.PLATE_T)); held += bz.intersection(e, engine="manifold").volume
        check(held > 1.0, f"{nm} seats against a carrier ledge" + (" / bezel skirt" if nm == "OLED91 glass" else ""))
tx, ty = C.TRAY_POS; ENV["ESP32-S3 board"] = boxz(C.BOARD_L, C.BOARD_W, zb + C.PIN_DROP, zb + C.PIN_DROP + 1.6 + 3.5, tx, ty)
ENV["ESP32-S3 pins + wires"] = boxz(C.BOARD_L - 12, C.BOARD_W - 9, zb + 0.5, zb + C.PIN_DROP, tx, ty)
for i, kx in enumerate(C.KNOB_X): ENV[f"KY-040 encoder {i + 1}"] = boxz(27.0, 10.5, -31.5, -12.5, kx, 89.25 if C.OUTLINE == "eight" else C.H - C.WALL - 7.0)   # PCB + parts under the roof
if getattr(C, "SPEAKER", None):
    sx, sy, sd, sdep = C.SPEAKER; ENV["3 W speaker (reserved)"] = cylz(sd / 2, zb + E, zb + sdep, sx, sy)
if getattr(C, "BATTERY", None):
    bx_, by_, bl_, bh_, bt_ = C.BATTERY; ENV["LiPo pack (reserved)"] = boxz(bl_, bh_, zb + E, zb + bt_, bx_, by_)
for nm in ("CHARGER", "AMP", "HUB"):
    if getattr(C, nm, None):
        cx_, cy_, cw_, ch_, ct_, cg_ = getattr(C, nm); ENV[f"{nm.lower()} module (reserved)"] = boxz(cw_, ch_, zb + cg_, zb + cg_ + ct_, cx_, cy_)
if getattr(C, "USB_EXIT", "side") == "back":
    bl = tx - C.BOARD_L / 2; uz = zb + C.PIN_DROP + 3.2
    ENV["USB 90deg plug"] = boxz(C.USB_PLUG_ROOM - 0.6, 12.0, uz - 4.0, uz + 4.0, bl - (C.USB_PLUG_ROOM - 0.6) / 2, ty)          # plug body at the board's USB end
    ux, uy = C.USB_BACK
    ENV["USB panel socket"] = boxz(C.USB_BACK_POCKET[0] - 0.4, C.USB_BACK_POCKET[1] - 0.4, zb + 0.02, zb + 22.0, ux, uy)        # female housing + cable exit

# internal layout: every reserved / board / encoder volume against every other component envelope
_keys = list(ENV)
for i, a_ in enumerate(_keys):
    for b_ in _keys[i + 1:]:
        if not any(k in a_ + b_ for k in ("reserved", "ESP32", "encoder", "USB")): continue
        ea, eb = ENV[a_], ENV[b_]
        if (ea.bounds[0] > eb.bounds[1]).any() or (eb.bounds[0] > ea.bounds[1]).any(): continue
        v = ea.intersection(eb, engine="manifold").volume
        check(v < 0.5, f"{a_} vs {b_}: overlap {v:.2f} mm3")
CHECK_PARTS = ["carrier_white", "cover_black", "body"]
for nm in ("carrier_white", "cover_black"):   # 10 Oct: the body (sleeve + front-screw lugs) must not cut into the front unit
    v = P["body"].intersection(P[nm], engine="manifold").volume
    check(v < 0.5, f"body vs {nm} (assembled): overlap {v:.2f} mm3")
if getattr(C, "SUBDIAL", False):   # bezels in place (their black skirts reach down to both glasses)
    bzs = []
    for (cx, cy) in (C.POD1, C.POD2):
        m = P["bezel_ring_black"].copy(); m.apply_translation((cx, cy, C.PLATE_T)); bzs.append(m)
    P["bezels (placed)"] = trimesh.util.concatenate(bzs); CHECK_PARTS.append("bezels (placed)")
    v = P["bezels (placed)"].intersection(P["carrier_white"], engine="manifold").volume
    check(v < 0.5, f"bezels + skirts vs carrier_white: overlap {v:.2f} mm3")
if "clamp_plate" in P:   # both plates in place, as one printed part
    cps = []
    for (cx, cy) in (C.POD1, C.POD2):
        m = P["clamp_plate"].copy(); m.apply_translation((cx, cy, 0)); cps.append(m)
    P["clamp plates (placed)"] = trimesh.util.concatenate(cps); CHECK_PARTS.append("clamp plates (placed)")
    for n in ["carrier_white", "body"]:
        v = P[n].intersection(P["clamp plates (placed)"], engine="manifold").volume
        check(v < 0.5, f"clamp plates vs {n}: overlap {v:.2f} mm3 (posts and ledges must touch, not cut in)")
    # it must actually press: plates nudged 0.3 mm toward the front hit the ring, the OLED board and the display PCB
    nudged = P["clamp plates (placed)"].copy(); nudged.apply_translation((0, 0, 0.3))
    for tgt in ("LED ring PCB 1", "OLED91 board 1", "TFT module 1"):
        check(nudged.intersection(ENV[tgt], engine="manifold").volume > 0.5, f"clamp plate presses {tgt.replace(' 1', '')}")
    # and the body must hold it: plates nudged 0.3 mm toward the back hit the body (post tops + ledges)
    nb = P["clamp plates (placed)"].copy(); nb.apply_translation((0, 0, -0.3))
    check(nb.intersection(P["body"], engine="manifold").volume > 0.5, "body posts + ledges back the clamp plates")
for en, e in ENV.items():
    for n in CHECK_PARTS:
        v = P[n].intersection(e, engine="manifold").volume if P[n].bounds[0][2] < e.bounds[1][2] else 0.0
        check(v < 0.5, f"{en} vs {n}: overlap {v:.2f} mm3")

# renders: front and back of carrier/cover/body, one sheet
def render(mesh, rot, title, size=(560, 420), col=(120, 170, 230)):
    def R(ax, a):
        a = np.deg2rad(a); c, s = np.cos(a), np.sin(a)
        return {'x': np.array([[1, 0, 0], [0, c, -s], [0, s, c]]), 'y': np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])}[ax]
    v = (mesh.vertices - mesh.vertices.mean(0)) @ (R('x', rot[0]) @ R('y', rot[1])).T; tri = v[mesh.faces]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); n /= np.linalg.norm(n, axis=1)[:, None] + 1e-9
    L = np.array([0.3, 0.5, 0.8]); L /= np.linalg.norm(L); sh = np.clip(0.35 + 0.65 * np.abs(n @ L), 0, 1)
    xy = tri[:, :, :2]; mn, mx = xy.reshape(-1, 2).min(0), xy.reshape(-1, 2).max(0); s = min((size[0] - 40) / (mx[0] - mn[0]), (size[1] - 60) / (mx[1] - mn[1]))
    im = Image.new('RGB', size, (18, 20, 24)); d = ImageDraw.Draw(im)
    for i in np.argsort(tri[:, :, 2].mean(1)):
        if n[i, 2] < -0.05: continue
        d.polygon([(size[0] / 2 + (p[0] - (mn[0] + mx[0]) / 2) * s, size[1] / 2 + 15 - (p[1] - (mn[1] + mx[1]) / 2) * s) for p in xy[i]], fill=tuple(int(c * sh[i]) for c in col))
    d.text((12, 10), title, fill=(230, 230, 230)); return im
views = [render(P["cover_black"], (0, 0), "cover front"), render(P["carrier_white"], (0, 180), "carrier back", col=(225, 225, 225)),
         render(P["body"], (0, 180), "body, looking in"), render(P["body"], (-60, 0), "body rear/top")]
sheet = Image.new('RGB', (1120, 840)); [sheet.paste(im, ((k % 2) * 560, (k // 2) * 420)) for k, im in enumerate(views)]
os.makedirs(os.path.join(HERE, "renders"), exist_ok=True); out = os.path.join(HERE, "renders", "check_views.png"); sheet.save(out)
print(f"\nrenders: {out}\n{'ALL PASS' if not fails else str(len(fails)) + ' FAIL'}")
sys.exit(1 if fails else 0)
