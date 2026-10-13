#!/usr/bin/env python3
"""Internal layout plan, seen from the BACK (mirrored so it matches looking into the open body). Writes renders/internal_layout.png."""
import os, sys, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import make_case_v6 as C
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon as MP
fig, ax = plt.subplots(figsize=(14, 8.2), facecolor="#14161a"); ax.set_facecolor("#14161a"); ax.set_aspect("equal"); ax.axis("off")
def X(x): return C.W - x                                        # mirror: seen from the back
def rect(cx, cy, w, h, col, label, a=0.85, fs=9, tc="w"):
    ax.add_patch(Rectangle((X(cx) - w / 2, cy - h / 2), w, h, fc=col, ec="w", lw=0.6, alpha=a, zorder=3))
    ax.text(X(cx), cy, label, color=tc, ha="center", va="center", fontsize=fs, zorder=4)
o = C.eight2d(); ax.add_patch(MP(np.c_[C.W - np.array(o.exterior.xy[0]), o.exterior.xy[1]], closed=True, fc="#26282d", ec="#888", lw=1.2, zorder=1))
i = C.eight2d(C.WALL); ax.add_patch(MP(np.c_[C.W - np.array(i.exterior.xy[0]), i.exterior.xy[1]], closed=True, fc="#1b1d21", ec="#555", lw=0.8, zorder=1))
for (cx, cy) in (C.POD1, C.POD2):
    ax.add_patch(Circle((X(cx), cy), C.RING_OD / 2, fc="none", ec="#4caf50", lw=1, ls="--", zorder=2))
    ax.add_patch(Circle((X(cx), cy), C.TFT_MOD_R, fc="none", ec="#3fa9f5", lw=1, ls="--", zorder=2))
for (sx, sy) in C.SCREWS: ax.add_patch(Circle((X(sx), sy), 3.5, fc="#999", ec="w", zorder=5))
for (sx, sy) in C.COVER_SCREWS: ax.add_patch(Circle((X(sx), sy), 1.8, fc="#555", ec="#bbb", zorder=5))
tx, ty = C.TRAY_POS; rect(tx, ty, C.BOARD_L, C.BOARD_W, "#2e7d32", "ESP32-S3 N16R8\n(USB-C out the side)")
ax.annotate("", xy=(X(-6), ty), xytext=(X(tx - C.BOARD_L / 2), ty), arrowprops=dict(arrowstyle="->", color="#ffd23f", lw=2), zorder=6)
ax.text(X(-8), ty + 5, "90 deg\nUSB-C", color="#ffd23f", fontsize=9, ha="left", zorder=6)
sx, sy, sd, sdep = C.SPEAKER; ax.add_patch(Circle((X(sx), sy), sd / 2, fc="#b23b3b", ec="w", alpha=0.85, zorder=3))
ax.text(X(sx), sy, f"3 W speaker\n{sd:.0f} mm, rear firing\n(grille in back wall)", color="w", ha="center", va="center", fontsize=9, zorder=4)
bx_, by_, bl_, bh_, bt_ = C.BATTERY; rect(bx_, by_, bl_, bh_, "#c58b00", f"LiPo 2 x 103450\n~4000 mAh, {bt_:.0f} mm stack")
rect(*C.CHARGER[:4], "#6a4c93", "charger /\nboost", fs=8); rect(*C.AMP[:4], "#00838f", "I2S amp", fs=8)
if getattr(C, "HUB", None): rect(*C.HUB[:4], "#e0e0e0", "wiring hub", fs=8)
for kx in C.KNOB_X: rect(kx, 89.25, 27, 10.5, "#555", "KY-040", fs=8)
for (ox, oy) in C.OLEDS: rect(ox, oy, *C.OLED91_BOARD[:2], "#1565c0", "", a=0.35)
ax.set_xlim(-25, C.W + 15); ax.set_ylim(-8, C.H + 8)
ax.set_title("Internal plan, looking into the body from the BACK (left/right mirrored)", color="w", fontsize=13)
ax.text(2, -5, "grey dots: carrier posts (4, both pods)   small dots: 2 cover screws   dashed: ring OD / display PCB / OLED board (front side)", color="#aaa", fontsize=9)
fig.savefig(os.path.join(HERE, "renders", "internal_layout.png"), dpi=100, facecolor=fig.get_facecolor(), bbox_inches="tight")
print("ok")
