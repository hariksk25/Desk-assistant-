#!/usr/bin/env python3
"""Generate a drop-in tray STL for an ESP32 DevKit V1 (30-pin) board.

Design: open-top pocket. Board drops in from above and is held laterally by
the walls (friction fit). Four corner posts support the bare PCB corners so
the downward header pins hang free. Both end walls stop at PCB-bottom height
so the USB connector and the PCB antenna are not blocked.

All dimensions in mm. Edit the CONFIG block and re-run.
"""
import struct

# ---------------- CONFIG ----------------
BOARD_L   = 51.5   # board length (USB end to antenna end). Measure yours.
BOARD_W   = 28.5   # board width across the two header rows. Measure yours.
CLEAR     = 0.30   # per-side clearance between board and wall
PIN_DROP  = 8.7    # header pin tail length below PCB underside (incl. plastic)
PCB_T     = 1.6
LIP       = 1.5    # side wall height above PCB top
WALL_T    = 1.6
BASE_T    = 1.2
POST_L    = 4.0    # corner post footprint along board length
POST_W    = 3.5    # corner post footprint across board width
OUT       = "/mnt/user-data/outputs/desk_plane_radar/esp32_devkit_v1_tray.stl"
# ----------------------------------------

inner_L = BOARD_L + 2 * CLEAR
inner_W = BOARD_W + 2 * CLEAR
outer_L = inner_L + 2 * WALL_T
outer_W = inner_W + 2 * WALL_T
z_base  = BASE_T
z_pcb   = BASE_T + PIN_DROP           # PCB underside
z_side  = z_pcb + PCB_T + LIP         # side wall top
z_end   = z_pcb                       # end wall top (USB / antenna clear)

boxes = []  # (x0,y0,z0,x1,y1,z1), axis-aligned, non-overlapping

# base
boxes.append((0, 0, 0, outer_L, outer_W, z_base))
# side walls (long)
boxes.append((0, 0, z_base, outer_L, WALL_T, z_side))
boxes.append((0, outer_W - WALL_T, z_base, outer_L, outer_W, z_side))
# end walls (short, low)
boxes.append((0, WALL_T, z_base, WALL_T, outer_W - WALL_T, z_end))
boxes.append((outer_L - WALL_T, WALL_T, z_base, outer_L, outer_W - WALL_T, z_end))
# corner posts inside the pocket
xi0, xi1 = WALL_T, outer_L - WALL_T
yi0, yi1 = WALL_T, outer_W - WALL_T
boxes.append((xi0, yi0, z_base, xi0 + POST_L, yi0 + POST_W, z_pcb))
boxes.append((xi0, yi1 - POST_W, z_base, xi0 + POST_L, yi1, z_pcb))
boxes.append((xi1 - POST_L, yi0, z_base, xi1, yi0 + POST_W, z_pcb))
boxes.append((xi1 - POST_L, yi1 - POST_W, z_base, xi1, yi1, z_pcb))


def box_tris(b):
    x0, y0, z0, x1, y1, z1 = b
    v = [(x0,y0,z0),(x1,y0,z0),(x1,y1,z0),(x0,y1,z0),
         (x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1)]
    faces = [  # outward-wound (CCW seen from outside)
        (0,2,1),(0,3,2),          # bottom  (-z)
        (4,5,6),(4,6,7),          # top     (+z)
        (0,1,5),(0,5,4),          # front   (-y)
        (1,2,6),(1,6,5),          # right   (+x)
        (2,3,7),(2,7,6),          # back    (+y)
        (3,0,4),(3,4,7),          # left    (-x)
    ]
    return [(v[a], v[b_], v[c]) for a, b_, c in faces]


def normal(t):
    (ax,ay,az),(bx,by,bz),(cx,cy,cz) = t
    ux,uy,uz = bx-ax, by-ay, bz-az
    vx,vy,vz = cx-ax, cy-ay, cz-az
    nx,ny,nz = uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx
    l = (nx*nx+ny*ny+nz*nz) ** 0.5 or 1.0
    return nx/l, ny/l, nz/l


tris = [t for b in boxes for t in box_tris(b)]
with open(OUT, "wb") as f:
    f.write(b"ESP32 DevKit V1 30-pin tray".ljust(80, b"\0"))
    f.write(struct.pack("<I", len(tris)))
    for t in tris:
        f.write(struct.pack("<3f", *normal(t)))
        for p in t:
            f.write(struct.pack("<3f", *p))
        f.write(struct.pack("<H", 0))

print(f"wrote {OUT}: {len(tris)} triangles")
print(f"outer footprint {outer_L:.1f} x {outer_W:.1f} mm, height {z_side:.1f} mm")
print(f"pocket {inner_L:.1f} x {inner_W:.1f} mm, PCB underside sits {z_pcb:.1f} mm above tray bottom")
