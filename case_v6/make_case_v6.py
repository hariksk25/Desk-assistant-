#!/usr/bin/env python3
"""Desk Plane Radar v6 "twin dial" enclosure (parametric, mm).

Horns layout: two 93 mm pods (1.28" GC9A01 + 24-LED WS2812B ring, 86 mm OD / 70 mm ID, each) joined by a bridge that holds the two 0.96"
OLEDs stacked in the middle. The ESP32-S3 lies under the left pod with its two USB-C ports through a small
flat facet cut into the round left end (plug the cable straight in, no pigtail). Both ends stay round. Nothing on the front but glass. Both encoders (red knobs)
on the roof, one above each dial, so pressing pushes the device into the desk.

Front is two layers:
  carrier (white PLA, 3 mm): every mounting lives here: display bosses, OLED bosses, LED ring posts, screws to
          the body (hidden), inserts for the cover. Over each LED ring the carrier is thinned to 1 mm so it
          is the diffuser; the ring sits RING_GAP (8 mm) behind it on 3 posts for an even glow.
  cover   (black PLA, 2 mm): bezel with the display, ring and OLED cut-outs; 4 x M2 x 6 wafer screws into
          3 mm brass inserts in the carrier.

Coordinates: x right, y up, z toward the viewer. Plate z in [0, PLATE_T], body z in [-DEPTH, 0].
Print: the outline (180 x about 125 mm) fits the A1 Mini (180 x 180) rotated 45 degrees in the slicer: about 160.7 x 160.7. Verify with check_case.py.
Requires: pip install trimesh manifold3d numpy shapely
"""
import numpy as np, trimesh, os
from trimesh.creation import box, cylinder

# ---------------- CONFIG ----------------
OUT_DIR = os.environ.get("V6_OUT", os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl"))
POD_R   = 45.5                      # pod outer radius (93 mm pods): ring OD 86 + 0.5 clearance + 2.5 wall each side
PITCH   = 89.0                      # pod centre spacing: rings 10 mm apart at the centre line; the two OLEDs sit above, between the knobs ("horns")
CY      = POD_R + 9.5               # pod centre line (y). Raised above the pod radius: the 30 degree lean lifts the floor plane up
                                    # to y 12.4 behind the ring (ring back at z -11.5), so the ring (86 mm) would otherwise cut into the sloped bottom wall.
BASE_R  = 45.5                      # the body has a flat base under the pods (bottom corners rounded with this radius)
POD1, POD2 = (POD_R, CY), (POD_R + PITCH, CY)
PLINTH  = None                      # no plinth in this form
W, H    = 2 * POD_R + PITCH, CY + POD_R
DEPTH   = 38.0                      # ring seat 8 + ring 3.5, encoder PCBs behind it (z -31.5), 4 mm to the back wall, room for the speaker later
WALL    = 2.5
PLATE_T = 3.0
SEG     = 96

# v1 learnings: the whole device leans back so the display is not cut off when viewed from above, the
# window edges are chamfered for the same reason, and each round display is held at 4 points (2 tab
# screws + a retainer bar across the top) with its glass sitting free in a shallow pocket, so no pressure mark.
TILT = 30.0                                    # degrees the front leans back; the bottom face is cut at this angle (sloped floor, full wall)
WIN_CHAMFER = 1.3                              # carrier window widens by this much toward the front (45 degree)

# GC9A01 round module: window and the two tab holes, measured from Noah Thio's panel
TFT_WIN_D = 34.4; TFT_HOLE_DX = 19.3 / 2; TFT_HOLE_DY = -20.4
BOSS_D, BOSS_H, INSERT_D = 6.0, 4.5, 3.2       # M2 heat-set insert bosses (3.5 mm OD inserts)
GLASS_POCKET_D, GLASS_POCKET_T = 36.6, 0.8     # the module's glass sits in this pocket on the carrier back, never touching the window edge
TFT_BAR_BOSS = (13.0, 18.0)                    # retainer bar bosses (dx, dy from the display centre), 1.7 mm self-tap holes
TFT_BAR = (34.0, 4.0, 2.0)                     # retainer bar: length, width, thickness; its lower edge overlaps the PCB top by ~2.7 mm

# 24-LED WS2812B ring, 86 mm OD / 70 mm ID (robu.in listing). The carrier is thinned to SKIN over the LEDs
# (white PLA = the diffuser); the ring sits RING_GAP behind it on 3 posts with shelves under its inner edge.
# LEDs sit on the mid radius (about 39 mm), 10 mm apart, so the gap is raised from 5 to 8 mm to hide the hot spots.
RING_OD, RING_ID = 85.0, 72.0              # LED ring outer/inner, measured on the delivered ring (listing said 86/70)
GLOW_OD, GLOW_ID = 84.0, 73.0              # thinned (1 mm) diffuser zone in the white carrier, and the cover cut-out
# 9 Oct, after the first carrier print: the ring would not drop in. The outer LED corners caught the 84 mm channel wall (LEDs sit
# nearer the ring's outer edge than centred), and the inner floor was 71.4 for a 72.0 ring. Now: LED channel in the back layer
# opened out to the ring OD (the ring rests on the inner ledge only, the clamp plate holds it flat), inner floor 71.0.
LED_CH_OD, LED_CH_ID = RING_OD + 0.6, GLOW_ID # 85.6 / 73.0: LEDs and their capacitors run right to the ring edge (back layer only; skin + cover hole unchanged)
RING_SEAT_ID = RING_ID - 1.0                  # 71.0 inner floor (0.5 mm clearance a side)
SKIN = 1.0                                 # carrier thickness left over the ring
RING_GAP = 8.0                             # ring face to skin distance (3 posts with shelves hold it there)
COVER_T = 2.0
BEZEL_OD, BEZEL_ID = GLOW_ID - 0.4, TFT_WIN_D   # separate inner bezel ring per pod (black, 18 mm wide), sits inside the ring's inner diameter on the white carrier
BEZEL_PEG_D, BEZEL_PEG_L, BEZEL_PEG_R = 1.8, 2.4, 21.5   # 3 press-fit pegs into the carrier
# dial ticks cut THROUGH the black bezel so the white carrier shows: 60 minute, 12 hour, 4 quarter marks
BEZEL_PIP = (2.4, 6.0, 32.6)               # 9 Oct, owner pick "C": one red 12 o'clock mark per bezel: slot w, l, radius of its centre
PIP_CLEAR = 0.1
# 9 Oct: clamp plates instead of glue. One printed plate per dial lies on the carrier's back floor, squeezed by the body:
# its face presses the LED ring back and the OLED board (pocket), two side arcs reach through the display pocket and press
# the round display's PCB rim (3 and 9 o'clock only, clear of its rear parts and the OLED glass). The 2 carrier screws per
# dial pass through it; 6 ledges inside the body wall press its rim. Window at 12 o'clock for the TFT header + ring wires
# (turn the ring so its wire joint sits at 12), window over the OLED's rear parts and header.
CLAMP = True
# 9 Oct: the body screws in from the BACK (the front unit, cover + bezels + carrier, is assembled first and never has to come off).
# M3 x 35 thread-forming screws for plastic: head in a counterbore in the back wall, through a tube post and the clamp plate,
# biting into the carrier's existing 2.8 / 2.4 mm screw holes (about 4.7 mm of thread), tip stops under the carrier's front counterbore.
BACK_SCREW = (3.4, 6.4, 4.0, 35.0)               # clearance bore, head counterbore dia, counterbore depth into the back, screw length
CLAMP_T = 1.6                                    # plate thickness
CLAMP_R = 42.3                                   # plate radius (body inner wall r 43)
CLAMP_LEDGES = (30.0, 60.0, 120.0)               # ledge angles away from the pod's outer direction (+/- each), clear of the waist
CLAMP_OLED_STRIP = 1.5                           # OLED board side strips the plate presses (check the real board: parts must stay 1.5 from its sides)
CLAMP_WIRE_WIN = (20.0, 21.0)                    # 12 o'clock wire window: width, start radius (to the plate edge)                            # per side: the red plug is 2.2 x 5.8 x COVER_T, press fit + a drop of CA glue
BEZEL_TICKS = "none"                       # 9 Oct, owner: minimal bezel. "none", "quarters" (12/3/6/9 only) or "full" (60 minute marks)
TICK_MIN = (0.7, 2.4, 33.0)                # tangential width, radial length, radius of the centre
TICK_HOUR = (1.3, 4.8, 31.6)
TICK_QTR = (2.0, 7.0, 30.5)
COVER_PEG_R, COVER_PEG_N = 43.8, 8        # 8 small pegs per pod on the cover's back hold its thin outer band (it only has 4 screws, all in the bridge)
# status bar: WS2812B 1x8 stick (51 x 10 mm) under the bridge roof, white diffuser strip in a recess on top
STATUS_BAR = False
STATUS_SLOT = (52.0, 11.0); STATUS_DIFF = (54.0, 13.0, 1.2); STATUS_POS = ((POD1[0] + POD2[0]) / 2, -DEPTH / 2)
MX = (POD1[0] + POD2[0]) / 2
# 9 Oct: "figure 8" / binocular outline: two round pods joined by a waist with concave fillets (no fill above and below the waist)
OUTLINE = "eight"                                # "eight" or "stadium" (the old full-height bridge + flat base)
WAIST_HH = 26.0                                  # waist half height at the centre line (52 mm waist vs 91 mm pods)
_RF = ((PITCH / 2) ** 2 + WAIST_HH ** 2 - POD_R ** 2) / (2 * (POD_R - WAIST_HH))   # fillet radius tangent to both pods (15.0)
if OUTLINE == "eight":
    COVER_SCREWS = [(MX, CY + 19.0), (MX, CY - 19.0)]                            # 2 cover screws in the waist
else:
    COVER_SCREWS = [(MX - 9.0, CY + 30.0), (MX + 9.0, CY + 30.0), (MX - 9.0, CY - 30.0), (MX + 9.0, CY - 30.0)]   # between the rings, above and below the centre line

# two 0.96" SSD1306 stacked in the bridge: window and hole pattern from the original panel
OLED_W, OLED_GAP = 27.0, 3.0                       # board width, gap between the two boards
HORN_DY = np.sqrt((RING_OD / 2 + 1.0) ** 2 - (PITCH / 2 - OLED_W - OLED_GAP / 2) ** 2)   # lowest board edge above the pod centre line (1 mm clear of the ring OD)
OLED_Y = CY + HORN_DY + OLED_W / 2                  # board centre line
OLEDS = [(MX - (OLED_W + OLED_GAP) / 2, OLED_Y), (MX + (OLED_W + OLED_GAP) / 2, OLED_Y)]
HORN_HW = OLED_W + OLED_GAP / 2 + 3.5               # half width of the raised block on top between the knobs
HORN_TOP = OLED_Y + OLED_W / 2 + 3.0
HORN_Y0, HORN_R = CY + 20.0, 5.0
OLED96_WIN = (24.7, 14.9); OLED96_WIN_OFF = 2.07; OLED96_HOLES = (23.25, 23.5)

# straight ends: the pod's outer half-circle is replaced by a rounded rectangle (needed where the USB ports exit)
STRAIGHT_ENDS = ()                               # ("left",), ("left", "right") or (); () keeps both ends round
END_R = 6.0                                      # corner radius of a straight end

# controls on the roof (shaft axis = y), one above each dial. z is the hole position behind the front face.
KNOB_HOLE = 7.4; KNOB_Z = -22.0                  # KY-040 PCB (26 x 19) hangs under the roof between z -31.5 and -12.5, behind the ring (ring back at -11.5)
KNOB_X = (POD1[0], POD2[0])

# ESP32-S3-DevKitC-1 under the left pod, long axis along x, USB end against the straight left wall.
# y is chosen so the board misses the TFT header pins (tab at y 17.6) and the ring post at (38, 61).
BOARD_L, BOARD_W, CLEAR, PIN_DROP, TRAY_WALL_H = 69.9, 28.5, 0.3, 9.5, 12.0
USB_FACET_X = 5.0                                # flat facet milled into the round left end at this x (4.5 mm deep at the centre line, fading out at its edges)
USB_FACET = (34.0, 11.5)                         # facet size (y, z): both ports plus plug overmould
USB_WALL = 1.5                                   # wall left between the facet and the connector faces
# 9 Oct: USB exits through the BACK wall: a short USB-C extension (90 degree male into the board, female panel end in the
# back wall). The board moves right to leave room for the 90 degree plug at its left end.
USB_EXIT = "side"                                # "back" or "side". 9 Oct (later): owner chose SIDE with a 90 degree USB-C cable
USB_PLUG_ROOM = 11.0                             # inner wall to the board's USB end (90 degree plug)
USB_BACK = (POD1[0], 33.0)                       # socket centre on the back wall (x, y): behind the left dial, low, between its two posts
USB_BACK_HOLE = (9.8, 4.2)                       # through hole for the female receptacle face (USB-C shell 8.9 x 3.3)
USB_BACK_POCKET = (13.8, 7.8, 12.0)              # sleeve inside the back wall for the female housing (w, h, depth); glue it in
TRAY_POS = ((USB_FACET_X + USB_WALL + CLEAR if USB_EXIT == "side" else POD1[0] - np.sqrt((POD_R - WALL) ** 2 - 4.5 ** 2) + USB_PLUG_ROOM + CLEAR) + BOARD_L / 2, CY + 4.5)   # 28.5 wide board: top edge stays at CY + 18.75 (same as the 25.5 DevKitC-1 at CY + 6), corners clear the round wall    # board x from 6.8 to 76.7; corners clear the round inner wall
USB_SLOT = (30.0, 7.5)                           # one slot through the facet covering both USB-C ports

# piezo clip on the inner back wall (right pod, upper), sound holes in the back wall. None: replaced by the 3 W speaker
PIEZO = None
# ---- 9 Oct: reserved volumes, planned now so the front never needs a reprint (body features can follow later)
# 3 W speaker: 40 mm round driver, rear firing, in the waist just right of the S3 board (waist posts removed for it); grille in the back wall
SPEAKER = (POD2[0] + 18.0, CY + 9.0, 40.0, 22.0)  # 10 Oct, owner: moved off the waist (the cover is held only by its 2 waist screws) to the RIGHT pod, away from the USB side:
                                                 # outer end of the right pod, above its right post, beside the encoder. x, y, diameter, depth from the inner back wall
# battery: flat 1S LiPo, 2 x 103450 (10 x 34 x 50, about 2000 mAh each) stacked in parallel = about 4000 mAh.
# 10 Oct: now in the waist (where the speaker was), starting just past the S3 tray end stop, above the right pod's left post.
BATTERY = (TRAY_POS[0] + BOARD_L / 2 + 2.4 + 50.5 / 2, CY + 5.5, 50.5, 34.5, 21.0)   # x, y, length (x), height (y), thickness (z) from the inner back wall
# charger / boost module (IP5306 class, about 25 x 18) and the I2S amp (MAX98357A, 18 x 19): flat, mid-depth, low in each pod.
# 10 Oct: swapped so the amp sits next to the speaker (right pod) and the charger under the S3 (left pod)
CHARGER = (POD1[0], CY - 25.0, 26.0, 18.0, 6.0, 13.0)    # ..., thickness, gap from the inner back wall
AMP = (POD2[0], CY - 23.0, 18.0, 19.0, 6.0, 13.0)
# 10 Oct: wiring hub (stripboard bus for GND / 3V3 / 5V / SDA / SCL; the shared SPI lines are Y-splices), taped to the inner back wall of the left pod under the S3, between its posts
HUB = (POD1[0], CY - 20.0, 26.0, 14.0, 8.0, 0.0)          # 10 x 5 hole stripboard (GND, 3V3, 5V, SDA, SCL); x, y, w, h, thickness incl. wires, gap from the inner back wall. Clear of the sloped floor, posts, S3 tray, charger


# red knob for the KY-040 (6 mm D-shaft, flat 4.5 mm across, flat length ~7 mm from the shaft end)
KNOB_D, KNOB_H = 18.0, 16.0                      # 18 mm wide, 16 mm tall
KNOB_BORE_D, KNOB_BORE_FLAT, KNOB_BORE_DEPTH = 6.3, 4.7, 14.5    # bore with print clearance, 1.5 mm solid top
KNOB_D_LEN = 6.5                                 # the D flat exists only in the top 6.5 mm of the bore (the shaft is round below its flat); the rest is a plain round bore
KNOB_NUT_RELIEF = (12.6, 5.0)                    # pocket at the bottom that swallows washer, nut and the threaded bush
KNOB_FLUTES, KNOB_FLUTE_D = 14, 1.8              # grip grooves
KNOB_POINTER = False                             # no pointer: the encoder is endless, there is nothing to point at

# plate screws (inserts in body bosses, M2 x 10 from the front)
# carrier to body: full-length posts from the back wall. One in the bridge above the OLEDs, two per pod INSIDE the ring's
# inner diameter (under the black bezel, which hides the heads), at 222 and 318 degrees, clear of the S3 tray and the TFT tabs.
COVER_SCREW_LUG = True                           # 10 Oct, owner: the 2 front (waist) screws continue into lugs in the body: M2 x 16 from the front
COVER_LUG_FLOOR = -14.0                          # lugs are clipped here so they stay 0.5 above the LiPo stack
COVER_SCREW_PILOT = 1.7                          # pilot for an M2 thread-forming / self-tapping screw in PLA
BODY_SLEEVE = True                               # 10 Oct, owner: body wraps the front unit's sides up to the cover front (no white carrier edge from the side)
SLEEVE_T, SLEEVE_CLEAR = 1.2, 0.2                # wall around the front unit, gap to it: the body outline grows 1.4 mm a side
KNOB_SPOT_D = 13.0                               # spot face (1.4 deep, hidden under the 18 mm knob) so the encoder bush still reaches through a 2.5 roof
STANDOFF_R, STANDOFF_ANG = 25.5, (222.0, 318.0)
SCREWS = ([] if OUTLINE == "eight" else [(MX, CY + 16.0), (MX, CY - 16.0)]) + [(cx + STANDOFF_R * np.cos(np.deg2rad(a)), cy + STANDOFF_R * np.sin(np.deg2rad(a))) for (cx, cy) in (POD1, POD2) for a in STANDOFF_ANG]
SCREW_D, SCREW_HEAD_D, SCREW_HEAD_H = 2.4, 4.2, 2.0
# ---- 9 Oct 2026: "sub-dial" layout, no horns. Each dial: round display 5 mm above centre, OLED below it inside the ring.
# Built face down: the ring drops LEDs-first into a pit in a back layer of the carrier (its back ends flush with that
# floor), the OLED board lies flat on the floor across the ring's back (it never blocks the LEDs, they face the front),
# its glass sinks into a pocket and looks out through one window. Ring, display and OLED are hot-glued.
SUBDIAL = True
DISP_DY = 0.0                                    # display centre above the pod / ring centre (0 = dead centre, owner's choice 9 Oct)
# GC9A01 1.28" module, datasheet R123209 (9 Oct): round PCB 38, tab 22.92 wide to 26.5 from centre (holes 2.0 at 20.67, 19.26 apart,
# 7-pin header 1.76 from the tab end), backlight/glass 35.6 wide x 38.1 tall (FPC ledge 1.3 past the circle on the tab side),
# front stack 1.6 + PCB 1.6, rear SMD max 1.2, 12 max incl. header
TFT_MOD_R = 19.0                                 # round PCB radius (pocket gets +0.4)
TFT_TAB = (22.92, 26.5)                          # tab width, tab end distance from the display centre
TFT_FRONT_T, TFT_PCB_T, TFT_SMD_H = 1.6, 1.6, 1.2
TFT_BL = (35.6, 20.3)                            # glass/backlight width, reach on the tab side (other side 17.8)
# OLED: 0.96" 128x64 SSD1306 yellow/blue, datasheet "OLED 4 Pin 0.96 inch" outer dimension drawing (owner: datasheet values are right).
# Normal orientation (header at top): board 27.3 sq x 1.0, glass 26.7 x 19.26 starting 4.02 below the board top, centred
# left-right; pixel area 21.74 x 10.86 starting 2.44 below the glass top, so a 5.96 mm clear FPC ledge below it; 2.37 total
# thickness; 4-pin header on the back 1.65 from the top edge (pitch 2.54, centred), short pin ends 1.5 (0.5 stub on the front);
# rear components 1.5 max. Yellow = top 16 rows.
OLED91_BOARD = (25.0, 25.0, 1.0)                 # owner's board measured 25 x 25 (the datasheets say 27.3 to 27.7: his JMD0.96D-1 is smaller)
OLED91_SCREEN = (24.5, 17.0)                     # glass measured 24.5 x 17.0 (incl. the 3 mm black non-pixel strip); print 1 pocket was 27.7 wide, too loose
OLED91_GLASS_T = 2.37 - 1.0
OLED_AA = (21.74, 10.86); OLED_AA_TOP = 2.44     # pixel area and its distance from the glass edge on the header side
OLED_GLASS_EDGE = (27.7 - 19.26) / 2             # board edge to glass edge: glass is centred top-bottom on the board in both drawings (4.02 on 27.3)
OLED_PIN_IN, OLED_PIN_SPAN = 1.65, 7.62          # header row from the board edge, pin 1 to pin 4
OLED_FLIP = True                                 # mounted upside down: header at the bottom, the clear FPC ledge tucks up behind the
                                                 # round display. The yellow band ends up at the BOTTOM of the window (firmware rotates 180)
# 9 Oct, print 1: owner measured header pin row to glass edge = 3.0 mm (datasheet implied about 2.4 to 2.6)
OLED_PIN_TO_GLASS = 3.0
OLED_GLASS_EDGE = OLED_PIN_IN + OLED_PIN_TO_GLASS     # 4.65 board edge (pin side) to glass edge
# black light shields: skirts on the back of the black bezel line the display windows down to each glass, so no white plastic
# (lit by the LEDs) shows around the pictures, and the OLED's non-pixel glass strip stays hidden
SKIRT_T = 0.8                                    # skirt wall
SKIRT_CLEAR = 0.2                                # skirt to carrier window, per side
TFT_SHOW_R = 16.5                                # visible round picture radius (pixel area 16.2)
OLED_LEDGE_EXTRA = 3.0                           # the OLED glass is two layers: the lower substrate runs on past the cover glass to the ribbon; pocket +3 mm on that side in case it is longer than the 17 mm measured
OLED_STUB_RELIEF = 2.6                           # 9 Oct, print 1: the header pin tips stick out about 2 mm on the board FRONT and lifted the board (relief was 1.0)
OLED_RING_MARGIN = 0.5                           # glass corners stay this far inside the ring hole
OLED_LIFT = 1.0                                  # 9 Oct: OLED moved 1 mm toward the display so its pin row (3.0 from the glass) stays inside the ring hole
OLED91_WIN = (24.3, 13.0)                        # 9 Oct, owner: visible OLED opening AFTER the bezel (Noah's look, fitted to the 24.5 glass)
OLED_WIN_TOP_MARGIN = 0.3                        # window top sits 0.3 above the pixels (the round display above leaves no more room); the extra height goes below
BEZ_DROP_TOP = float(os.environ.get("BEZ_DROP_TOP", 0.8))   # 9 Oct, print 2: bezel OLED window top edge lowered by this much to hide the round display's board edge (pixels start below it)
BEZ_DROP_BOT = min(0.4, BEZ_DROP_TOP)            # bottom edge follows by up to 0.4 (bezel-only change: skirt bottom wall 0.5 still fits the carrier cut)
SKIRT_TOP_FULL = True                            # top wall of the OLED skirt goes all the way down to the OLED glass (blocks angled views of the board edge / white carrier)
SKIRT_BOT_T = 0.5                                # thinner bottom wall of the OLED skirt so it can move down 0.4 inside the same carrier cut
SKIRT_SIDE_T = 0.5                               # thinner side walls of the OLED skirt so the carrier keeps the snug 25.5 glass pocket
TFT_MOD_T = TFT_FRONT_T + TFT_PCB_T              # 3.2: glass front to PCB back (datasheet)
RING_PCB_T = 1.2                                 # measured 9 Oct: bare ring PCB 1.2, ring 3.0 overall incl. LEDs
RING_LED_H = 1.8                                 # 3.0 total - 1.2 PCB
RING_SEAT_Z = -2.5                               # ring PCB front face; LEDs (1.8 tall) end 0.7 mm behind the carrier back,
                                                 # 2.7 mm of air + 1 mm white skin in front of them
BACK_T = -RING_SEAT_Z + RING_PCB_T               # back layer of the carrier: ring back flush with its floor (3.7 mm)
BEZEL_PEG_ANG = (20.0, 160.0, 215.0, 325.0) if SUBDIAL else (90.0, 210.0, 330.0)
if SUBDIAL:
    TFT_HOLE_DY = 20.67                          # module mounted upside down: tab at the top, OLED below (firmware rotates 180)
    TFT_HOLE_DX = 19.26 / 2
    GLASS_POCKET_T = 2.1                             # display glass pocket depth (0.9 mm lip): the display's rear parts (to z -2.3)
                                                     # clear the OLED glass ledge tucked behind it (glass front z -2.33)
    GLASS_POCKET_D = 36.0                            # backlight 35.6 + 0.2 a side: keeps the round picture centred on the window
    # glass position: bottom corners on a circle OLED_RING_MARGIN inside the ring hole
    _gb = -np.sqrt((RING_ID / 2 - OLED_RING_MARGIN) ** 2 - (26.7 / 2) ** 2) + OLED_LIFT   # glass bottom edge (-31.9), position kept from the 26.7 mm datasheet layout
    OLED_GLASS_DY = _gb + OLED91_SCREEN[1] / 2                                                     # glass centre
    if OLED_FLIP:
        OLED_AA_DY = _gb + OLED_AA_TOP + OLED_AA[1] / 2                                            # pixel-area centre
        OLED_SCR_DY = OLED_AA_DY + OLED_AA[1] / 2 + OLED_WIN_TOP_MARGIN - OLED91_WIN[1] / 2      # window centre
        OLED_BOARD_DY = _gb - OLED_GLASS_EDGE + OLED91_BOARD[1] / 2
        OLED_PIN_DY = _gb - OLED_GLASS_EDGE + OLED_PIN_IN
    else:
        _gt = _gb + OLED91_SCREEN[1]
        OLED_SCR_DY = _gt - OLED_AA_TOP - OLED_AA[1] / 2
        OLED_BOARD_DY = _gt + OLED_GLASS_EDGE - OLED91_BOARD[1] / 2
        OLED_PIN_DY = _gt + OLED_GLASS_EDGE - OLED_PIN_IN
    OLEDS = [(cx, cy + OLED_BOARD_DY) for (cx, cy) in (POD1, POD2)]      # board centres
    OLED_SCR = [(cx, cy + OLED_SCR_DY) for (cx, cy) in (POD1, POD2)]     # pixel-area / window centres
    OLED_GLASS = [(cx, cy + OLED_GLASS_DY) for (cx, cy) in (POD1, POD2)]  # glass centres
    OLED_PINS = [(cx, cy + OLED_PIN_DY) for (cx, cy) in (POD1, POD2)]     # header row centres
DISP = [(cx, cy + (DISP_DY if SUBDIAL else 0.0)) for (cx, cy) in (POD1, POD2)]
# ----------------------------------------

def cyl(d, h, at=(0, 0, 0), axis="z"):
    c = cylinder(radius=d / 2, height=h, sections=SEG)
    if axis == "x": c.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    if axis == "y": c.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
    c.apply_translation(at); return c
def bx(size, at):
    b = box(extents=size); b.apply_translation(at); return b
def union(parts):
    m = parts[0]
    for p in parts[1:]: m = m.union(p, engine="manifold")
    return m
def diff(a, parts):
    for p in parts: a = a.difference(p, engine="manifold")
    return a
def rrect(x0, y0, x1, y1, r, t, z0):
    w, h = x1 - x0, y1 - y0; cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    core = [bx((w - 2 * r, h, t), (cx, cy, z0 + t / 2)), bx((w, h - 2 * r, t), (cx, cy, z0 + t / 2))]
    corners = [cyl(2 * r, t, (x, y, z0 + t / 2)) for x in (x0 + r, x1 - r) for y in (y0 + r, y1 - r)]
    return union(core + corners)

def tilt_keep(y_offset=0.0):
    """Half-space y >= tan(TILT) * (front - z) + y_offset, i.e. everything above the sloped bottom plane that passes
    through the cover's front-bottom edge. Returned as a big box; intersect with it."""
    front = PLATE_T + COVER_T
    b = box(extents=(600, 600, 600)); b.apply_translation((W / 2, 300, 0))
    b.apply_transform(trimesh.transformations.rotation_matrix(np.deg2rad(TILT), [1, 0, 0], point=(0, 0, front)))
    b.apply_translation((0, y_offset, 0))
    return b

def inv_cone(R, apex_z, cx=0.0, cy=0.0):
    """45 degree cone whose radius grows with z, apex at (cx, cy, apex_z). Used for chamfers that open toward the front."""
    c = trimesh.creation.cone(radius=R, height=R, sections=SEG)
    c.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0]))     # apex now at z = -R, base at z = 0
    c.apply_translation((cx, cy, apex_z + R)); return c

def eight2d(inset=0.0):
    """Figure-8 / binocular outline in 2D (shapely): pods + waist with concave fillets tangent to both pods, offset inward by inset."""
    from shapely.geometry import Point, Polygon
    yf = WAIST_HH + _RF                                                  # fillet centres at (MX, CY +/- yf)
    d = np.hypot(PITCH / 2, yf); ux, uy = (PITCH / 2) / d, yf / d          # unit vector pod1 centre -> top fillet centre
    t1 = (POD1[0] + POD_R * ux, CY + POD_R * uy); t2 = (POD2[0] - POD_R * ux, CY + POD_R * uy)
    kite = Polygon([POD1, t1, t2, POD2, (t2[0], 2 * CY - t2[1]), (t1[0], 2 * CY - t1[1])])
    shape = Point(POD1).buffer(POD_R, 128).union(Point(POD2).buffer(POD_R, 128)).union(kite)
    shape = shape.difference(Point(MX, CY + yf).buffer(_RF, 64)).difference(Point(MX, CY - yf).buffer(_RF, 64))
    return shape.buffer(-inset, 64) if inset else shape

def outline(t, z0, inset=0.0):
    """Twin pods + bridge + plinth, extruded t along z from z0, shrunk by inset on every side."""
    if OUTLINE == "eight":
        m = trimesh.creation.extrude_polygon(eight2d(inset), t); m.apply_translation((0, 0, z0)); return m
    r = POD_R - inset
    pods = [cyl(2 * r, t, (POD1[0], POD1[1], z0 + t / 2)), cyl(2 * r, t, (POD2[0], POD2[1], z0 + t / 2))]
    bridge = bx((POD2[0] - POD1[0], H - 2 * inset, t), ((POD1[0] + POD2[0]) / 2, H / 2, z0 + t / 2))   # full height, covers the base's top edge
    rb = BASE_R - inset
    base = [bx((W - 2 * BASE_R, CY - inset, t), (W / 2, inset + (CY - inset) / 2, z0 + t / 2)),
            bx((W - 2 * inset, CY - BASE_R, t), (W / 2, BASE_R + (CY - BASE_R) / 2, z0 + t / 2)),
            cyl(2 * rb, t, (BASE_R, BASE_R, z0 + t / 2)), cyl(2 * rb, t, (W - BASE_R, BASE_R, z0 + t / 2))]
    base[0] = bx((W - 2 * BASE_R, CY - inset, t), (W / 2, inset + (CY - inset) / 2, z0 + t / 2))
    horn = rrect(MX - HORN_HW + inset, HORN_Y0, MX + HORN_HW - inset, HORN_TOP - inset, HORN_R - inset, t, z0)
    parts = pods + [bridge] + base + ([] if SUBDIAL else [horn])
    if "left" in STRAIGHT_ENDS:
        parts.append(rrect(inset, inset, POD1[0], 2 * POD_R - inset, END_R - inset if END_R > inset else 0.01, t, z0))
    if "right" in STRAIGHT_ENDS:
        parts.append(rrect(POD2[0], inset, 2 * POD_R + PITCH - inset, 2 * POD_R - inset, END_R - inset if END_R > inset else 0.01, t, z0))
    if PLINTH:
        x0, y0, x1, y1 = PLINTH
        parts.append(rrect(x0 + inset, y0 + inset, x1 - inset, y1 - inset, PLINTH_R, t, z0))
    return union(parts)

def carrier():
    plate = outline(PLATE_T, 0)
    adds, cuts = [], []
    for (pc, (cx, cy)) in zip((POD1, POD2), DISP):
        cuts.append(cyl(2 * (TFT_SHOW_R + SKIRT_T + SKIRT_CLEAR) if SUBDIAL else TFT_WIN_D, PLATE_T * 3, (cx, cy, PLATE_T / 2)))
        if not SUBDIAL:                                                                      # sub-dial: the bezel ring carries the chamfer
            cone = inv_cone(TFT_WIN_D / 2 + WIN_CHAMFER + 5, PLATE_T - WIN_CHAMFER - TFT_WIN_D / 2, cx, cy)   # 45 degree chamfer opening toward the front
            cuts.append(cone.intersection(bx((60, 60, 10), (cx, cy, PLATE_T - WIN_CHAMFER + 5)), engine="manifold"))
        cuts.append(cyl(GLASS_POCKET_D, GLASS_POCKET_T * 2, (cx, cy, 0)))                      # glass pocket on the back face
        for sx in ((-1, 1) if not SUBDIAL else ()):                                         # sub-dial: display is hot-glued in its pocket
            px, py = cx + sx * TFT_HOLE_DX, cy + TFT_HOLE_DY
            adds.append(cyl(5.0, BOSS_H, (px, py, -BOSS_H / 2)))
            cuts.append(cyl(INSERT_D, BOSS_H + 1, (px, py, -BOSS_H / 2)))
            bx_, by_ = cx + sx * TFT_BAR_BOSS[0], cy + TFT_BAR_BOSS[1]                         # retainer bar bosses above the display
            adds.append(cyl(5.0, BOSS_H, (bx_, by_, -BOSS_H / 2)))
            cuts.append(cyl(1.7, BOSS_H + 1, (bx_, by_, -BOSS_H / 2)))
        cx, cy = pc
        glow = cyl(GLOW_OD, (PLATE_T - SKIN) * 2, (cx, cy, 0)).difference(cyl(GLOW_ID, (PLATE_T - SKIN) * 2 + 2, (cx, cy, 0)), engine="manifold")
        cuts.append(glow)                                                                    # thin the carrier to SKIN over the ring
        for k in (range(3) if not SUBDIAL else ()):                                          # posts inside the ring (between display and ring) with a shelf the ring's inner edge rests on
            a = np.deg2rad(90 + 120 * k); r = RING_ID / 2 - 2.6
            px, py = cx + r * np.cos(a), cy + r * np.sin(a)
            adds.append(cyl(4.0, RING_GAP + 4.0, (px, py, -(RING_GAP + 4.0) / 2)))
            sh = bx((4.0, 3.0, 1.5), (0, 0, 0)); sh.apply_transform(trimesh.transformations.rotation_matrix(a, [0, 0, 1]))
            sh.apply_translation((cx + (r + 2.8) * np.cos(a), cy + (r + 2.8) * np.sin(a), -(RING_GAP + 3.5 + 0.75)))
            adds.append(sh)
    if SUBDIAL:   # back layer with the ring pit, display pocket, OLED floor + glass pocket, and the OLED window
        back = outline(BACK_T, -BACK_T, inset=WALL + 0.25)
        adds.append(back)
        for (pc, dc, sc, gc, pn) in zip((POD1, POD2), DISP, OLED_SCR, OLED_GLASS, OLED_PINS):
            cx, cy = pc
            seat = cyl(RING_OD + 0.6, RING_SEAT_Z + BACK_T + 1.0, (cx, cy, (RING_SEAT_Z - BACK_T - 1.0) / 2)).difference(cyl(RING_SEAT_ID, 2 * BACK_T + 4, (cx, cy, -BACK_T)), engine="manifold")
            cuts.append(seat)                                                                # ring PCB seat: from the floor up to RING_SEAT_Z
            led = cyl(LED_CH_OD, -RING_SEAT_Z + 0.02, (cx, cy, RING_SEAT_Z / 2)).difference(cyl(LED_CH_ID, -RING_SEAT_Z + 2, (cx, cy, RING_SEAT_Z / 2)), engine="manifold")
            cuts.append(led)                                                                 # LED channel in front of the ring, joins the glow pocket
            dx_, dy_ = dc
            zt = max(0.0, GLASS_POCKET_T - TFT_FRONT_T) + 0.1                                # module PCB front sits this far into the plate
            cuts.append(cyl(2 * TFT_MOD_R + 0.8, BACK_T + zt + 0.02, (dx_, dy_, (zt - BACK_T) / 2)))   # display module pocket
            tab0, tab1 = TFT_MOD_R - 5.0, TFT_TAB[1] + 0.6
            cuts.append(bx((TFT_TAB[0] + 1.0, tab1 - tab0, BACK_T + zt + 0.02), (dx_, dy_ + (tab0 + tab1) / 2, (zt - BACK_T) / 2)))   # its tab (top), full length
            cuts.append(bx((TFT_BL[0] - 10.0, 6.0, GLASS_POCKET_T * 2), (dx_, dy_ + TFT_BL[1] - 2.5, 0)))   # glass pocket relief for the FPC ledge
            sx_, sy_ = sc
            ext = OLED_LEDGE_EXTRA * (1 if OLED_FLIP else -1)                                    # extra length on the ribbon (FPC) side for the thin lower glass step
            cuts.append(bx((OLED91_SCREEN[0] + 1.0, OLED91_SCREEN[1] + 1.0 + OLED_LEDGE_EXTRA, OLED91_GLASS_T + 0.3), (gc[0], gc[1] + ext / 2, -BACK_T + (OLED91_GLASS_T + 0.3) / 2 - 0.01)))   # glass pocket
            cuts.append(bx((OLED_PIN_SPAN + 3.0, 3.6, 2 * OLED_STUB_RELIEF), (pn[0], pn[1], -BACK_T)))   # relief for the header pin tips on the board front
            ex = 2 * (SKIRT_T + SKIRT_CLEAR); exs = 2 * (SKIRT_SIDE_T + 0.1)
            cuts.append(bx((OLED91_WIN[0] + exs, OLED91_WIN[1] + ex, BACK_T + PLATE_T + 2), (sx_, sy_, (PLATE_T - BACK_T) / 2)))   # window (+ room for the bezel's black skirt)
        for (sx, sy) in SCREWS:
            cuts.append(cyl(SCREW_D + 0.4, BACK_T * 3, (sx, sy, -BACK_T / 2)))
        for (sx, sy) in COVER_SCREWS:
            cuts.append(cyl(INSERT_D, BACK_T * 3, (sx, sy, -BACK_T / 2)))
    for (cx, cy) in (OLEDS if not SUBDIAL else ()):
        cuts.append(bx((OLED96_WIN[0], OLED96_WIN[1], PLATE_T * 3), (cx, cy + OLED96_WIN_OFF, PLATE_T / 2)))
        for sx in (-1, 1):
            for sy in (-1, 1):
                px, py = cx + sx * OLED96_HOLES[0] / 2, cy + sy * OLED96_HOLES[1] / 2
                adds.append(cyl(5.0, 3.0, (px, py, -1.5))); cuts.append(cyl(1.7, 5, (px, py, -1.5)))
    for (sx, sy) in SCREWS:                                                                  # carrier to body, hidden under the cover
        cuts.append(cyl(SCREW_D, PLATE_T * 3, (sx, sy, PLATE_T / 2)))
        cuts.append(cyl(SCREW_HEAD_D, SCREW_HEAD_H * 2, (sx, sy, PLATE_T)))
    for (sx, sy) in COVER_SCREWS:                                                            # 3 mm brass inserts, set from the front
        cuts.append(cyl(INSERT_D, PLATE_T * 3, (sx, sy, PLATE_T / 2)))
    for (cx, cy) in (POD1, POD2):                                                            # peg holes for the inner bezel ring (between display and ring)
        for ang in BEZEL_PEG_ANG:
            a = np.deg2rad(ang)
            cuts.append(cyl(BEZEL_PEG_D + 0.15, BEZEL_PEG_L + 0.6, (cx + BEZEL_PEG_R * np.cos(a), cy + BEZEL_PEG_R * np.sin(a), PLATE_T - BEZEL_PEG_L / 2)))
    for (cx, cy) in (POD1, POD2):                                                            # peg holes for the cover's thin outer band
        for k in range(COVER_PEG_N):
            a = np.deg2rad(22.5 + 360.0 * k / COVER_PEG_N)
            cuts.append(cyl(BEZEL_PEG_D + 0.15, BEZEL_PEG_L + 0.6, (cx + COVER_PEG_R * np.cos(a), cy + COVER_PEG_R * np.sin(a), PLATE_T - BEZEL_PEG_L / 2)))
    return diff(union([plate] + adds), cuts).intersection(tilt_keep(), engine="manifold")

def tft_retainer():
    """Flat bar screwed to the two bosses above each round display; its lower edge overlaps the PCB rim and holds it flat."""
    L, Wd, T = TFT_BAR
    bar = bx((L, Wd, T), (0, 0, T / 2))
    cuts = [cyl(2.3, T * 3, (sx * TFT_BAR_BOSS[0], 0, T / 2)) for sx in (-1, 1)]
    return diff(bar, cuts)

def cover():
    plate = outline(COVER_T, PLATE_T)
    cuts = []
    for (cx, cy) in (POD1, POD2):
        cuts.append(cyl(GLOW_OD, COVER_T * 3, (cx, cy, PLATE_T + COVER_T / 2)))               # one clean 66 mm hole; the inner bezel ring is a separate part
    for (cx, cy) in (OLEDS if not SUBDIAL else ()):
        cuts.append(bx((OLED96_WIN[0], OLED96_WIN[1], COVER_T * 3), (cx, cy + OLED96_WIN_OFF, PLATE_T + COVER_T / 2)))
    for (sx, sy) in COVER_SCREWS:                                                            # M2 x 6 wafer head, 0.8 mm counterbore
        cuts.append(cyl(SCREW_D, COVER_T * 3, (sx, sy, PLATE_T + COVER_T / 2)))
        cuts.append(cyl(4.4, 1.6, (sx, sy, PLATE_T + COVER_T)))
    adds = []
    for (cx, cy) in (POD1, POD2):                                                            # pegs on the back of the outer band
        for k in range(COVER_PEG_N):
            a = np.deg2rad(22.5 + 360.0 * k / COVER_PEG_N)
            adds.append(cyl(BEZEL_PEG_D, BEZEL_PEG_L + 0.1, (cx + COVER_PEG_R * np.cos(a), cy + COVER_PEG_R * np.sin(a), PLATE_T - BEZEL_PEG_L / 2 + 0.05)))
    return diff(union([plate] + adds), cuts).intersection(tilt_keep(), engine="manifold")

def bezel_ring():   # black inner ring, 2 mm, with 3 pegs on the back; press into the carrier, drop of glue optional. Inner edge chamfered 45 degrees.
    inner_back = TFT_WIN_D + 2 * (0.6 if SUBDIAL else WIN_CHAMFER)   # sub-dial: smaller chamfer keeps a bridge above the OLED window
    dy0 = DISP_DY if SUBDIAL else 0.0
    if SUBDIAL:   # straight bore 0.3 outside the display window, 1 mm 45 degree chamfer at the front: keeps a 1 mm bridge above the OLED window
        rb = TFT_SHOW_R
        cone = union([cyl(2 * rb, COVER_T * 3, (0, dy0, COVER_T / 2)), inv_cone(rb + 3, COVER_T - 1.0 - rb, 0.0, dy0)])
    else:
        cone = inv_cone(inner_back / 2 + COVER_T + 2, -inner_back / 2, 0.0, dy0)        # ID = inner_back at the back face, +2 x COVER_T at the front
    r = cyl(BEZEL_OD, COVER_T, (0, 0, COVER_T / 2)).difference(cone, engine="manifold")
    _wt = OLED_SCR_DY + OLED91_WIN[1] / 2 - BEZ_DROP_TOP; _wb = OLED_SCR_DY - OLED91_WIN[1] / 2 - BEZ_DROP_BOT
    _bwh, _bcy = _wt - _wb, (_wt + _wb) / 2                                                 # bezel's OLED window (may sit lower than the carrier's)
    if SUBDIAL:                                                                              # OLED window: straight cut, exactly OLED91_WIN at the front face (no step)
        r = r.difference(bx((OLED91_WIN[0], _bwh, COVER_T * 3), (0, _bcy, COVER_T / 2)), engine="manifold")
    ticks = []
    for k in (range(60) if BEZEL_TICKS != "none" else ()):                               # through-cut marks: the white carrier shows behind them
        a = 2 * np.pi * k / 60
        w, l, rc = TICK_QTR if k % 15 == 0 else TICK_HOUR if k % 5 == 0 else TICK_MIN
        if SUBDIAL and abs(k - 30) <= 3: continue                                            # no ticks behind the OLED window (6 o'clock)
        if BEZEL_TICKS == "quarters" and k % 15: continue
        t = bx((w, l, COVER_T * 3), (0, 0, COVER_T / 2))
        t.apply_transform(trimesh.transformations.rotation_matrix(a, [0, 0, 1]))            # tick lies along the radius at angle a from 12 o'clock
        t.apply_translation((rc * np.sin(a), rc * np.cos(a), 0)); ticks.append(t)
    if BEZEL_PIP: ticks.append(bx((BEZEL_PIP[0], BEZEL_PIP[1], COVER_T * 3), (0, BEZEL_PIP[2], COVER_T / 2)))   # 12 o'clock slot for the red plug
    r = diff(r, ticks)
    if SUBDIAL and SKIRT_T:   # black skirts down to the glass (bezel local z: 0 = carrier front face)
        zt = GLASS_POCKET_T - PLATE_T                                                          # round display glass front (-0.9)
        sk = cyl(2 * (TFT_SHOW_R + SKIRT_T), -zt, (0, dy0, zt / 2)).difference(cyl(2 * TFT_SHOW_R, -zt + 2, (0, dy0, zt / 2)), engine="manifold")
        zo = RING_SEAT_Z - RING_PCB_T + OLED91_GLASS_T - PLATE_T                               # OLED glass front (-5.33)
        zo_top = (GLASS_POCKET_T - TFT_FRONT_T + 0.2) - PLATE_T                                # top side stops above the display PCB (-2.3)
        ww, wh = OLED91_WIN
        wt, wb = _bcy + _bwh / 2, _bcy - _bwh / 2
        o = bx((ww + 2 * SKIRT_SIDE_T, wt + SKIRT_T - (wb - SKIRT_BOT_T), -zo), (0, (wt + SKIRT_T + wb - SKIRT_BOT_T) / 2, zo / 2)).difference(bx((ww, _bwh, -zo + 2), (0, _bcy, zo / 2)), engine="manifold")
        cut_top = bx((ww + 2 * SKIRT_T + 2, SKIRT_T + 0.02, zo_top - zo + 0.01), (0, wt + SKIRT_T / 2, (zo + zo_top) / 2 - 0.005))
        if SKIRT_TOP_FULL:   # 9 Oct: top wall runs full depth to the OLED glass, with a notch only where the round display's board (r 19) and glass pass through
            zpb = GLASS_POCKET_T - TFT_FRONT_T - TFT_PCB_T - PLATE_T                         # display PCB back (-4.1)
            o = o.difference(cyl(2 * (TFT_MOD_R + 0.3), zt - zo + 0.2, (0, dy0, (zt + zo - 0.2) / 2)), engine="manifold")   # notch runs to the tip: the round display goes in from the back AFTER the bezel
        else:
            o = o.difference(cut_top, engine="manifold")
            o = o.difference(cyl(GLASS_POCKET_D, zt - zo_top + 0.02, (0, dy0, (zt + zo_top) / 2)), engine="manifold")   # top wall clears the round glass
        r = union([r, sk, o])
    pegs = [cyl(BEZEL_PEG_D, BEZEL_PEG_L, (BEZEL_PEG_R * np.cos(np.deg2rad(a)), BEZEL_PEG_R * np.sin(np.deg2rad(a)), -BEZEL_PEG_L / 2)) for a in BEZEL_PEG_ANG]
    return union([r] + pegs)

def clamp_plate():
    """Per-dial clamp plate in pod-local coordinates (x, y about the pod centre), z from -BACK_T - CLAMP_T to -BACK_T,
    with the TFT press arcs rising to the display PCB back. Print back face down, 2 off."""
    z0, z1 = -BACK_T - CLAMP_T, -BACK_T
    p = cyl(2 * CLAMP_R, CLAMP_T, (0, 0, (z0 + z1) / 2))
    cuts, adds = [], []
    ox, oy = 0.0, OLED_BOARD_DY                                                              # OLED board (pod local)
    bw, bh, bt = OLED91_BOARD
    cuts.append(bx((bw + 0.4, bh + 0.4, bt * 2), (ox, oy, z1)))                             # pocket as deep as the board: plate face meets the board back
    # window over everything on the board's back (rear parts, FPC fold + pads, header + wires): the plate presses only two
    # side strips of CLAMP_OLED_STRIP along the board's left and right edges (clear of the FPC and the parts)
    cuts.append(bx((bw - 2 * CLAMP_OLED_STRIP, bh + 1.2, CLAMP_T * 3), (ox, oy, (z0 + z1) / 2)))
    ww, wr = CLAMP_WIRE_WIN                                                                  # TFT header + ring wire joint at 12 o'clock
    cuts.append(bx((ww, CLAMP_R - wr + 2.0, CLAMP_T * 3), (0, (wr + CLAMP_R + 2.0) / 2, (z0 + z1) / 2)))
    for a in STANDOFF_ANG:                                                                   # carrier screws pass through
        cuts.append(cyl(BACK_SCREW[0] if BACK_SCREW else SCREW_D + 0.3, CLAMP_T * 3, (STANDOFF_R * np.cos(np.deg2rad(a)), STANDOFF_R * np.sin(np.deg2rad(a)), (z0 + z1) / 2)))
    zt = GLASS_POCKET_T - TFT_FRONT_T - TFT_PCB_T                                            # display PCB back (-1.1)
    arc = cyl(2 * 18.8, zt - z1, (0, DISP_DY, (z1 + zt) / 2)).difference(cyl(2 * 17.6, zt - z1 + 1, (0, DISP_DY, (z1 + zt) / 2)), engine="manifold")
    keep = union([bx((20.0, 2 * 18.8 * np.sin(np.deg2rad(40.0)), zt - z1 + 2), (sx * 15.0, DISP_DY, (z1 + zt) / 2)) for sx in (-1, 1)])
    adds.append(arc.intersection(keep, engine="manifold"))                                   # 3 and 9 o'clock arcs, +/- 40 degrees
    return diff(union([p] + adds), cuts)

def pip_plug():   # red 12 o'clock plug for the bezel slot; print 4 (2 + spares), top face down for a clean surface
    w, l = BEZEL_PIP[0] - 2 * PIP_CLEAR, BEZEL_PIP[1] - 2 * PIP_CLEAR
    return bx((w, l, COVER_T), (0, 0, COVER_T / 2))

def status_diffuser():
    return bx((STATUS_DIFF[0], STATUS_DIFF[1], STATUS_DIFF[2]), (0, 0, STATUS_DIFF[2] / 2))

def knob():
    """Fluted knob, modelled upright (shaft axis z, top at z = KNOB_H). Print it upside down: top face on the bed."""
    body = cyl(KNOB_D, KNOB_H - 1.0, (0, 0, (KNOB_H - 1.0) / 2))
    cone = trimesh.creation.cone(radius=KNOB_D / 2, height=KNOB_D / 2, sections=SEG)              # 45 degree cone from the rim: its top 1 mm is the edge chamfer
    cone.apply_translation((0, 0, KNOB_H - 1.0))
    body = body.union(cone.intersection(bx((KNOB_D + 2, KNOB_D + 2, 1.0), (0, 0, KNOB_H - 0.5)), engine="manifold"), engine="manifold")
    cuts = []
    for k in range(KNOB_FLUTES):                                                                     # grooves stop 2 mm below the top
        a = 2 * np.pi * k / KNOB_FLUTES
        cuts.append(cyl(KNOB_FLUTE_D, KNOB_H, (KNOB_D / 2 * np.cos(a), KNOB_D / 2 * np.sin(a), KNOB_H / 2 - 2.0)))
    z_d = KNOB_BORE_DEPTH - KNOB_D_LEN
    bore_round = cyl(KNOB_BORE_D, z_d + 0.01, (0, 0, z_d / 2))                                       # plain round bore from the pocket up to the D section
    bore_d = cyl(KNOB_BORE_D, KNOB_D_LEN, (0, 0, z_d + KNOB_D_LEN / 2))
    flat_cut = bx((KNOB_BORE_D, KNOB_BORE_D, KNOB_D_LEN + 1), (0, KNOB_BORE_FLAT, z_d + KNOB_D_LEN / 2))
    bore_d = bore_d.difference(flat_cut, engine="manifold")                                          # D only at the top: flat on the +y side
    cuts.append(bore_round); cuts.append(bore_d)
    cuts.append(cyl(KNOB_NUT_RELIEF[0], KNOB_NUT_RELIEF[1] * 2, (0, 0, 0)))                          # pocket over the panel nut
    if KNOB_POINTER:
        cuts.append(bx((1.6, KNOB_D / 2 - 2.0, 1.2), (0, -(KNOB_D / 4 + 0.5), KNOB_H)))               # pointer groove opposite the D flat
    return diff(body, cuts)

def body():
    fz = PLATE_T + COVER_T if BODY_SLEEVE else 0.0                                        # body front edge: flush with the cover front when the sleeve is on
    grow = -(SLEEVE_T + SLEEVE_CLEAR) if BODY_SLEEVE else 0.0
    outer = outline(DEPTH + fz, -DEPTH, inset=grow).intersection(tilt_keep(), engine="manifold")
    inner = outline(DEPTH, -DEPTH + WALL, inset=WALL).intersection(tilt_keep(WALL / np.cos(np.deg2rad(TILT))), engine="manifold")   # open toward the plate, sloped floor of full wall thickness
    shell = outer.difference(inner, engine="manifold")
    adds, cuts = [], []
    if BODY_SLEEVE:   # 10 Oct: the front unit (carrier + cover) drops into a pocket in the body's front, so no white shows from the side
        cuts.append(outline(fz + 2.0, 0.0, inset=-SLEEVE_CLEAR))                            # pocket: the carrier rests on the old rim at z 0
        for kx in KNOB_X:                                                                    # spot face under each knob keeps the roof at WALL for the encoder nut
            cuts.append(cyl(KNOB_SPOT_D, 2 * (SLEEVE_T + SLEEVE_CLEAR), (kx, H + SLEEVE_T + SLEEVE_CLEAR, KNOB_Z), axis="y"))
    zb = -DEPTH + WALL
    for (sx, sy) in SCREWS:                                                                  # full-length posts from the back wall up to the carrier
        top = (-BACK_T - (CLAMP_T if CLAMP else 0.2)) if SUBDIAL else 0.0                    # sub-dial: posts end under the carrier's back layer (clamp plate between)
        if SUBDIAL and CLAMP and BACK_SCREW:                                                 # tube post, screw from the back
            bore, hd, cbd, _ = BACK_SCREW
            adds.append(cyl(hd + 2.4, top - zb, (sx, sy, (zb + top) / 2)))
            cuts.append(cyl(bore, DEPTH + 2, (sx, sy, -DEPTH / 2)))
            cuts.append(cyl(hd, cbd * 2, (sx, sy, -DEPTH)))                                   # head counterbore from the back face
        else:
            adds.append(cyl(BOSS_D + 1, top - zb, (sx, sy, (zb + top) / 2))); cuts.append(cyl(INSERT_D, 8, (sx, sy, top - 4)))
    for (sx, sy) in COVER_SCREWS:                                                            # relief in the body rim: a 4 mm insert in the 3 mm carrier pokes 1 mm out the back
        cuts.append(cyl(INSERT_D + 0.6, 3.0, (sx, sy, 0)))
        if COVER_SCREW_LUG:   # 10 Oct, owner: the 2 front screws also bite into the body. Lug off the waist wall just behind the carrier,
            from shapely.geometry import Polygon                                             # 45 degree gusset underneath (prints back-down without support), clear of the LiPo
            d = 1.0 if sy > CY else -1.0
            y0, y1 = sy - d * 3.5, sy + d * 7.0                                              # inner face of the lug .. into the waist wall
            zt_, zl_ = -BACK_T - 0.1, -BACK_T - 6.1                                          # under the carrier's back layer (z -3.7)
            lug = bx((8.0, abs(y1 - y0), zt_ - zl_), (sx, (y0 + y1) / 2, (zt_ + zl_) / 2))
            g = trimesh.creation.extrude_polygon(Polygon([(y0, zl_), (y1, zl_), (y1, zl_ - abs(y1 - y0))]), 8.0)   # (y, z) triangle, extruded along x
            g.apply_transform(np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]], float)); g.apply_translation((sx - 4.0, 0, 0))
            lug = union([lug, g]).intersection(bx((10.0, 30.0, 20.0), (sx, sy, COVER_LUG_FLOOR + 10.0)), engine="manifold")   # stops above the LiPo (top at z -14.5)
            adds.append(lug)
            cuts.append(cyl(COVER_SCREW_PILOT, 9.0, (sx, sy, zt_ - 4.5 + 0.2)))                # M2 thread-forming pilot, 8.8 deep below the carrier
    # S3 tray under the left pod: side rails, one end stop (the left wall is the other), corner posts (pins hang free)
    inner_L, inner_W = BOARD_L + 2 * CLEAR, BOARD_W + 2 * CLEAR
    tx, ty = TRAY_POS
    for sy in (-1, 1):
        adds.append(bx((inner_L + 1.6, 1.6, TRAY_WALL_H), (tx + 0.8, ty + sy * (inner_W / 2 + 0.8), zb + TRAY_WALL_H / 2)))
    adds.append(bx((1.6, 8, TRAY_WALL_H), (tx + inner_L / 2 + 0.8, ty + inner_W / 2 - 4, zb + TRAY_WALL_H / 2)))   # end stop away from the wall
    for sx in (-1, 1):
        for sy in (-1, 1):
            adds.append(bx((4, 3.5, PIN_DROP), (tx + sx * (inner_L / 2 - 2), ty + sy * (inner_W / 2 - 1.75), zb + PIN_DROP / 2)))
    # the two USB-C ports exit through the straight left wall: slot centred on the connector height (PCB top + 1.6)
    usb_z = zb + PIN_DROP + 1.6 + 1.6
    if USB_EXIT == "side":
        cuts.append(bx((2 * USB_FACET_X, USB_FACET[0], USB_FACET[1]), (0, ty, usb_z)))          # flat facet in the round end
        cuts.append(bx((2 * (USB_FACET_X + USB_WALL + 0.5), USB_SLOT[0], USB_SLOT[1]), (0, ty, usb_z)))   # port slot through the facet wall
    else:   # female panel end of a USB-C extension, glued into a sleeve on the inner back wall; the receptacle face shows at the back
        ux_, uy_ = USB_BACK; pw, ph, pd = USB_BACK_POCKET
        adds.append(bx((pw + 3.2, ph + 3.2, pd), (ux_, uy_, zb + pd / 2)))                   # sleeve block
        cuts.append(bx((pw, ph, pd + 0.02), (ux_, uy_, zb + pd / 2 + 0.01)))                  # housing pocket
        hole = rrect(ux_ - USB_BACK_HOLE[0] / 2, uy_ - USB_BACK_HOLE[1] / 2, ux_ + USB_BACK_HOLE[0] / 2, uy_ + USB_BACK_HOLE[1] / 2, 1.4, WALL * 3, -DEPTH - WALL)
        cuts.append(hole)
    if SUBDIAL and CLAMP:   # ledges on the inner wall press the clamp plate rim; 45 degree chamfer underneath (prints without support)
        zl = -BACK_T - CLAMP_T
        for (cx, cy), outer in ((POD1, 180.0), (POD2, 0.0)):
            for da in CLAMP_LEDGES:
                for sg in (-1, 1):
                    a = np.deg2rad(outer + sg * da)
                    led = bx((3.6, 8.0, 3.0), (0, 0, 0))
                    ch = trimesh.creation.extrude_polygon(__import__("shapely.geometry", fromlist=["Polygon"]).Polygon([(-1.8, 0), (1.8, 0), (1.8, -3.6)]), 8.0)
                    ch.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0])); ch.apply_translation((0, 4.0, 0))
                    ch.apply_translation((0, 0, -1.5))
                    piece = union([led, ch]); piece.apply_translation((POD_R - WALL - 1.6 + 0.6, 0, zl - 1.5))
                    piece.apply_transform(trimesh.transformations.rotation_matrix(a, [0, 0, 1], point=(0, 0, 0)))
                    piece.apply_translation((cx, cy, 0)); adds.append(piece)
    # controls on the roof, one above each dial
    for kx in KNOB_X:
        cuts.append(cyl(KNOB_HOLE, 20, (kx, H, KNOB_Z), axis="y"))
        cuts.append(bx((2.4, 20, 1.6), (kx, H, KNOB_Z + KNOB_HOLE / 2 + 0.3)))     # anti-rotation notch toward the front
    # piezo: clip ring on the inner back wall of the bridge + 5 sound holes through the back wall
    if PIEZO:
        px, py = PIEZO
        clip = cyl(16.0, 5, (px, py, zb + 2.5)).difference(cyl(12.4, 7, (px, py, zb + 2.5)), engine="manifold")
        adds.append(clip)
        for i in range(-2, 3): cuts.append(cyl(2.0, WALL * 3, (px + i * 4.0, py, -DEPTH)))
    if SPEAKER:   # locating ring for the driver (glue / foam gasket) + round grille through the back wall
        sx_, sy_, sd, _ = SPEAKER
        adds.append(cyl(sd + 3.0, 3.0, (sx_, sy_, zb + 1.5)).difference(cyl(sd + 0.6, 5.0, (sx_, sy_, zb + 1.5)), engine="manifold"))
        for gx in np.arange(-15.0, 15.1, 3.4):
            for gy in np.arange(-15.0, 15.1, 3.4):
                if gx * gx + gy * gy <= 15.5 ** 2: cuts.append(cyl(2.0, WALL * 3, (sx_ + gx, sy_ + gy, -DEPTH)))
    # vents in the back wall: over the S3 (left pod) and in the bridge
    def near_post(x, y, m=7.0): return any((x - sx) ** 2 + (y - sy) ** 2 < m * m for (sx, sy) in SCREWS)
    for i in range(-3, 4):
        for yy in (CY + 28, CY - 16):
            if not near_post(POD1[0] + i * 6, yy): cuts.append(bx((2.0, 8, WALL * 3), (POD1[0] + i * 6, yy, -DEPTH)))
    for i in (() if SPEAKER else range(-2, 3) if OUTLINE == "eight" else range(-4, 5)):
        for yy in ((CY + 16, CY - 16) if OUTLINE == "eight" else (CY + 24, CY - 16)):
            if not near_post(MX + i * 6, yy, 5.0 if OUTLINE == "eight" else 7.0): cuts.append(bx((2.0, 8 if OUTLINE == "eight" else 10, WALL * 3), (MX + i * 6, yy, -DEPTH)))
    if STATUS_BAR:   # slot through the bridge roof for the 1x8 stick, recess on top for the white diffuser strip
        cuts.append(bx((STATUS_SLOT[0], WALL * 3, STATUS_SLOT[1]), (STATUS_POS[0], H, STATUS_POS[1])))
        cuts.append(bx((STATUS_DIFF[0] + 0.4, STATUS_DIFF[2] * 2, STATUS_DIFF[1] + 0.4), (STATUS_POS[0], H, STATUS_POS[1])))
        for sx in (-1, 1):   # two small ledges inside for the stick to rest on (tape)
            adds.append(bx((4, 3, STATUS_SLOT[1] + 4), (STATUS_POS[0] + sx * (STATUS_SLOT[0] / 2 + 2), H - WALL - 4.5, STATUS_POS[1])))
    # feet: the sloped bottom is flat; stick 2 rubber bumpers under the pods
    return diff(union([shell] + adds), cuts).intersection(tilt_keep(), engine="manifold")

if __name__ == "__main__":
    os.makedirs(OUT_DIR, exist_ok=True)
    parts = {"v6_carrier_white.stl": carrier(), "v6_cover_black.stl": cover(), "v6_bezel_ring_black.stl": bezel_ring(), "v6_body.stl": body(), "v6_knob_red.stl": knob()}
    if BEZEL_PIP: parts["v6_pip_red.stl"] = pip_plug()
    if SUBDIAL and CLAMP: parts["v6_clamp_plate.stl"] = clamp_plate()
    if not SUBDIAL: parts["v6_tft_retainer.stl"] = tft_retainer()
    if STATUS_BAR: parts["v6_status_diffuser_white.stl"] = status_diffuser()
    for name, m in parts.items():
        m.export(os.path.join(OUT_DIR, name))
        print(f"{name}: {len(m.faces)} faces, watertight={m.is_watertight}, bbox={np.round(m.extents, 1)}")
