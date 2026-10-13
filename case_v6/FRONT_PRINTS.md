# Desk Plane Radar v6: front prints

Printer: Bambu A1 Mini (180 mm bed). Layer height 0.2 mm, PLA, no supports needed on any part.

| File | Qty | Filament | Orientation on the bed | Notes |
|---|---|---|---|---|
| v6_carrier_white.stl | 1 | White PLA (matte) | Front face DOWN, flat on the bed, turned 45 deg around Z (diagonal on the bed, not tilted): 180 mm part, 153.9 mm footprint | The 1 mm skin over the LEDs is the first 5 layers: keep them 100% solid (default). This is the light diffuser |
| v6_cover_black.stl | 1 | Black PLA (matte) | Front face DOWN, flat, turned 45 deg around Z (diagonal, not tilted) | Pegs on the back point up |
| v6_bezel_ring_black.stl | 2 | Black PLA (matte) | Front face DOWN | Pegs and the black skirts (light shields, up to 5.3 mm) point up. Has the 12 o'clock slot for the red plug |
| v6_pip_red.stl | 4 (2 + spares) | Red PLA | Top face DOWN | 2.2 x 5.8 x 2.0, press into the bezel slot flush, drop of CA glue |
| v6_knob_red.stl | 2 | Red PLA | Top face DOWN | Same as before (fits the KY-040 D-shaft) |

## Slicer settings (Bambu Studio, A1 Mini, 0.4 nozzle)

Start from `0.20mm Standard @BBL A1M` + the Bambu PLA profile, then change only these.

| Setting | Value | Why |
|---|---|---|
| Wall generator | **Arachne** | The OLED skirt side walls are 0.5 mm and the bezel bridge is 0.8 mm. Classic can drop them |
| Wall loops | 3 | Strong pegs and screw bosses |
| Sparse infill | 15% gyroid | Plenty for flat plates |
| Top / bottom shell layers | 5 / 5 | Keeps the 1 mm LED skin fully solid |
| Bottom surface pattern | Monotonic line | Clean, even front face (it prints face down) |
| Elephant foot compensation | 0.15 | Windows and bezel bores stay true at the face |
| Seam | Aligned | |
| Supports / brim | Off / off | Nothing overhangs; parts are flat |
| Outer wall speed | 150 mm/s | Sharper window edges and fits; leave the other speeds at default |
| Speed mode (printer) | Standard | Sport is fine for the cover only |

| Part | Changes from the table |
|---|---|
| Carrier (white) | None. Use the same plate type as the cover so both faces match |
| Cover (black) | Sport speed is OK |
| Bezels (black, 2) | None. Print both together |
| Pips (red, 4) | Print with the knobs. Leave "slow down for layer cooling" on |
| Knobs (red, 2) | 0.16 mm layers, 4 walls, 30% infill |

Plate: textured PEI gives a matte, textured front face; smooth PEI or a cool plate gives a flat, satin face. Both work.
Filament: PLA at 220 C nozzle, 65 C bed for textured PEI (55 C for smooth). Dry the black first if it strings.

## Hardware

| Item | Qty | Use |
|---|---|---|
| M2 heat-set insert (3.2 mm hole, about 3.5 mm OD, 4 mm long) | 2 | Into the carrier's back, in the waist, for the cover screws |
| M2 x 16 machine screw, wafer / flat head (ISO 7380 or DIN 7991) | 2 | From the front (waist): through the cover and the carrier's insert into the body's waist lugs |
| Thin double-sided tape (0.2 to 0.5 mm) or 4 to 6 dots of CA glue | | Cover's outer band to the carrier, so the pod edges sit flat (the front unit never has to come apart) |
| M3 x 35 thread-forming screw for plastic (pan or flat head) | 4 | Body to the front unit, from the BACK (when the body is printed) |

## How it holds together

| Joint | Held by |
|---|---|
| Cover to carrier to body | 2 x M2 x 16 in the waist (into the body lugs) + 16 pegs (8 per dial) locating the outer band + tape/CA on the band |
| Bezels to carrier | 4 pegs each, press fit (drop of CA if loose) |
| Front unit to body | 4 x M3 x 35 from the back through the body posts and clamp plates into the carrier's screw holes (2 per dial, hidden behind the bezels) |
| Displays, rings, OLEDs | Clamp plates squeezed by the body (no glue) |

## Assembly

1. Heat-set the 2 inserts into the carrier's back (waist).
2. Carrier face UP on the table. Lay the cover on it (pegs into the holes, tape/CA on the outer band), 2 x M2 x 8 from the front into the inserts.
3. Press the bezel rings in (12 o'clock slot up): their black skirts slide down into the display and OLED windows. Push the red plugs into the slots. This front unit is now permanent.
4. Turn the front unit face DOWN on a soft cloth.
5. Round displays: glass into the recess, pin tab toward 12 o'clock.
6. LED rings: LEDs down into the pits, back flush with the floor, wire joint at 12 o'clock.
7. 0.96" OLEDs UPSIDE DOWN (pins toward 6 o'clock): glass into the slot, clear strip tucks under the round display, board rests on the ring back.
8. Body later: a clamp plate on each dial, body over it, 4 x M3 x 35 from the back. To service anything, undo those 4 screws: the front never comes apart.

For a test before the body exists, a few dabs of hot glue (board edges only, never on glass) hold the displays, rings and OLEDs.

Firmware notes: both round displays and both OLEDs are rotated 180 degrees; the OLED yellow band is the bottom 16 rows; one OLED needs its I2C address resistor moved (0x3C / 0x3D).

The body is NOT final yet. Its internal plan (internal_layout.png) reserves: 40 mm 3 W rear-firing speaker in the waist, 2 x 103450 LiPo (about 4000 mAh) behind the right dial, charger/boost and I2S amp, S3 with USB-C out the left side for a 90 degree cable. None of that changes these front parts.
