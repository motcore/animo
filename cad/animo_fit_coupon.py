# Animo — fit coupon: round and square, a female strip and loose male pins each
# FreeCAD Python Macro
#
# Settles the PLA fits in one print instead of trial and error (user,
# 2026-10-18). Nominal 6.00 for every shape:
#   - FEMALE strip: holes 6.00, 6.05 ... 6.40, labelled 00, 05 ... 40
#     (hundredths ABOVE nominal), through a 5 mm plate.
#   - MALE set: pins 6.00, 5.95 ... 5.60, labelled 00, 05 ... 40
#     (hundredths BELOW nominal), 6 mm tall, each on its own loose tile so
#     any pin goes into any hole (on one strip they only fit all at once).
# Try the 00 pin in every hole and every pin in the 00 hole: the first gives
# how holes print, the second how outsides print, so the macros can set
# both. For each, note: does not go in / press (by hand) / slides / turns
# with play. Square = the flat faces of the hex and the D.
#
# Printed as they lie: holes and pins vertical, like the parts they stand for.
# Run headless:
#   "C:/Program Files/FreeCAD 1.0/bin/freecadcmd.exe" animo_fit_coupon.py

import FreeCAD as App
import Part
import os

NOMINAL = 6.0     # mm
STEP    = 0.05    # mm
N       = 9       # 0.00 .. 0.40
PITCH   = 10.0    # mm between features
PLATE_T = 5.0     # mm — female plate
BASE_T  = 2.0     # mm — male tiles' base
PIN_H   = 6.0     # mm
WIDTH   = 18.0    # mm — strip width: feature row + label row
ROW_Y   = 6.0     # mm — feature centres from the strip's edge
LABEL_Y = 14.0    # mm — label centres
LABEL_H = 3.5     # mm — text height
TILE_W  = 8.0     # mm — male tiles, PITCH apart: 2 mm between them
ENGRAVE = 0.4     # mm
FONT    = "C:/Windows/Fonts/arialbd.ttf"

v = App.Vector


def feature(shape, size, x, z0, z1):
    """A round or square prism, vertical, centred at (x, ROW_Y)."""
    h = z1 - z0
    if shape == "round":
        return Part.makeCylinder(size / 2.0, h, v(x, ROW_Y, z0))
    return Part.makeBox(size, size, h, v(x - size / 2.0, ROW_Y - size / 2.0, z0))


def label(text, x, z_top):
    """Engraving cutter for `text`, centred at (x, LABEL_Y), from the top."""
    wires = Part.makeWireString(text, FONT, LABEL_H)
    faces = []
    for char in wires:
        for f in Part.makeFace(char, "Part::FaceMakerBullseye").Faces:
            faces.append(f)
    comp = Part.makeCompound(faces)
    bb = comp.BoundBox
    comp.translate(v(x - (bb.XMin + bb.XMax) / 2.0,
                     LABEL_Y - (bb.YMin + bb.YMax) / 2.0, z_top - ENGRAVE))
    return comp.extrude(v(0, 0, ENGRAVE + 1.0))


def length():
    return N * PITCH


def female(shape):
    plate = Part.makeBox(length(), WIDTH, PLATE_T)
    for i in range(N):
        x = PITCH / 2.0 + i * PITCH
        size = NOMINAL + i * STEP
        plate = plate.cut(feature(shape, size, x, -1, PLATE_T + 1))
        plate = plate.cut(label(f"{i * 5:02d}", x, PLATE_T))
    return plate.removeSplitter()


def male(shape):
    tiles = []
    for i in range(N):
        x = PITCH / 2.0 + i * PITCH
        size = NOMINAL - i * STEP
        tile = Part.makeBox(TILE_W, WIDTH, BASE_T, v(x - TILE_W / 2.0, 0, 0))
        tile = tile.cut(label(f"{i * 5:02d}", x, BASE_T))
        pin = feature(shape, size, x, BASE_T - 0.01, BASE_T + PIN_H)
        tiles.append(tile.fuse(pin).removeSplitter())
    return Part.makeCompound(tiles)


PARTS = [(f"{kind}_{shape}", (female if kind == "female" else male)(shape))
         for shape in ("round", "square") for kind in ("female", "male")]

_here = os.path.dirname(os.path.abspath(__file__)) if "__file__" in dir() \
    else App.getUserMacroDir(True)
_dir = os.path.join(_here, "stl", "coupon")
os.makedirs(_dir, exist_ok=True)
for name, shape in PARTS:
    assert shape.isValid(), name
    if name.startswith("male"):
        assert len(shape.Solids) == N, name     # loose tiles, not one strip
    shape.exportStl(os.path.join(_dir, f"animo_fit_{name}.stl"))
    bb = shape.BoundBox
    print(f"  {name}: {bb.XLength:.0f} x {bb.YLength:.0f} x {bb.ZLength:.0f} mm,"
          f" {shape.Volume / 1000:.1f} cm3")

if getattr(App, "GuiUp", 0):
    doc = App.newDocument("AnimoFitCoupon")
    for i, (name, shape) in enumerate(PARTS):
        s = shape.copy()
        s.translate(v(0, i * (WIDTH + 5), 0))
        doc.addObject("Part::Feature", name).Shape = s
    doc.recompute()
