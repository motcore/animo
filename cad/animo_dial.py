# Animo — servo dial: a protractor for the cube's outputs
# FreeCAD Python Macro
#
# To read where each output really is (firmware/calibrate.py, the guide in
# notas/montaje-calibrar.md), so every wall turns the same for the same
# command (firmware/cal.py):
#   - DIAL: a disc held on the wall's face by two LEGO Technic pins in the
#     two lower pin holes (+-8, -16 from the output; animo_v2_cube.py). Only
#     those two: they are symmetric, so the dial fits every wall one way and
#     its 0 is always straight down. Grooves every 5 deg, longer every 15 and
#     45; a dimple marks the + side (clockwise seen from outside).
#   - POINTER: the legs' printed cross spigot (animo_duck.py) with a needle
#     along one arm, so the needle points down when the cross is square.
#     One dial and one pointer go from wall to wall.
#
# Coordinates: the wall's face is z = 0 and the output's axis is z, +z out of
# the cube; the dial's +y is up, so seen from outside its -x is the + side.
#   "C:/Program Files/FreeCAD 1.0/bin/freecadcmd.exe" animo_dial.py

import math
import os
import Part
import FreeCAD as App
from FreeCAD import Vector

MATERIAL = os.environ.get("ANIMO_MATERIAL", "PETG")

# ── The wall (from animo_v2_cube.py / animo_duck.py) ────────────────────────
hub_recess = 0.2     # mm — the hub's face, below the wall's face
cross_hole = 6.54    # mm — the hub's cross hole, deep from the wall's face
tech_pin_d = 4.8     # mm — Technic pin hole
pin_grow   = 0.15    # mm — PETG (the Technic coupon, 2026-10-20)
PINS_XY    = [(-8.0, -16.0), (8.0, -16.0)]   # the two lower pin holes

# ── The printed cross spigot (as animo_duck.py) ─────────────────────────────
spigot     = 4.80    # mm — cross, tip to tip: the coupon (animo_trex.py) in the 4.90 hubs, 2026-11-01
arm_less   = 3.0     # mm — arm width = spigot - 3.0
spigot_len = 6.0     # mm — 0.34 short of the hole's floor
boss_r     = 4.5     # mm

# ── Dial ────────────────────────────────────────────────────────────────────
dial_r     = 26.0    # mm — inside the 56 mm face
dial_t     = 3.0     # mm
dial_hole  = boss_r + 1.0   # mm — round the pointer's boss
groove_w   = 0.6     # mm — the tick grooves, on the top face
groove_d   = 0.6     # mm
TICKS      = [(5, 22.5), (15, 21.0), (45, 19.0)]   # every n deg, from radius
tick_out   = 25.5    # mm — all ticks end here
zero_from  = 17.0    # mm — the 0 tick, longest
dimple     = (2.0, -14.0, 0.0)   # mm — diameter, x, y: the + side (90 deg)

# ── Pointer ─────────────────────────────────────────────────────────────────
ptr_gap    = 0.6     # mm — the needle above the dial
ptr_t      = 1.6     # mm — the needle's thickness
ptr_w      = 5.0     # mm — its width at the boss
ptr_r      = 25.0    # mm — its tip, over the ticks
grip       = (6.0, 10.0)   # mm — a disc round the boss and a tail to hold it

Z_AXIS = Vector(0, 0, 1)
X_AXIS = Vector(1, 0, 0)


def box(x0, x1, y0, y1, z0, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, Vector(x0, y0, z0))


def cyl(r, z0, z1, x=0.0, y=0.0):
    return Part.makeCylinder(r, z1 - z0, Vector(x, y, z0))


def cross_z(span, z0, z1):
    arm = span - arm_less
    return box(-span / 2, span / 2, -arm / 2, arm / 2, z0, z1).fuse(
        box(-arm / 2, arm / 2, -span / 2, span / 2, z0, z1)).removeSplitter()


def prism(pts, z0, z1):
    w = Part.makePolygon([Vector(x, y, z0) for x, y in pts + pts[:1]])
    return Part.Face(w).extrude(Vector(0, 0, z1 - z0))


def make_dial():
    d = cyl(dial_r, 0, dial_t).cut(cyl(dial_hole, -1, dial_t + 1))
    for x, y in PINS_XY:
        d = d.cut(cyl((tech_pin_d + pin_grow) / 2, -1, dial_t + 1, x, y))
    top = dial_t - groove_d
    for deg in range(0, 360, 5):
        r0 = zero_from if deg == 0 else min(r for n, r in TICKS if deg % n == 0)
        g = box(-groove_w / 2, groove_w / 2, -tick_out, -r0, top, dial_t + 1)
        g.rotate(Vector(0, 0, 0), Z_AXIS, -deg)   # clockwise seen from outside
        d = d.cut(g)
    dd, dx, dy = dimple
    return d.cut(cyl(dd / 2, top, dial_t + 1, dx, dy)).removeSplitter()


def make_pointer():
    z0 = dial_t + ptr_gap
    p = cross_z(spigot, -hub_recess - spigot_len, -hub_recess)
    p = p.fuse(cyl(boss_r, -hub_recess, z0))
    needle = prism([(-ptr_w / 2, 0), (0, -ptr_r), (ptr_w / 2, 0)], z0, z0 + ptr_t)
    tail = box(-ptr_w / 2, ptr_w / 2, 0, grip[1], z0, z0 + ptr_t)
    return p.fuse([needle, tail, cyl(grip[0], z0, z0 + ptr_t)]).removeSplitter()


DIAL = make_dial()
POINTER = make_pointer()

# ═══════════════════════════════════════════════════════════════════
# CHECKS
# ═══════════════════════════════════════════════════════════════════
fails = 0


def check(name, val, need):
    global fails
    ok = val >= need
    fails += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}: {val:.2f} (>= {need})")


print("\nAnimo dial")
check("pointer clear of the dial", DIAL.distToShape(POINTER)[0], 0.5)
check("spigot short of the hole's floor", cross_hole - hub_recess - spigot_len, 0.2)
hole_r = (tech_pin_d + pin_grow) / 2
for x, y in PINS_XY:     # the 45 deg grooves pass nearest the pin holes
    a = math.radians(45) - math.atan2(abs(x), abs(y))
    check("pin hole clear of the 45 deg groove", math.hypot(x, y) * math.sin(a) - hole_r - groove_w / 2, 1.0)
check("needle tip on the ticks", tick_out - ptr_r, 0.0)
print(f"  dial {DIAL.Volume / 1000:.1f} cm3, pointer {POINTER.Volume / 1000:.2f} cm3")


def on_bed(shape, flip=False):
    s = shape.copy()
    if flip:
        s.rotate(Vector(0, 0, 0), X_AXIS, 180)
    bb = s.BoundBox
    s.translate(Vector(-bb.Center.x, -bb.Center.y, -bb.ZMin))
    return s


DOC_NAME = "Animo_Dial"
if App.ActiveDocument and App.ActiveDocument.Name == DOC_NAME:
    App.closeDocument(DOC_NAME)
doc = App.newDocument(DOC_NAME)
for name, sh in (("Dial", DIAL), ("Pointer", POINTER)):
    doc.addObject("Part::Feature", name).Shape = sh
doc.recompute()

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl", "animo_dial", MATERIAL)
os.makedirs(out, exist_ok=True)
EXPORT = [("animo_dial_Dial", on_bed(DIAL)),                  # grooves up
          ("animo_dial_Pointer", on_bed(POINTER, True))]       # needle down, spigot up
for name, sh in EXPORT:
    sh.exportStl(os.path.join(out, name + ".stl"))
print(f"\n  STL -> {out}  ({fails} FAIL)")
