# Animo — the T-rex walker, v1: legs, feet and tail (the head comes later)
# FreeCAD Python Macro
#
# The first model on the v2 cube (animo_v2_cube.py, 56 mm, brain on top).
# No LEGO parts (user, 2026-10-28): each part carries its own printed cross
# spigot that goes straight into the output hub's cross hole.
#
# The idea (user, 2026-10-17): the passive dynamics live in the MODEL. The
# cube rocks side to side on arched feet, the tail pumps that rocking, and
# the legs step while one foot is unloaded. Open loop, no sensors.
#
#   - LEGS on the side walls (+-X): each output turns about X, so the leg
#     swings fore and aft (the walk is along +Y).
#   - FEET: one solid of revolution about the hip axis. Its section (in the
#     frontal plane) is an arc of radius R_FRONT centred on the body's middle,
#     so the two soles are two pieces of one rocker: upright, the cube stands
#     on the feet's inner edges; tilted, it rolls onto one foot and lifts the
#     other. The section is revolved about an axis SAG_UP above the hip, so
#     fore and aft the sole is an arc too, centred there: the stance foot
#     rolls heel to toe as the leg turns. Not about the hip itself: with the
#     brain on top the centre of mass is ABOVE the hip, and a sole centred on
#     the hip would let the cube topple forward or back.
#     Stable both ways while the centre of mass is below both arcs' centres.
#     The feet reach in under the cube so the rocker has no wide gap.
#   - TAIL on the back wall (-Y): that output turns about Y, so a tail
#     pointing straight back would only spin on itself. It goes back and
#     DOWN, with an M5 screw and nuts at the tip as the weight: the output's
#     turn wags the tip from side to side, and swung fast, its reaction rolls
#     the body the other way (gravity helps it back: it hangs). A crossbar
#     (2026-10-28, first draw) pushes about twice as hard but doesn't look
#     like a tail (user). Nuts on the screw tune it without reprinting.
#     Its weight sits behind the cube: the firmware's LEAN turns both legs
#     together to keep the body level.
#
# Coordinates: as the cube's. Origin = cube centre, +Z up, +Y = forward.

import os
import Part
import FreeCAD as App
from FreeCAD import Vector

MATERIAL = os.environ.get("ANIMO_MATERIAL", "PETG")

# ── The cube (from animo_v2_cube.py) ────────────────────────────────────────
half       = 28.0    # mm — cube centre to a wall's outer face
brain_top  = 56.0    # mm — the brain's lid, above the cube's centre
hub_recess = 0.2     # mm — the hub's face, below the wall's face
hub_d      = 10.0    # mm — the hub turns with the part: rest on it only
cross_hole = 6.54    # mm — the hub's cross hole, deep from the wall's face

# ── The printed cross spigot ────────────────────────────────────────────────
# The hub's hole is 4.87 / 1.87 (2026-10-28: a real LEGO axle, 4.8, goes in
# hard). Outsides print ~0.17 small in PETG, so a drawn 4.86 should come out
# ~4.7: a firm push. SPIGOT_TRY are the coupon's.
spigot     = 4.86    # mm — cross, tip to tip
arm_less   = 3.0     # mm — arm width = spigot - 3.0 (as 4.8 / 1.8)
spigot_len = 6.0     # mm — 0.34 short of the hole's floor (from the hub's face)
SPIGOT_TRY = [4.80, 4.86, 4.92]
boss_r     = 4.5     # mm — the spacer that rests on the hub (r 5): the part
stand_off  = 0.6     #      stands 0.6 off the wall, so it never rubs it

# ── Legs and feet ───────────────────────────────────────────────────────────
leg_t      = 6.0     # mm — the leg plate
leg_w      = 12.0    # mm — its width (fore-aft)
SWING      = 20.0    # deg — the leg's travel each way (the servo gives ~45)
R_FRONT    = 75.0    # mm — the rocker's radius, frontal plane
x_in       = 0.5     # mm — the feet's inner edges: a 1 mm gap. With 6 the cube
                     #      sat on two edges and the tail could not rock it
                     #      (MuJoCo, 2026-10-28): the rocker must be whole
SAG_UP     = 20.0    # mm — the fore-aft arc's centre above the hip; the
                     #      centre of mass is ~5 above the hip (brain)
HIP_H      = 52.0    # mm — hip to the sole at the inner edge: the cube's
                     #      bottom stands HIP_H - 28 off the ground. 48 left
                     #      no room under the cube for the longer fore-aft arc
r_top_in   = 42.5    # mm — the foot's top under the cube (from the hip): it
                     #      must clear the cube's bottom over the whole swing;
                     #      the ends, lifted by the leg's turn, set it
r_top_out  = 30.0    # mm — and outside the cube, where it meets the leg plate
foot_arc   = SWING   # deg — the sole each way (about its own centre): the
                     #      contact is where the sole's normal is vertical,
                     #      SWING behind or ahead of the foot's middle
clear      = 0.4     # mm — the least gap the checks accept

# ── Tail ────────────────────────────────────────────────────────────────────
TAIL_SWING = 35.0    # deg — each way
TAIL_TIP   = (-52.0, -30.0)   # mm — (y, z) of the tip: 24 behind the wall,
                     #      30 below the axis (22 off the ground hanging)
TAIL_BEND  = (-52.0, 0.0)     # mm — the curve's control point: straight back,
                     #      then down
tail_r     = (4.5, 3.0)       # mm — half its height, at the wall and at the tip
tail_t     = spigot  # mm — thickness (along X): it prints on its side, and
                     #      the spigot's tips must reach the bed
tip_r      = 6.0     # mm — the tip's disc, round the screw
m5_clear   = 5.4     # mm — M5 across the tail (along X): screw + nuts
WEIGHT     = (4.6, 9.0)       # mm — the ghost weight: nut's radius, and how far
                              #      it stands out each side (M5x16, 2 nuts)

# ═══════════════════════════════════════════════════════════════════
# HELPERS
# ═══════════════════════════════════════════════════════════════════

def box(x0, x1, y0, y1, z0, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, Vector(x0, y0, z0))


def cyl_x(r, x0, x1, y=0.0, z=0.0):
    a, b = min(x0, x1), max(x0, x1)
    return Part.makeCylinder(r, b - a, Vector(a, y, z), Vector(1, 0, 0))


def cyl_y(r, y0, y1, x=0.0, z=0.0):
    a, b = min(y0, y1), max(y0, y1)
    return Part.makeCylinder(r, b - a, Vector(x, a, z), Vector(0, 1, 0))


def cross_x(span, x0, x1):
    arm = span - arm_less
    a, b = min(x0, x1), max(x0, x1)
    return box(a, b, -span / 2, span / 2, -arm / 2, arm / 2).fuse(
        box(a, b, -arm / 2, arm / 2, -span / 2, span / 2)).removeSplitter()


def cross_y(span, y0, y1):
    arm = span - arm_less
    a, b = min(y0, y1), max(y0, y1)
    return box(-span / 2, span / 2, a, b, -arm / 2, arm / 2).fuse(
        box(-arm / 2, arm / 2, a, b, -span / 2, span / 2)).removeSplitter()


def rot(shape, axis, deg):
    s = shape.copy()
    s.rotate(Vector(0, 0, 0), axis, deg)
    return s


X_AXIS, Y_AXIS, Z_AXIS = Vector(1, 0, 0), Vector(0, 1, 0), Vector(0, 0, 1)

# The rocker: the frontal arc's centre, above the hip, so that the inner
# edge's sole is HIP_H below the hip.
z_c = (R_FRONT ** 2 - x_in ** 2) ** 0.5 - HIP_H


def r_sole(x):
    """Hip to sole, at |x| from the middle."""
    return (R_FRONT ** 2 - x ** 2) ** 0.5 - z_c


x_leg0 = half + stand_off              # the leg plate's inner face
x_leg1 = x_leg0 + leg_t                # its outer face
x_cube = x_leg0                        # past here the foot is out of the cube

# ═══════════════════════════════════════════════════════════════════
# PARTS
# ═══════════════════════════════════════════════════════════════════

def spigot_x(s, span=None):
    """Spacer on the hub + cross into it, on the +-X wall (s = +1 / -1)."""
    face = half - hub_recess
    boss = cyl_x(boss_r, s * face, s * x_leg0)
    return boss.fuse(cross_x(span or spigot, s * (face - spigot_len), s * face))


def leg_body(s):
    """Leg plate and foot, without the spigot (s = +1 right, -1 left)."""
    plate = cyl_x(leg_w / 2, s * x_leg0, s * x_leg1).fuse(
        box(min(s * x_leg0, s * x_leg1), max(s * x_leg0, s * x_leg1),
            -leg_w / 2, leg_w / 2, -(r_top_out + 2.0), 0.0))
    # The foot's section in the plane y = 0, below the hip: the sole's arc
    # from the inner edge out, back up the outer face, and the top in two
    # steps (low under the cube, high outside it).
    xo = x_leg1
    p_in = Vector(s * x_in, 0, -r_sole(x_in))
    p_mid = Vector(s * (x_in + xo) / 2, 0, -r_sole((x_in + xo) / 2))
    p_out = Vector(s * xo, 0, -r_sole(xo))
    edges = [Part.Arc(p_in, p_mid, p_out).toShape()]
    pts = [p_out, Vector(s * xo, 0, -r_top_out), Vector(s * x_cube, 0, -r_top_out),
           Vector(s * x_cube, 0, -r_top_in), Vector(s * x_in, 0, -r_top_in), p_in]
    edges += [Part.makeLine(a, b) for a, b in zip(pts, pts[1:])]
    sec = Part.Face(Part.Wire(Part.__sortEdges__(edges)))
    c = Vector(0, 0, SAG_UP)
    sec = sec.copy()
    sec.rotate(c, X_AXIS, -foot_arc)
    foot = sec.revolve(c, X_AXIS, 2 * foot_arc)
    return plate.fuse(Part.Solid(foot) if foot.ShapeType == "Shell" else foot).removeSplitter()


def make_leg(s):
    return leg_body(s).fuse(spigot_x(s)).removeSplitter()


def _tail_outline():
    """The tail's side view (y, z): a quadratic curve from the wall back and
    down to the tip, its height tapering; returns the closed outline."""
    p0 = (-x_leg0, 0.0)
    p1, p2 = TAIL_BEND, TAIL_TIP
    up, lo = [], []
    n = 24
    for i in range(n + 1):
        t = i / n
        y = (1 - t) ** 2 * p0[0] + 2 * t * (1 - t) * p1[0] + t ** 2 * p2[0]
        z = (1 - t) ** 2 * p0[1] + 2 * t * (1 - t) * p1[1] + t ** 2 * p2[1]
        dy = 2 * (1 - t) * (p1[0] - p0[0]) + 2 * t * (p2[0] - p1[0])
        dz = 2 * (1 - t) * (p1[1] - p0[1]) + 2 * t * (p2[1] - p1[1])
        L = (dy * dy + dz * dz) ** 0.5
        ny, nz = -dz / L, dy / L
        r = tail_r[0] + (tail_r[1] - tail_r[0]) * t
        up.append(Vector(0, y + r * ny, z + r * nz))
        lo.append(Vector(0, y - r * ny, z - r * nz))
    return up, lo


def tail_body(weight=False):
    up, lo = _tail_outline()
    e1 = Part.BSplineCurve()
    e1.interpolate(up)
    e2 = Part.BSplineCurve()
    e2.interpolate(lo[::-1])
    edges = [e1.toShape(), Part.makeLine(up[-1], lo[-1]), e2.toShape(),
             Part.makeLine(lo[0], up[0])]
    face = Part.Face(Part.Wire(edges))
    face.translate(Vector(-tail_t / 2, 0, 0))
    t = face.extrude(Vector(tail_t, 0, 0))
    ty, tz = TAIL_TIP
    t = t.fuse(cyl_x(tip_r, -tail_t / 2, tail_t / 2, ty, tz)).removeSplitter()
    t = t.cut(cyl_x(m5_clear / 2, -tail_t, tail_t, ty, tz))
    if weight:      # screw head, nuts: the clearance checks see them
        t = t.fuse(cyl_x(WEIGHT[0], -tail_t / 2 - WEIGHT[1], tail_t / 2 + WEIGHT[1], ty, tz))
    return t


def make_tail():
    face = half - hub_recess
    boss = cyl_y(boss_r, -face, -x_leg0).common(
        box(-tail_t / 2, tail_t / 2, -face - 1, -x_leg0 + 1, -boss_r, boss_r))
    return tail_body().fuse(boss).fuse(
        cross_y(spigot, -(face - spigot_len), -face)).removeSplitter()


def make_coupon():
    """Three spigots on numbered tabs, spigot up: try each in a hub."""
    parts = []
    for i, sp in enumerate(SPIGOT_TRY):
        x = i * 16.0
        tab = Part.makeCylinder(6.0, 2.0, Vector(x, 0, 0))
        spg = cross_x(sp, 0, spigot_len)
        spg = rot(spg, Y_AXIS, -90)         # along +Z
        spg.translate(Vector(x, 0, 2.0))
        t = tab.fuse(spg)
        for k in range(i + 1):              # notches: 1, 2, 3
            ang = 90 + 25 * k
            t = t.cut(rot(box(5.0, 7.0, -0.6, 0.6, -1, 3), Z_AXIS, ang).translated(Vector(x, 0, 0)))
        parts.append(t)
    return parts[0].fuse(parts[1:]).removeSplitter()


# ═══════════════════════════════════════════════════════════════════
# CHECKS
# ═══════════════════════════════════════════════════════════════════

CUBE = box(-half, half, -half, half, -half, brain_top)   # the cube + brain,
                                                         # as a plain block


def gap(a, b):
    if a.common(b).Volume > 1e-3:
        return -1.0
    return a.distToShape(b)[0]


checks = []
LEG = {s: leg_body(s) for s in (1, -1)}
TAIL = tail_body(weight=True)

worst = min(gap(rot(LEG[1], X_AXIS, a), CUBE) for a in range(-int(SWING), int(SWING) + 1, 5))
checks.append(("leg and foot clear the cube over the swing", worst, clear))
worst = min(gap(rot(TAIL, Y_AXIS, a), CUBE)
            for a in range(-int(TAIL_SWING), int(TAIL_SWING) + 1, 7))
checks.append(("tail clears the cube over its swing", worst, clear))
worst = min(gap(rot(TAIL, Y_AXIS, t), rot(LEG[s], X_AXIS, a))
            for t in (-TAIL_SWING, 0, TAIL_SWING) for a in (-SWING, 0, SWING)
            for s in (1, -1))
checks.append(("tail clears the legs (both at the ends)", worst, clear))
checks.append(("spigot short of the hole's floor", cross_hole - hub_recess - spigot_len, 0.2))

print("\nT-rex v1 checks:")
fails = 0
for name, val, need in checks:
    ok = val >= need
    fails += not ok
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}: {val:6.2f}  (>= {need})")

hip = r_sole(x_in)
print(f"\n  Hip {hip:.1f} above the ground; the cube's bottom {hip - half:.1f} off it.")
print(f"  Rocker: R {R_FRONT:.0f}, centre {z_c + hip:.1f} above the ground"
      f" (the centre of mass must stay below it).")
print(f"  Sole: {r_sole(x_in):.1f} from the hip at the inner edge, {r_sole(x_leg1):.1f} at the outer;"
      f" fore-aft arc R {hip + SAG_UP:.0f}, centre {hip + SAG_UP:.0f} above the ground.")
for n, sh in (("leg", make_leg(1)), ("tail", make_tail())):
    print(f"  {n}: {sh.Volume / 1000:.1f} cm3, ~{sh.Volume / 1000 * 1.27:.1f} g solid PETG")

# ═══════════════════════════════════════════════════════════════════
# DOCUMENT + EXPORT (print-oriented)
# ═══════════════════════════════════════════════════════════════════

def on_bed(shape, axis=None, deg=0.0):
    s = rot(shape, axis, deg) if axis else shape.copy()
    bb = s.BoundBox
    s.translate(Vector(-bb.Center.x, -bb.Center.y, -bb.ZMin))
    return s


DOC_NAME = "Animo_TRex"
if App.ActiveDocument and App.ActiveDocument.Name == DOC_NAME:
    App.closeDocument(DOC_NAME)
doc = App.newDocument(DOC_NAME)
for name, sh in (("LegRight", make_leg(1)), ("LegLeft", make_leg(-1)),
                 ("Tail", make_tail()), ("CubeGhost", CUBE)):
    o = doc.addObject("Part::Feature", name)
    o.Shape = sh
doc.recompute()

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl", "animo_trex", MATERIAL)
os.makedirs(out, exist_ok=True)
EXPORT = [("animo_trex_LegRight", on_bed(make_leg(1), Y_AXIS, 90)),     # outer face down
          ("animo_trex_LegLeft", on_bed(make_leg(-1), Y_AXIS, -90)),
          ("animo_trex_Tail", on_bed(make_tail(), Y_AXIS, 90)),         # on its side
          ("animo_trex_SpigotCoupon", on_bed(make_coupon()))]
for name, sh in EXPORT:
    sh.exportStl(os.path.join(out, name + ".stl"))
print(f"\n  STL -> {out}  ({fails} FAIL)")

# Assembled copies, for a picture (ANIMO_VIEW=<folder>)
view = os.environ.get("ANIMO_VIEW")
if view:
    for name, sh in (("LegRight", make_leg(1)), ("LegLeft", make_leg(-1)),
                     ("Tail", make_tail()), ("Cube", CUBE)):
        sh.exportStl(os.path.join(view, name + ".stl"))
