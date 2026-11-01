# Animo — the duckling walker, v1: two legs with doubly curved feet
# FreeCAD Python Macro
#
# The first model on the v2 cube (animo_v2_cube.py, 56 mm, brain on top),
# as the simulator found it (sim/trex_sim.py, 2026-10-30/31; the study in
# docs/walker-sim/). No tail, no head: the two hip servos swing the legs
# +-15 deg at 1.2 Hz and the body rocks side to side on the soles. Open
# loop, no sensors.
#
#   - LEGS on the side walls (+-X), each with the printed cross spigot of
#     animo_trex.py: the output turns about X, the leg swings fore and aft
#     (the walk is along +Y). Hip 92 above the lowest point of the sole.
#   - FEET under the cube: each sole is the sim's cap foot, a translation
#     surface (a circle side to side, R_SIDE, plus a circle fore and aft,
#     R_FORE): z = -HIP + (R_SIDE - sqrt(R_SIDE^2 - (x - GAP/2)^2))
#                       + (R_FORE - sqrt(R_FORE^2 - y^2)).
#     Its outline (seen from below) is the sim's rounded one: full width
#     only from WIDE[0] to WIDE[1], then quarter ellipses in to HEEL_X at the
#     heel and TOE_X at the toe. The inner edge (x_in from the middle) and
#     the heel's edge stay sharp: in the sim both are part of the gait.
#   - Each leg is TWO prints: the plate (on its side, spigot up) and the
#     foot (flat top on the bed, the sole up, so the sole prints as a top
#     surface with its edges sharp). Two M3x12 (the kit's) hold the plate's
#     flange on the foot, self-tapping into the foot's pilot holes: a foot
#     can be swapped without reprinting the plate.
#
# Coordinates: as the cube's. Origin = cube centre = the hips, +Z up,
# +Y = forward. Run headless to read the checks:
#   "C:/Program Files/FreeCAD 1.0/bin/freecadcmd.exe" animo_duck.py

import math
import os
import Part
import FreeCAD as App
from FreeCAD import Vector

MATERIAL = os.environ.get("ANIMO_MATERIAL", "PETG")

# ── The cube (from animo_v2_cube.py) ────────────────────────────────────────
half       = 28.0    # mm — cube centre to a wall's outer face
brain_top  = 56.0    # mm — the brain's lid, above the cube's centre
hub_recess = 0.2     # mm — the hub's face, below the wall's face
cross_hole = 6.54    # mm — the hub's cross hole, deep from the wall's face

# ── The printed cross spigot (as animo_trex.py) ─────────────────────────────
spigot     = 4.80    # mm — cross, tip to tip: the coupon (animo_trex.py) in the 4.90 hubs, 2026-11-01
arm_less   = 3.0     # mm — arm width = spigot - 3.0
spigot_len = 6.0     # mm — 0.34 short of the hole's floor
boss_r     = 4.5     # mm — the spacer that rests on the hub
stand_off  = 0.6     #      the plate stands 0.6 off the wall

# ── The sole: the sim's winner (sim/trex_sim.py P names in brackets) ────────
HIP        = 92.0    # mm — hip above the sole's lowest point [hip]
R_SIDE     = 130.0   # mm — the sole's radius side to side [ball_r]
R_FORE     = 186.0   # mm — and fore and aft [ball_rs]
GAP        = 7.0     # mm — between the two side circles' centres [ball_gap]
x_in       = 0.5     # mm — the inner edge, out from the middle [cap_in]
x_out      = 60.0    # mm — the outer edge, at its widest [cap_out]
HEEL       = 40.0    # mm — the heel's edge behind the hip [cap_heel]
TOE        = 80.0    # mm — the toe's edge ahead of it [cap_toe]
WIDE       = (-10.0, 25.0)   # mm — y where the outer edge is at x_out
                             #      [cap_wide_from, cap_wide_to]
HEEL_X     = 50.0    # mm — the outer edge at the heel's end [cap_heel_x]
TOE_X      = 15.0    #      and at the toe's [cap_toe_x]
CAP_T      = 15.0    # mm — the foot's top at least this far above the
                     #      sole's lowest point, and 3 above its highest
                     #      (the sim's CAP_T, so the same solid)

# ── The leg plate ───────────────────────────────────────────────────────────
leg_t      = 6.0     # mm — the plate (as animo_trex.py)
leg_w      = 12.0    # mm — its width (fore-aft)
SWING      = 15.0    # deg — the gait's swing each way; checked to CHECK_SWING
CHECK_SWING = 25.0   #      (a servo overshoots, a trim is off)
flange_t   = 4.0     # mm — the flange on the foot's top
flange_in  = 18.6    # mm — it reaches in to here from the middle
flange_w   = 20.0    # mm — fore-aft
screw_x    = 23.6    # mm — the two M3 out from the middle, at y = +-screw_y
screw_y    = 6.0
m3_clear   = 3.4     # mm — through the flange
m3_pilot   = 2.6     # mm — into the foot: the M3 cuts its own thread
pilot_deep = 10.0    # mm — M3x12: 4 of flange + 8 in the foot
head_r     = 2.9     # mm — an M3 cheese head's radius, + a little
gusset     = (4.0, 6.0, 8.0)  # mm — the plate-flange rib: wide (y), in (x),
                              #      up (z); between the two screw heads
clear      = 0.4     # mm — the least gap the checks accept

# ═══════════════════════════════════════════════════════════════════
# HELPERS
# ═══════════════════════════════════════════════════════════════════

def box(x0, x1, y0, y1, z0, z1):
    a, b = min(x0, x1), max(x0, x1)
    return Part.makeBox(b - a, y1 - y0, z1 - z0, Vector(a, y0, z0))


def cyl_x(r, x0, x1, y=0.0, z=0.0):
    a, b = min(x0, x1), max(x0, x1)
    return Part.makeCylinder(r, b - a, Vector(a, y, z), Vector(1, 0, 0))


def cyl_z(r, z0, z1, x=0.0, y=0.0):
    return Part.makeCylinder(r, z1 - z0, Vector(x, y, z0))


def cross_x(span, x0, x1):
    arm = span - arm_less
    a, b = min(x0, x1), max(x0, x1)
    return box(a, b, -span / 2, span / 2, -arm / 2, arm / 2).fuse(
        box(a, b, -arm / 2, arm / 2, -span / 2, span / 2)).removeSplitter()


def rot(shape, axis, deg):
    s = shape.copy()
    s.rotate(Vector(0, 0, 0), axis, deg)
    return s


X_AXIS, Y_AXIS, Z_AXIS = Vector(1, 0, 0), Vector(0, 1, 0), Vector(0, 0, 1)


def sole_z(x, y, s):
    """The sole's height at (x, y), foot s (+1 right, -1 left)."""
    dx = x - s * GAP / 2
    return (-HIP + R_SIDE - math.sqrt(R_SIDE ** 2 - dx * dx)
            + R_FORE - math.sqrt(R_FORE ** 2 - y * y))


def outer_x(y):
    """The outer edge's |x| at y: the sim's outline (quarter ellipses)."""
    if y < WIDE[0]:
        u = (y - WIDE[0]) / (-HEEL - WIDE[0])
        return HEEL_X + (x_out - HEEL_X) * math.sqrt(max(1 - u * u, 0))
    if y > WIDE[1]:
        u = (y - WIDE[1]) / (TOE - WIDE[1])
        return TOE_X + (x_out - TOE_X) * math.sqrt(max(1 - u * u, 0))
    return x_out


# The foot's flat top: as the sim's cap, CAP_T above the lowest point or 3
# above the highest point of the sole, whichever is higher.
_hi = max(sole_z(x_in + (outer_x(y) - x_in) * k / 40, y, 1)
          for y in [-HEEL + i * (HEEL + TOE) / 240 for i in range(241)]
          for k in range(41))
z_top = max(-HIP + CAP_T, _hi + 3.0)

x_leg0 = half + stand_off              # the plate's inner face
x_leg1 = x_leg0 + leg_t                # its outer face

# ═══════════════════════════════════════════════════════════════════
# PARTS
# ═══════════════════════════════════════════════════════════════════

def ell_arc(s, cx, cy, ax, ay, p0, p1):
    """A quarter of the ellipse centred (s*cx, cy), semi-axes ax (x) and
    ay (y), from p0 to p1 (each (|x|, y))."""
    c = Vector(s * cx, cy, 0)
    if ay >= ax:
        e = Part.Ellipse(c + Vector(0, ay, 0), c + Vector(ax, 0, 0), c)
    else:
        e = Part.Ellipse(c + Vector(ax, 0, 0), c + Vector(0, ay, 0), c)
    a = Vector(s * p0[0], p0[1], 0)
    b = Vector(s * p1[0], p1[1], 0)
    u0, u1 = e.parameter(a), e.parameter(b)
    if u1 < u0:
        u0, u1 = u1, u0
    if u1 - u0 > math.pi:                       # the short way round
        u0, u1 = u1, u0 + 2 * math.pi
    return e.toShape(u0, u1)


def outline(s):
    """The foot seen from below, as a face at z = 0."""
    v = lambda x, y: Vector(s * x, y, 0)
    edges = [Part.makeLine(v(x_in, -HEEL), v(x_in, TOE)),           # inner edge
             Part.makeLine(v(x_in, TOE), v(TOE_X, TOE)),            # toe
             ell_arc(s, TOE_X, WIDE[1], x_out - TOE_X, TOE - WIDE[1],
                     (TOE_X, TOE), (x_out, WIDE[1])),
             Part.makeLine(v(x_out, WIDE[1]), v(x_out, WIDE[0])),
             ell_arc(s, HEEL_X, WIDE[0], x_out - HEEL_X, HEEL + WIDE[0],
                     (x_out, WIDE[0]), (HEEL_X, -HEEL)),
             Part.makeLine(v(HEEL_X, -HEEL), v(x_in, -HEEL))]       # heel
    return Part.Face(Part.Wire(Part.__sortEdges__(edges)))


def make_foot(s):
    """The foot under the cube: the sole surface, the outline, the flat top,
    and the two pilot holes for the plate's screws."""
    nx, ny = 36, 66
    xs = [s * (x_in - 3 + (x_out + 6 - x_in) * i / (nx - 1)) for i in range(nx)]
    ys = [-HEEL - 3 + (HEEL + TOE + 6) * j / (ny - 1) for j in range(ny)]
    surf = Part.BSplineSurface()
    surf.interpolate([[Vector(x, y, sole_z(x, y, s)) for y in ys] for x in xs])
    slab = surf.toShape().extrude(Vector(0, 0, 60))     # the sole and above
    prism = outline(s).extrude(Vector(0, 0, z_top + HIP + 10))
    prism.translate(Vector(0, 0, -HIP - 5))
    top = box(-100, 100, -100, 100, -HIP - 10, z_top)
    foot = slab.common(prism).common(top)
    for y in (-screw_y, screw_y):
        foot = foot.cut(cyl_z(m3_pilot / 2, z_top - pilot_deep, z_top + 1, s * screw_x, y))
    return foot.removeSplitter().Solids[0]      # the booleans leave a compound


def make_plate(s):
    """Spigot, plate down from the hip, flange and rib on the foot's top."""
    face = half - hub_recess
    p = cyl_x(boss_r, s * face, s * x_leg0).fuse(
        cross_x(spigot, s * (face - spigot_len), s * face))
    p = p.fuse(cyl_x(leg_w / 2, s * x_leg0, s * x_leg1))
    p = p.fuse(box(s * x_leg0, s * x_leg1, -leg_w / 2, leg_w / 2, z_top, 0.0))
    p = p.fuse(box(s * flange_in, s * x_leg1, -flange_w / 2, flange_w / 2,
                   z_top, z_top + flange_t))
    gw, gi, gu = gusset
    z0 = z_top + flange_t
    rib = Part.Face(Part.makePolygon([Vector(s * x_leg0, -gw / 2, z0),
                                      Vector(s * (x_leg0 - gi), -gw / 2, z0),
                                      Vector(s * x_leg0, -gw / 2, z0 + gu),
                                      Vector(s * x_leg0, -gw / 2, z0)]))
    p = p.fuse(rib.extrude(Vector(0, gw, 0)))
    for y in (-screw_y, screw_y):
        p = p.cut(cyl_z(m3_clear / 2, z_top - 1, z0 + 1, s * screw_x, y))
    return p.removeSplitter().Solids[0]


# ═══════════════════════════════════════════════════════════════════
# CHECKS
# ═══════════════════════════════════════════════════════════════════

CUBE = box(-half, half, -half, half, -half, brain_top)   # cube + brain


def gap(a, b):
    if a.common(b).Volume > 1e-3:
        return -1.0
    return a.distToShape(b)[0]


FOOT = {s: make_foot(s) for s in (1, -1)}
PLATE = {s: make_plate(s) for s in (1, -1)}
LEG = {s: PLATE[s].fuse(FOOT[s]) for s in (1, -1)}
HEADS = {s: Part.makeCompound([cyl_z(head_r, z_top + flange_t, z_top + flange_t + 3,
                                     s * screw_x, y) for y in (-screw_y, screw_y)])
         for s in (1, -1)}

checks = []
A = int(CHECK_SWING)
HUB = box(0, x_leg0, -boss_r - 1, boss_r + 1, -boss_r - 1, boss_r + 1)   # spigot and
BODY = LEG[1].cut(HUB).fuse(HEADS[1])                  # boss: in the wall by design
worst = min(gap(rot(BODY, X_AXIS, a), CUBE) for a in range(-A, A + 1, 5))
checks.append((f"leg, foot and screw heads clear the cube (+-{A} deg)", worst, clear))
worst = min(gap(rot(LEG[1], X_AXIS, a), rot(LEG[-1], X_AXIS, -a)) for a in range(-A, A + 1, 5))
checks.append((f"the two feet clear each other, legs in scissors (+-{A} deg)", worst, clear))
worst = min(gap(rot(LEG[1], X_AXIS, a), rot(LEG[-1], X_AXIS, a)) for a in (-A, 0, A))
checks.append(("and side by side (both at once)", worst, clear))
worst = gap(HEADS[1], PLATE[1].cut(box(0, 100, -100, 100, -100, z_top + flange_t)))
checks.append(("screw heads clear the plate and its rib", worst, 0.2))
low = min(sole_z(screw_x, y, 1) for y in (-screw_y, screw_y))
checks.append(("pilot holes short of the sole", (z_top - pilot_deep) - low, 2.0))
checks.append(("spigot short of the hole's floor", cross_hole - hub_recess - spigot_len, 0.2))
bb = FOOT[1].optimalBoundingBox()   # BoundBox is loose on B-splines
checks.append(("foot's lowest point at -HIP (the sim's)", -abs(bb.ZMin + HIP), -0.05))

print("\nDuckling v1 checks:")
fails = 0
for name, val, need in checks:
    ok = val >= need
    fails += not ok
    print(f"  [{'OK ' if ok else 'FAIL'}] {name}: {val:6.2f}  (>= {need})")

print(f"\n  Foot: x {bb.XMin:.1f}..{bb.XMax:.1f}, y {bb.YMin:.1f}..{bb.YMax:.1f}, "
      f"z {bb.ZMin:.1f}..{max(v.Z for v in FOOT[1].Vertexes):.1f} (top {z_top:.1f}; {z_top + HIP:.1f} thick at most)")
print(f"  The cube's bottom {HIP - half:.0f} off the ground.")
for n, sh in (("plate", PLATE[1]), ("foot", FOOT[1])):
    c = sh.CenterOfMass
    print(f"  {n}: {sh.Volume / 1000:.1f} cm3 ({sh.Volume / 1000 * 1.27:.0f} g solid PETG),"
          f" centre ({c.x:.1f}, {c.y:.1f}, {c.z:.1f})")

# ═══════════════════════════════════════════════════════════════════
# DOCUMENT + EXPORT (print-oriented)
# ═══════════════════════════════════════════════════════════════════

def on_bed(shape, axis=None, deg=0.0):
    s = rot(shape, axis, deg) if axis else shape.copy()
    bb = s.BoundBox
    s.translate(Vector(-bb.Center.x, -bb.Center.y, -bb.ZMin))
    return s


DOC_NAME = "Animo_Duck"
if App.ActiveDocument and App.ActiveDocument.Name == DOC_NAME:
    App.closeDocument(DOC_NAME)
doc = App.newDocument(DOC_NAME)
for name, sh in (("PlateRight", PLATE[1]), ("PlateLeft", PLATE[-1]),
                 ("FootRight", FOOT[1]), ("FootLeft", FOOT[-1]),
                 ("CubeGhost", CUBE)):
    o = doc.addObject("Part::Feature", name)
    o.Shape = sh
doc.recompute()

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl", "animo_duck", MATERIAL)
os.makedirs(out, exist_ok=True)
# The plate is symmetric fore and aft, so the left one is the right one
# turned round: print two. The feet are mirror images: one of each.
EXPORT = [("animo_duck_Plate", on_bed(PLATE[1], Y_AXIS, 90)),         # outer face down
          ("animo_duck_FootRight", on_bed(FOOT[1], X_AXIS, 180)),     # top down, sole up
          ("animo_duck_FootLeft", on_bed(FOOT[-1], X_AXIS, 180))]
for name, sh in EXPORT:
    sh.exportStl(os.path.join(out, name + ".stl"))
print(f"\n  STL -> {out}  ({fails} FAIL)")

view = os.environ.get("ANIMO_VIEW")     # assembled copies, for a picture
if view:
    for name, sh in (("DuckLegRight", LEG[1]), ("DuckLegLeft", LEG[-1]), ("Cube", CUBE)):
        sh.exportStl(os.path.join(view, name + ".stl"))
