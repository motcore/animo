# Animo v1 — the configurable cube (the "small league")
# FreeCAD Python Macro
#
# Not a clutch: one MG90D (position) or MG90S 360 (continuous) per side wall,
# both the same body, so ONE wall module serves both. A blank wall is the
# same plate without the servo. Top and bottom are for stacking and for the
# half-cube "brain" (battery, ESP32, drivers) — not drawn yet.
#
# The rules this file is built on (user, 2026-10-14/15):
#   - every output shaft on the CENTRE of its wall: symmetry is fundamental;
#   - the outer face is FLAT: the output is a female D socket flush with it
#     (user, 2026-10-15), and four disc magnets sit FLUSH in round holes near
#     the corners to hold the model on. The model brings the male D, which
#     centres it and lets it in one way only, so the servo's zero is known;
#   - all four wall modules are IDENTICAL, only rotated about Z.
#
# Layout (the user's, 2026-10-15): the four servos stand upright — case length
# vertical, shaft toward their own wall — in a pinwheel round the cube's
# centre, each one's base against its neighbour's side. Seen from above the
# pack is a square of (servo height + servo thickness), with a square hole in
# the middle for the wiring. Each wall's gears are flat discs parallel to the
# wall, in the thin slab between the servo pack and the wall: the neighbour
# servo that lies along that wall stays behind the slab. The servo's shaft is
# ~14 mm off the wall centre; two equal gears, servo gear -> output gear, bring
# it to the centre 1:1 with the servo's whole travel (~180 deg, or endless on a
# 360). The servo gear sweeps where the lower tab's post would run, so that
# post steps down and passes UNDER the gear on its way to the wall.
#
# Coordinates: origin = cube centre, +Z up. One module is built for the +Y
# (north) wall: +Y outward, X across the wall, Z up. It is then rotated onto
# its wall about Z — rotations only, never a mirror.
#
# Run headless to read the checks:
#   "C:/Program Files/FreeCAD 1.0/bin/freecadcmd.exe" animo_v1_cube.py
# or from FreeCAD: Macro -> Macros -> animo_v1_cube.py -> Execute

import FreeCAD as App
import Part
import math
import os

try:
    import FreeCADGui as Gui
    HAS_GUI = True
except ImportError:
    HAS_GUI = False

# ═══════════════════════════════════════════════════════════════════
# PARAMETERS
# ═══════════════════════════════════════════════════════════════════

# ── Servo MG90D / MG90S 360 (datasheet, same as the v6 macro) ───────────────
sv_case_len  = 22.4   # mm — case length (here vertical, Z)
sv_case_t    = 12.5   # mm — case thickness (here along X, across the wall)
sv_case_h    = 22.4   # mm — case height, base to the top of the main case
sv_tower_h   = 28.2   # mm — base to the top of the gear tower
sv_tower_len = 12.2   # mm — the tower's length, at the shaft's end
sv_spline_h  = 4.0    # mm — splined shaft standing off the tower
sv_spline_d  = 4.5    # mm — spline outer diameter: the horn's bore reads 4.28 / 4.49
                      #      (minor / major of its spline), 2026-10-15
sv_shaft_end = 6.25   # mm — shaft axis from the case's end, along its length
sv_tab_z0    = 18.0   # mm — base to the underside of the tab plate
sv_tab_t     = 2.5    # mm — tab plate thickness
sv_tab_out   = 4.7    # mm — how far each tab reaches past the case
sv_tab_hole  = 2.35   # mm — tab hole centre, in from the tab's tip
sv_tab_hole_d = 2.0   # mm — tab hole (M2 screw)

# ── The servo's own double horn, pocketed in the servo gear ─────────────────
# The factory horn grips the spline; its arms, cut down, key it in the gear.
# No horn screw: the wall, g_gap behind the gear, keeps it on the spline.
# Measured 2026-10-15 (averages of 3-5 readings, +0.04 caliper).
horn_hub_d   = 6.82   # mm — avg 6.78 + 0.04
horn_hub_h   = 4.69   # mm — avg 4.65 + 0.04
horn_arm_t   = 2.08   # mm — arms' thickness, flush with the hub's top (2.04 + 0.04)
horn_arm_w   = 5.13   # mm — their width where they meet the hub (avg 5.09 + 0.04)
horn_arm_len = 9.0    # mm — tip to tip, CUT to this (the gear's root is Ø12.4)
horn_lift    = 1.5    # mm — the hub's underside above the tower: ~1.5, "maybe a
                      #      bit less" (can't get the caliper in). The cube follows it
horn_grow    = 0.2    # mm — pocket over the horn (holes print ~0.2 small)

# ── Gears: two identical spur gears per wall, 1:1 ───────────────────────────
g_mod   = 0.8    # mm — module (prints with a 0.4 nozzle)
g_z     = 18     # teeth. Their centre distance sets where the servo stands:
                 #      the pinwheel opens up if it has to
g_alpha = 20.0   # deg — pressure angle
g_gap   = 0.3    # mm — axial gap: servo pack to the gears, gears to the wall
g_w_min = 3.7    # mm — face width at least; the gears take whatever the slab gives
wall_min = 1.6   # mm — wall left behind the servo gear's boss recess

# ── Clearances and walls ────────────────────────────────────────────────────
clr      = 0.5   # mm — general clearance between parts that do not touch
pack_gap = 0.5   # mm — between neighbouring servos in the pinwheel, at least
wall_t   = 3.0   # mm — wall plate (and deck) thickness
col_w_max = 7.5  # mm — corner columns of the frame, square: this, or as big as
                 #      the servo gear passing the corner lets them
screw_in = 4.0   # mm — deck screws in from both outer edges (margin round the
                 #      countersunk head)
plate_gap = 0.2  # mm — wall plate to columns and decks
# Walls held by tongue and groove: a tongue on the wall's top and bottom
# edges sits in a groove along each edge of the decks. Closing the decks
# traps every wall; swapping one means opening the top deck.
tongue_t  = 1.6  # mm — tongue thickness, centred in the wall
tongue_h  = 1.2  # mm — how far it enters the deck
groove_w  = 1.9  # mm — groove width (tongue + 0.3: holes print ~0.2 small)
groove_d  = 1.4  # mm — groove depth (tongue + 0.2)

# ── Output: a printed hub with a female D, no bearings ──────────────────────
# Two parts, captured by the wall alone: the HUB comes in from outside, its
# flange sitting in a counterbore flush with the face (it cannot go in); the
# GEAR is pressed onto the hub's hex from inside (it cannot come out — its
# face meets the wall). The hub turns in a plain hole.
hub_d      = 10.0   # mm — hub diameter, runs in the wall
hub_clear  = 0.2    # mm — radial clearance: hole 10.4. Printed OD comes ~+0.07 (PETG cross,
                    #      4.6 -> 4.67 avg of 10, 2026-10-15) and a turning fit wants +0.3 (T1.1)
hub_recess = 0.2    # mm — flange face below the outer face, so it never rubs
flange_d   = 14.0   # mm
flange_t   = 1.2    # mm — counterbore depth (the flange is this less the recess)
hex_af     = 8.0    # mm — hub's hex spigot, across flats, through the gear
hex_hole   = 8.25   # mm — gear's hex hole, AF (press): spigot ~8.07 printed + 0.2 (D5 -> 5.2)
sock_d     = 6.0    # mm — socket, round part
sock_flat  = 4.8    # mm — socket across the flat (the D)
sock_depth = 4.5    # mm — socket depth from the outer face

# ── Holes (calibrated in PLA, test-status T1.1, 2026-10) ────────────────────
m2_pilot   = 1.6    # mm — M2 self-tapping pilot (as the v6 servo cradle)
m3_pilot   = 2.5    # mm — M3 self-tapping pilot
m3_clear   = 3.3    # mm

# ── Model magnets on the outer face, flush ──────────────────────────────────
mag_d, mag_h = 6.0, 2.0    # disc magnet N52, flush with the face (user: 6 x 2)
mag_hole   = 6.2    # mm — press-ish (holes print ~0.2 small)
loc_in     = mag_d / 2.0 + 2.5   # mm — magnet centre in from the plate's edge
mag_back   = 1.5    # mm — wall left behind each magnet (a boss inside)

# ── Servo mounting posts (from the wall plate down to the tabs) ─────────────
post_gap   = 0.3    # mm — post to the servo case's end
post_step  = 4.0    # mm — height of the lower post's step under the gear

# ── The brain: a half cube that REPLACES the top deck ───────────────────────
# Its floor is the deck itself (grooves, columns, screws); on it a box with
# the electronics, closed by a flat lid (the top stacking face). Cube + brain
# stand 1.5 cubes tall. Board sizes are their listed outlines — envelopes.
cable_hole = 12.0   # mm — square hole through every deck, over the pack's hole
brain_wall = 2.0    # mm — the brain box's side walls
lid_t      = 3.0    # mm
lid_boss   = 5.0    # mm — square corner bosses for the lid's two screws
lid_screw_in = 4.5  # mm — lid screws in from the edges
standoff   = 2.0    # mm — boards sit this far off the floor
usb_w, usb_h = 10.0, 4.5   # mm — USB-C opening in the wall (plug overmould)
# (name, x0, x1, y0, y1, height) on the floor, z from the floor + standoff
BOARDS = [("Esp32",   4.15, 26.65,   7.5, 25.5, 4.5),   # ESP32-C3 SuperMini, USB +X
          ("Charger", 0.65, 26.65, -21.7, -4.7, 4.5),   # TP4056 USB-C, USB +X
          ("Boost", -26.5, -6.5,  -24.5,  0.5, 6.0)]    # 5 V boost
# Power switch: a KCD11 mini rocker (3 A — the servos' current goes through
# it; a mini slide switch is 0.5 A), snapped into a cutout in the +Y wall,
# lying on its side under the battery. Sizes from listings — MEASURE.
sw_cut   = (13.5, 9.0)    # mm — panel cutout (along X, along Z)
sw_body  = (13.2, 8.7, 14.0)   # mm — behind the panel, terminals included
sw_bezel = (15.2, 10.0, 2.0)   # mm — the face outside the wall
sw_x     = -11.4               # mm — centre along the +Y wall
sw_z     = 5.5                 # mm — centre above the floor
battery = (50.0, 34.0, 10.0)   # 103450 LiPo 1S, ~1800 mAh, under the lid
# Holding them, no screws, no glue: each board drops into a CRADLE on the
# floor (a low wall round its outline, a ledge it rests on, two snap lips on
# its long sides); its USB end butts the box wall, which is why that side of
# the cradle is open. The battery sits in a FRAME under the lid, on a strip
# of double-sided foam tape, and comes out with the lid.
pcb_t      = 1.0    # mm — board thickness (MEASURE the real ones)
cradle_w   = 1.2    # mm — cradle wall
cradle_clr = 0.3    # mm — board to cradle wall
ledge_w    = 1.0    # mm — the ledge the board rests on, inside the wall
lip        = 0.4    # mm — snap lip overhang over the board's edge
lip_h      = 0.8    # mm
comp_inset = 0.6    # mm — components keep this far from the long edges
frame_h    = 6.0    # mm — battery frame depth under the lid
frame_t    = 1.2    # mm
frame_clr  = 0.4    # mm
tape_t     = 3.0    # mm — foam pad between battery and lid (50 x 30 x 3, Amazon B0CGRCD27D)

EXPORT_STL = True
RUN_CHECKS = None   # None: checks when headless, skipped in the GUI

# ═══════════════════════════════════════════════════════════════════
# DERIVED GEOMETRY  (everything for the north module, +Y wall)
# ═══════════════════════════════════════════════════════════════════

r_pitch = g_mod * g_z / 2.0
r_tip   = r_pitch + g_mod
a_mesh  = 2 * r_pitch                  # centre distance, equal gears

# The pinwheel. Its square is (tower height + thickness + gap); each servo's
# tower top is on the pack's edge, its base against the neighbour's side. The
# servo shaft must sit exactly one centre distance off the wall centre, so the
# gap opens up to whatever that asks.
pack_half = max((sv_tower_h + sv_case_t + pack_gap) / 2.0,
                a_mesh + sv_case_t / 2.0)
pack_gap_real = 2 * pack_half - sv_tower_h - sv_case_t
sv_x0   = -pack_half                    # north servo: X from sv_x0 ...
sv_x1   = sv_x0 + sv_case_t             # ... to sv_x1
y_tower = pack_half                     # tower top
y_base  = y_tower - sv_tower_h          # = -pack_half + thickness + gap
y_tab0  = y_base + sv_tab_z0
y_tab1  = y_tab0 + sv_tab_t

# Vertical: the shaft at the case's LOWER end, at z = 0 (the output's height).
x_sv    = (sv_x0 + sv_x1) / 2.0         # shaft: centred in the thickness
case_z0 = -sv_shaft_end
case_z1 = case_z0 + sv_case_len
tab_z0  = case_z0 - sv_tab_out
tab_z1  = case_z1 + sv_tab_out

# Radial stack, from the pack outward: gear slab, wall. The horn stands
# taller than a plain gear slab, so the servo gear's CENTRE (a boss inside its
# root circle) stands proud toward the wall and sits in a shallow recess in
# the wall's inner face. The wall goes as far out as that recess needs to
# leave wall_min behind it; the gears' teeth widen to fill the slab.
y_g0    = y_tower + g_gap               # gear slab, inner face
y_horn  = y_tower + horn_lift + horn_hub_h      # horn top
y_sg1   = y_horn + horn_grow                    # servo gear boss, outer face
y_wi    = max(y_g0 + g_w_min + g_gap,           # wall inner face
              y_sg1 + g_gap - (wall_t - wall_min))
y_g1    = y_wi - g_gap                  # gear slab, outer face
g_w     = y_g1 - y_g0                   # gears' face width
sg_boss_r = r_pitch - 1.25 * g_mod             # boss = the gear's root circle
sg_recess = max(0.0, y_sg1 + g_gap - y_wi)      # wall recess behind the boss
y_wo    = y_wi + wall_t                 # wall outer face = cube half side
half_in = y_wi                          # cube centre to a wall's inner face

# Shafts in the wall plane (x, z).
S_xz = (x_sv, 0.0)                      # servo shaft
O_xz = (0.0, 0.0)                       # output, wall centre


# Tab holes (on the shaft's line, near each tab's tip).
tab_hole_z = (tab_z0 + sv_tab_hole, tab_z1 - sv_tab_hole)

# Corner columns of the frame occupy |x|,|y| in [col_in, y_wo].
col_w  = min(col_w_max, y_wo - (abs(x_sv) + r_tip + clr))
col_in = y_wo - col_w
lim_x  = col_in - plate_gap             # wall plate half width
loc_xz = lim_x - loc_in                 # model pockets at (±loc, ±loc)

# ═══════════════════════════════════════════════════════════════════
# HELPERS
# ═══════════════════════════════════════════════════════════════════

def v(x, y, z):
    return App.Vector(x, y, z)


X_AXIS, Y_AXIS, Z_AXIS = v(1, 0, 0), v(0, 1, 0), v(0, 0, 1)
ORIGIN = v(0, 0, 0)


def box(x0, x1, y0, y1, z0, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, v(x0, y0, z0))


def cyl_y(x, z, r, y0, y1):
    """Cylinder along Y at (x, z), from y0 to y1."""
    return Part.makeCylinder(r, y1 - y0, v(x, y0, z), Y_AXIS)


def cyl_z(x, y, r, z0, z1):
    return Part.makeCylinder(r, z1 - z0, v(x, y, z0), Z_AXIS)


def d_bar_y(x, z, d, flat, y0, y1):
    """A D-section along Y: round Ø d with a flat `flat` across, flat on +X."""
    rod = cyl_y(x, z, d / 2.0, y0, y1)
    cut_x = x + flat - d / 2.0
    return rod.cut(box(cut_x, x + d, y0 - 1, y1 + 1, z - d, z + d))


def add(doc, name, shape, color=(0.8, 0.8, 0.8), transparency=0):
    obj = doc.addObject("Part::Feature", name)
    obj.Shape = shape
    if HAS_GUI and obj.ViewObject is not None:
        obj.ViewObject.ShapeColor = color
        obj.ViewObject.Transparency = transparency
    return obj


def _inv(a):
    return math.tan(a) - a


def gear_outline(z, m, phase):
    """Involute spur gear outline, (u, w) points in its own plane, closed.
    `phase` is the angle of tooth 0's centre line."""
    alpha = math.radians(g_alpha)
    r = m * z / 2.0
    ra, rf, rb = r + m, r - 1.25 * m, r * math.cos(alpha)
    psi_p = math.pi / (2 * z) + _inv(alpha)      # half-tooth angle at the base
    r0 = max(rb, rf)
    flank = []
    n = 8
    for i in range(n + 1):
        rho = r0 + (ra - r0) * i / n
        a_rho = math.acos(min(1.0, rb / rho))
        flank.append((rho, psi_p - _inv(a_rho)))
    if rf < rb:
        flank.insert(0, (rf, flank[0][1]))
    pts = []
    for k in range(z):
        th = phase + 2 * math.pi * k / z
        for rho, psi in flank:                    # right flank, root -> tip
            pts.append((rho * math.cos(th - psi), rho * math.sin(th - psi)))
        psi_tip = flank[-1][1]
        for j in range(1, 4):                     # tip land
            t = th - psi_tip + 2 * psi_tip * j / 4
            pts.append((ra * math.cos(t), ra * math.sin(t)))
        for rho, psi in reversed(flank):          # left flank, tip -> root
            pts.append((rho * math.cos(th + psi), rho * math.sin(th + psi)))
        psi_root = flank[0][1]
        t0, t1 = th + psi_root, th + 2 * math.pi / z - psi_root
        for j in range(1, 4):                     # root land
            t = t0 + (t1 - t0) * j / 4
            pts.append((rf * math.cos(t), rf * math.sin(t)))
    return pts


def make_gear(cx, cz, phase, y0, y1, bore=None):
    pts = [v(cx + u, y0, cz + w) for u, w in gear_outline(g_z, g_mod, phase)]
    face = Part.Face(Part.makePolygon(pts + [pts[0]]))
    g = face.extrude(v(0, y1 - y0, 0))
    if bore is not None:
        g = g.cut(bore)
    return g


def mesh_phase(phase_a, za, a_xz, b_xz, zb):
    """Phase for gear B so it meshes with A (external mesh): a tooth of A at
    theta + e (theta = direction A->B) meets a GAP of B at theta + pi - e*za/zb."""
    th = math.atan2(b_xz[1] - a_xz[1], b_xz[0] - a_xz[0])
    pitch_a = 2 * math.pi / za
    e = (phase_a - th) % pitch_a
    if e > pitch_a / 2:
        e -= pitch_a
    return th + math.pi - e * za / zb + math.pi / zb


# ═══════════════════════════════════════════════════════════════════
# PARTS — north module
# ═══════════════════════════════════════════════════════════════════

def make_servo():
    """MG90D envelope, standing: case, gear tower at the shaft (lower) end,
    tab plate with its two holes, spline."""
    case = box(sv_x0, sv_x1, y_base, y_base + sv_case_h, case_z0, case_z1)
    tower = box(sv_x0, sv_x1, y_base + sv_case_h - 0.01, y_tower,
                case_z0, case_z0 + sv_tower_len)
    tabs = box(sv_x0, sv_x1, y_tab0, y_tab1, tab_z0, tab_z1)
    for hz in tab_hole_z:
        tabs = tabs.cut(cyl_y(x_sv, hz, sv_tab_hole_d / 2.0, y_tab0 - 1, y_tab1 + 1))
    spline = cyl_y(x_sv, 0.0, sv_spline_d / 2.0, y_tower - 0.01,
                   y_tower + sv_spline_h)
    return case.fuse([tower, tabs, spline]).removeSplitter()


def _horn(grow, y_top_extra=0.0):
    """The horn (or, grown, its pocket): hub + two arms along X, arms flush
    with the hub's top. y_top_extra lifts the pocket's top through the gear."""
    y0 = y_horn - horn_hub_h
    hub = cyl_y(x_sv, 0.0, horn_hub_d / 2.0 + grow, y0 - grow, y_horn + y_top_extra)
    half = horn_arm_len / 2.0 + grow
    arms = box(x_sv - half, x_sv + half, y_horn - horn_arm_t - grow,
               y_horn + y_top_extra, -horn_arm_w / 2.0 - grow, horn_arm_w / 2.0 + grow)
    return hub.fuse(arms)


def make_horn():
    h = _horn(0.0).removeSplitter()
    return h.cut(cyl_y(x_sv, 0.0, sv_spline_d / 2.0, y_tower - 1, y_horn - 1.0))


def make_servo_gear():
    """Servo gear, with the horn's shape as a pocket right through it.
    Printed flat, the SERVO side down (its floor with the spline hole on the
    bed, the boss up), so the horn's pocket opens upward."""
    g = make_gear(S_xz[0], S_xz[1], PH_S, y_g0, y_g1)
    if y_sg1 > y_g1:
        g = g.fuse(cyl_y(x_sv, 0.0, sg_boss_r, y_g1 - 0.01, y_sg1)).removeSplitter()
    g = g.cut(_horn(horn_grow, 1.0))
    # The spline passes through the floor under the horn; that floor is also
    # what stops the horn sliding through toward the servo.
    return g.cut(cyl_y(x_sv, 0.0, sv_spline_d / 2.0 + clr, y_g0 - 1, y_sg1))


def hex_y(af, y0, y1):
    """Hexagonal prism along Y on the wall centre, `af` across flats, flats
    top and bottom."""
    r = af / math.sqrt(3.0)
    pts = [v(r * math.cos(math.radians(60 * k)), y0, r * math.sin(math.radians(60 * k)))
           for k in range(6)]
    return Part.Face(Part.makePolygon(pts + [pts[0]])).extrude(v(0, y1 - y0, 0))


def make_output_gear():
    """Output gear, pressed onto the hub's hex. Printed flat."""
    return make_gear(0, 0, PH_O, y_g0, y_g1, hex_y(hex_hole, y_g0 - 1, y_g1 + 1))


def make_output_hub():
    """Hub: flange (flush in the wall's counterbore), round part turning in
    the wall, hex spigot through the gear, female D socket. PETG — the torque
    crosses the layers. Printed flange down."""
    y_step = y_wo - flange_t
    flange = cyl_y(0, 0, flange_d / 2.0, y_step, y_wo - hub_recess)
    hub = cyl_y(0, 0, hub_d / 2.0, y_g1 - 0.01, y_step + 0.01)
    spigot = hex_y(hex_af, y_g0, y_g1)
    h = flange.fuse([hub, spigot]).removeSplitter()
    return h.cut(d_bar_y(0, 0, sock_d, sock_flat, y_wo - sock_depth, y_wo + 1))


def make_wall():
    """The wall module: flat plate, the output's hole, two servo posts, the
    four model locators. One part, printed outer face down."""
    lim_z = half_in - plate_gap
    plate = box(-lim_x, lim_x, y_wi, y_wo, -lim_z, lim_z)
    # Tab posts. Upper: straight up to the wall. Lower: the servo gear sweeps
    # through its path, so it stops short of the gear slab, steps down and
    # runs to the wall UNDER the gear's tip.
    pz0, pz1 = tab_z0, case_z0 - post_gap
    y_stop = y_g0 - clr
    z_under = -r_tip - clr                      # top of the step, under the gear
    z_low = z_under - post_step
    posts = [box(sv_x0, sv_x1, y_tab1, y_wi + 0.01, case_z1 + post_gap, tab_z1),
             box(sv_x0, sv_x1, y_tab1, y_stop, min(pz0, z_low), pz1),
             box(sv_x0, sv_x1, y_stop - 0.01, y_wi + 0.01, z_low, z_under)]
    lbosses = [cyl_y(sx * loc_xz, sz * loc_xz, mag_d / 2.0 + 2.0,
                     y_wo - mag_h - mag_back, y_wi + 0.01)
               for sx in (1, -1) for sz in (1, -1)]
    y_mid = (y_wi + y_wo) / 2.0
    tongues = [box(-lim_x, lim_x, y_mid - tongue_t / 2, y_mid + tongue_t / 2,
                   lim_z - 0.01, half_in + tongue_h),
               box(-lim_x, lim_x, y_mid - tongue_t / 2, y_mid + tongue_t / 2,
                   -half_in - tongue_h, -lim_z + 0.01)]
    w = plate.fuse(posts + lbosses + tongues).removeSplitter()
    # Recess behind the servo gear's boss.
    if sg_recess > 0:
        w = w.cut(cyl_y(x_sv, 0.0, sg_boss_r + clr, y_wi - 1, y_wi + sg_recess))
    # Output: the hub's plain bearing hole, counterbored for its flange.
    w = w.cut(cyl_y(0, 0, hub_d / 2.0 + hub_clear, y_wi - 1, y_wo + 1))
    w = w.cut(cyl_y(0, 0, flange_d / 2.0 + hub_clear, y_wo - flange_t, y_wo + 1))
    # Servo screws: M2 pilots up the posts.
    for hz in tab_hole_z:
        w = w.cut(cyl_y(x_sv, hz, m2_pilot / 2.0, y_tab1 - 1, y_tab1 + 6.0))
    # Model magnets: flush, in round holes.
    for sx in (1, -1):
        for sz in (1, -1):
            w = w.cut(cyl_y(sx * loc_xz, sz * loc_xz, mag_hole / 2.0,
                            y_wo - mag_h, y_wo + 1))
    return w


def make_magnet(sx, sz):
    cx, cz = sx * loc_xz, sz * loc_xz
    return cyl_y(cx, cz, mag_d / 2.0, y_wo - mag_h, y_wo)


def make_deck():
    """One deck — top and bottom are the SAME part. Drawn as the bottom one:
    a plate with two full-height corner columns on one diagonal and two
    countersunk M3 holes on the other. The top deck is this part turned over
    (180 deg about X): its columns come down onto the bottom one's holes and
    the other way round. Four M3x10 countersunk, two from each face, into
    pilots in the column ends."""
    d = box(-y_wo, y_wo, -y_wo, y_wo, -y_wo, -half_in)
    cxy = y_wo - screw_in
    for sx, sy in ((1, 1), (-1, -1)):            # columns
        x0, x1 = sorted((sx * col_in, sx * y_wo))
        y0, y1 = sorted((sy * col_in, sy * y_wo))
        d = d.fuse(box(x0, x1, y0, y1, -half_in - 0.01, half_in))
        d = d.cut(cyl_z(sx * cxy, sy * cxy, m3_pilot / 2.0, half_in - 8.0,
                        half_in + 1))
    y_mid = (y_wi + y_wo) / 2.0                  # grooves for the walls
    for rot in (0, 90, 180, 270):
        g = box(-col_in, col_in, y_mid - groove_w / 2, y_mid + groove_w / 2,
                -half_in - groove_d, -half_in + 0.01)
        g.rotate(ORIGIN, Z_AXIS, rot)
        d = d.cut(g)
    d = d.cut(box(-cable_hole / 2, cable_hole / 2, -cable_hole / 2, cable_hole / 2,
                  -y_wo - 1, -half_in + 1))      # cables up to the brain
    for sx, sy in ((1, -1), (-1, 1)):            # countersunk holes
        x, y = sx * cxy, sy * cxy
        d = d.cut(cyl_z(x, y, m3_clear / 2.0, -y_wo - 1, -half_in + 1))
        d = d.cut(Part.makeCone(3.2, m3_clear / 2.0, 1.6,
                                v(x, y, -y_wo - 0.001), Z_AXIS))
    return d.removeSplitter()


def deck_top(deck):
    s = deck.copy()
    s.rotate(ORIGIN, X_AXIS, 180)
    return s


# Brain heights: floor = the top deck's plate (z half_in .. y_wo).
brain_h = 2 * y_wo / 2.0 + wall_t        # half a cube above the walls' tops
z_top   = half_in + brain_h              # the lid's top face
z_lid0  = z_top - lid_t                  # the lid's underside
b_in    = y_wo - brain_wall              # brain box inside half width
LID_CORNERS = ((1, -1), (-1, 1))         # the top deck's column corners: free
                                         # above (its screws are at the others)


def make_brain_base(deck):
    """The top deck with the brain's box on it: side walls, two lid bosses,
    openings for the two USB-C plugs and the switch on +X. One printed part,
    printed with the box's open rim on the bed, the deck's columns standing up."""
    b = deck_top(deck)
    shell = box(-y_wo, y_wo, -y_wo, y_wo, y_wo - 0.01, z_lid0).cut(
        box(-b_in, b_in, -b_in, b_in, y_wo - 1, z_lid0 + 1))
    bosses = [box(*sorted((sx * (b_in - lid_boss), sx * b_in + sx * 0.01)),
                  *sorted((sy * (b_in - lid_boss), sy * b_in + sy * 0.01)),
                  y_wo - 0.01, z_lid0) for sx, sy in LID_CORNERS]
    b = b.fuse([shell] + bosses + make_cradles()).removeSplitter()
    for sx, sy in LID_CORNERS:
        b = b.cut(cyl_z(sx * (y_wo - lid_screw_in), sy * (y_wo - lid_screw_in),
                        m3_pilot / 2.0, z_lid0 - 8.0, z_lid0 + 1))
    zb = y_wo + standoff
    for name, x0, x1, y0, y1, h in BOARDS:
        if x1 < b_in - 1.0:                # only the boards whose USB meets +X
            continue
        yc = (y0 + y1) / 2.0
        zc = zb + 2.5
        b = b.cut(box(b_in - 1, y_wo + 1, yc - usb_w / 2, yc + usb_w / 2,
                      zc - usb_h / 2, zc + usb_h / 2))
    zc = y_wo + sw_z                       # the switch's cutout, +Y wall
    b = b.cut(box(sw_x - sw_cut[0] / 2, sw_x + sw_cut[0] / 2, b_in - 1, y_wo + 1,
                  zc - sw_cut[1] / 2, zc + sw_cut[1] / 2))
    return b


def make_switch():
    zc = y_wo + sw_z
    body = box(sw_x - sw_body[0] / 2, sw_x + sw_body[0] / 2, y_wo - sw_body[2], y_wo,
               zc - sw_body[1] / 2, zc + sw_body[1] / 2)
    bezel = box(sw_x - sw_bezel[0] / 2, sw_x + sw_bezel[0] / 2, y_wo, y_wo + sw_bezel[2],
                zc - sw_bezel[1] / 2, zc + sw_bezel[1] / 2)
    return body.fuse(bezel)


def make_cradles():
    """One cradle per board: wall, ledge and two snap lips; the side toward
    the box wall stays open (the board's USB end butts that wall)."""
    zf = y_wo - 0.01                       # the floor
    z_pcb1 = y_wo + standoff + pcb_t       # board's top face
    out = []
    for name, x0, x1, y0, y1, h in BOARDS:
        c = cradle_clr
        open_px = x1 > b_in - 1.0          # against the +X wall
        open_mx = x0 < -b_in + 1.0         # against the -X wall
        X0 = max(x0 - c - cradle_w, -b_in + 0.01)
        X1 = min(x1 + c + cradle_w, b_in - 0.01)
        wall = box(X0, X1, y0 - c - cradle_w, y1 + c + cradle_w, zf, z_pcb1 + lip_h)
        wall = wall.cut(box(x0 - c, x1 + c, y0 - c, y1 + c, zf - 1, z_pcb1 + lip_h + 1))
        if open_px:
            wall = wall.cut(box(x1, b_in + 1, y0 - c, y1 + c, zf - 1, z_pcb1 + 5))
        if open_mx:
            wall = wall.cut(box(-b_in - 1, x0, y0 - c, y1 + c, zf - 1, z_pcb1 + 5))
        ledge = box(x0 - c, x1 + c, y0 - c, y1 + c, zf, y_wo + standoff).cut(
            box(x0 + ledge_w, x1 - ledge_w, y0 + ledge_w, y1 - ledge_w,
                zf - 1, y_wo + standoff + 1))
        lips = [box(x0 + 2.0, x1 - 2.0, y0 - c - 0.01, y0 + lip, z_pcb1, z_pcb1 + lip_h),
                box(x0 + 2.0, x1 - 2.0, y1 - lip, y1 + c + 0.01, z_pcb1, z_pcb1 + lip_h)]
        out.append(wall.fuse([ledge] + lips))
    return out


def make_brain_lid():
    """Flat lid, the top stacking face: two countersunk M3 into the bosses,
    and the battery's frame underneath."""
    l = box(-y_wo, y_wo, -y_wo, y_wo, z_lid0, z_top)
    L, W, H = battery
    fx, fy = L / 2 + frame_clr, W / 2 + frame_clr
    frame = box(-fx - frame_t, fx + frame_t, -fy - frame_t, fy + frame_t,
                z_lid0 - frame_h, z_lid0 + 0.01).cut(
        box(-fx, fx, -fy, fy, z_lid0 - frame_h - 1, z_lid0 + 1))
    l = l.fuse(frame).removeSplitter()
    for sx, sy in LID_CORNERS:
        x, y = sx * (y_wo - lid_screw_in), sy * (y_wo - lid_screw_in)
        l = l.cut(cyl_z(x, y, m3_clear / 2.0, z_lid0 - 1, z_top + 1))
        l = l.cut(Part.makeCone(m3_clear / 2.0, 3.2, 1.6,
                                v(x, y, z_top - 1.6 + 0.001), Z_AXIS))
    return l


def make_boards():
    """Board envelopes: the PCB, and its components kept off the long edges
    (where the cradle's lips reach over)."""
    zb = y_wo + standoff
    out = []
    for n, x0, x1, y0, y1, h in BOARDS:
        pcb = box(x0, x1, y0, y1, zb, zb + pcb_t)
        comp = box(x0, x1, y0 + comp_inset, y1 - comp_inset, zb + pcb_t - 0.01, zb + h)
        out.append((n, pcb.fuse(comp).removeSplitter()))
    return out


def make_battery():
    L, W, H = battery
    return box(-L / 2, L / 2, -W / 2, W / 2, z_lid0 - tape_t - H, z_lid0 - tape_t)


# Gear phases: servo gear arbitrary, the rest follow the mesh.
PH_S = 0.0
PH_O = mesh_phase(PH_S, g_z, S_xz, O_xz, g_z)

# ═══════════════════════════════════════════════════════════════════
# ASSEMBLY
# ═══════════════════════════════════════════════════════════════════

# (name, rotation about Z). All four modules identical.
WALLS = [("N", 0), ("W", 90), ("S", 180), ("E", 270)]

MODULE = [("Wall", make_wall(), (0.95, 0.75, 0.30), 0),
          ("Servo", make_servo(), (0.20, 0.35, 0.80), 0),
          ("ServoGear", make_servo_gear(), (0.90, 0.90, 0.90), 0),
          ("Horn", make_horn(), (0.95, 0.95, 0.95), 0),
          ("OutputGear", make_output_gear(), (0.90, 0.90, 0.90), 0),
          ("OutputHub", make_output_hub(), (0.85, 0.45, 0.20), 0)]
MODULE += [(f"Magnet{'P' if sx > 0 else 'M'}{'P' if sz > 0 else 'M'}",
            make_magnet(sx, sz), (0.55, 0.55, 0.60), 0)
           for sx in (1, -1) for sz in (1, -1)]


def place(shape, rot):
    """A copy rotated onto its wall about Z. Rotations only, never a mirror;
    always from a fresh copy (Shape.rotate mutates in place)."""
    s = shape.copy()
    if rot:
        s.rotate(ORIGIN, Z_AXIS, rot)
    return s


PARTS = []   # (name, shape, color, transparency)
for wname, rot in WALLS:
    for pname, sh, col, tr in MODULE:
        PARTS.append((f"{pname}_{wname}", place(sh, rot), col, tr))
DECK = make_deck()
PARTS.append(("DeckBottom", DECK, (0.45, 0.55, 0.75), 60))
PARTS.append(("BrainBase", make_brain_base(DECK), (0.45, 0.55, 0.75), 60))
PARTS.append(("BrainLid", make_brain_lid(), (0.45, 0.55, 0.75), 60))
PARTS += [(n, sh, (0.15, 0.50, 0.25), 0) for n, sh in make_boards()]
PARTS.append(("Battery", make_battery(), (0.85, 0.20, 0.20), 0))
PARTS.append(("Switch", make_switch(), (0.15, 0.15, 0.15), 0))

DOC_NAME = "Animo_v1"
if App.ActiveDocument and App.ActiveDocument.Name == DOC_NAME:
    App.closeDocument(DOC_NAME)
doc = App.newDocument(DOC_NAME)
for n, sh, col, tr in PARTS:
    add(doc, n, sh, col, tr)

# Tree groups: one per wall (its module, servo and all), one for the frame,
# so each can be shown or hidden at once (select the group, Space).
WALL_LABEL = {"N": "Pared N", "W": "Pared O", "S": "Pared S", "E": "Pared E"}
_groups = {}
for _obj in list(doc.Objects):
    if _obj.TypeId != "Part::Feature":
        continue
    _w = _obj.Name.rsplit("_", 1)[-1] if "_" in _obj.Name else ""
    _label = WALL_LABEL.get(_w, "Marco")
    if _obj.Name.startswith(("Brain", "Esp32", "Charger", "Boost", "Switch",
                             "Battery")):
        _label = "Cerebro"
    if _label not in _groups:
        _groups[_label] = doc.addObject("App::DocumentObjectGroup",
                                        "G_" + _label.replace(" ", "_"))
        _groups[_label].Label = _label
    _groups[_label].addObject(_obj)
doc.recompute()
if HAS_GUI and getattr(App, "GuiUp", 0):
    Gui.SendMsgToActiveView("ViewFit")
    Gui.activeDocument().activeView().viewIsometric()

# ── Printable parts to STL (north module's frame; orient in the slicer) ─────
if EXPORT_STL:
    _here = os.path.dirname(os.path.abspath(__file__)) if "__file__" in dir() \
        else App.getUserMacroDir(True)
    _pdir = os.path.join(_here, "stl", "animo_v1")
    os.makedirs(_pdir, exist_ok=True)
    _by = dict((n, sh) for n, sh, c, t in MODULE)
    for _n in ("Wall", "ServoGear", "OutputGear", "OutputHub"):
        _by[_n].exportStl(os.path.join(_pdir, f"animo_v1_{_n}.stl"))
    DECK.exportStl(os.path.join(_pdir, "animo_v1_Deck.stl"))
    make_brain_base(DECK).exportStl(os.path.join(_pdir, "animo_v1_BrainBase.stl"))
    make_brain_lid().exportStl(os.path.join(_pdir, "animo_v1_BrainLid.stl"))

# ═══════════════════════════════════════════════════════════════════
# CHECKS
# ═══════════════════════════════════════════════════════════════════

if RUN_CHECKS is None:
    RUN_CHECKS = not getattr(App, "GuiUp", 0)

# Pairs that are MEANT to touch or nest (prefix match, same wall).
ALLOWED = {("Horn", "Servo"),             # horn on the spline
           ("Horn", "ServoGear"),         # horn in its pocket
           ("DeckBottom", "BrainBase"),    # columns land on the other deck
           ("BrainBase", "BrainLid"),      # lid on its bosses
           ("Battery", "BrainLid"),        # taped into its frame
           ("Switch", "BrainBase"),        # snapped into its cutout
           ("ServoGear", "OutputGear"),                        # meshing
           ("OutputHub", "OutputGear"),                        # pressed hex
           ("Magnet", "Wall")}


def _allowed(a, b):
    pa, wa = a.rsplit("_", 1) if "_" in a else (a, "")
    pb, wb = b.rsplit("_", 1) if "_" in b else (b, "")
    if wa != wb:
        return False
    for x, y in ALLOWED:
        if (pa.startswith(x) and pb.startswith(y)) or \
           (pa.startswith(y) and pb.startswith(x)):
            return True
    return False


if RUN_CHECKS:
    checks = []
    bad = []
    names = [p[0] for p in PARTS]
    shapes = [p[1] for p in PARTS]
    for i in range(len(PARTS)):
        for j in range(i + 1, len(PARTS)):
            if _allowed(names[i], names[j]):
                continue
            bi, bj = shapes[i].BoundBox, shapes[j].BoundBox
            if not bi.intersect(bj):
                continue
            vol = shapes[i].common(shapes[j]).Volume
            if vol > 1e-3:
                bad.append((names[i], names[j], vol))

    by = dict((p[0], p[1]) for p in PARTS)

    def gap(a, b):
        return by[a].distToShape(by[b])[0]

    d_so = math.hypot(S_xz[0], S_xz[1])
    checks.append(("servo gear - output centre distance", d_so, f"= {a_mesh:.2f}",
                   abs(d_so - a_mesh) < 1e-6))
    checks.append(("pinwheel gap", pack_gap_real, f">= {pack_gap}",
                   pack_gap_real >= pack_gap - 1e-9))
    for a, b, tgt in (("Servo_N", "Servo_E", "> 0, pinwheel neighbours"),
                      ("Servo_N", "Servo_W", "> 0"),
                      ("Servo_N", "Servo_S", "> 0, opposite"),
                      ("Servo_E", "ServoGear_N", "> 0, the one along the wall"),
                      ("Servo_E", "OutputGear_N", "> 0"),
                      ("ServoGear_N", "Wall_N", "> 0, in its recess, over the post"),
                      ("Horn_N", "Wall_N", "> 0, the wall keeps the horn on"),
                      ("ServoGear_N", "Servo_N", "> 0, off the tower"),
                      ("ServoGear_N", "OutputHub_N", "> 0"),
                      ("OutputGear_N", "Wall_N", "> 0"),
                      ("Wall_N", "Servo_E", "> 0, posts"),
                      ("Wall_N", "Wall_E", "> 0, corner"),
                      ("Servo_N", "BrainBase", "> 0"),
                      ("Servo_N", "DeckBottom", "> 0"),
                      ("Servo_E", "BrainBase", "> 0, corner column"),
                      ("Servo_W", "DeckBottom", "> 0, corner column")):
        g = gap(a, b)
        checks.append((f"gap {a} / {b}", g, tgt, g > 0.05))

    hole = 2 * (pack_half - sv_case_t - pack_gap_real)
    print("=" * 72)
    print("Animo v1 — configurable cube, MG90D standing in a pinwheel, 1:1 gear pairs")
    print("-" * 72)
    for label, value, target, ok in checks:
        print(f"  [{'OK ' if ok else 'FAIL'}] {label}:  {value:.3f}  ({target})")
    if bad:
        for a, b, vol in bad:
            print(f"  [FAIL] interference {a} / {b}: {vol:.3f} mm3")
    else:
        print(f"  [OK ] no interference among {len(PARTS)} parts"
              " (designed contacts excepted)")
    print("-" * 72)
    print(f"  CUBE side {2 * y_wo:.1f} mm  (inside {2 * half_in:.1f}, walls {wall_t:.1f})")
    print(f"    = servo pack {2 * pack_half:.1f} (tower {sv_tower_h} + thickness"
          f" {sv_case_t} + gap {pack_gap_real:.2f}) + 2 x {y_wo - pack_half:.1f}"
          f" (gears {g_w:.1f} + gaps + wall {wall_t:.0f})")
    print(f"  Faces FLAT: output = female D Ø{sock_d:.0f} / {sock_flat:.1f} across the"
          f" flat, {sock_depth:.1f} deep, in a Ø{flange_d:.0f} flange flush with the wall")
    print(f"  Deck screws {screw_in:.1f} mm in from the edges: {screw_in - 3.2:.1f} mm"
          f" round the countersunk head; corner columns {col_w:.1f} square")
    print(f"  Hole through the middle of the pack: {hole:.1f} x {hole:.1f} mm (wiring)")
    print(f"  Servo shaft {abs(x_sv):.2f} mm off the wall centre, at the output's height")
    print(f"  Gears: 2 x z{g_z} m{g_mod}, Ø{2 * r_pitch:.1f} pitch, Ø{2 * r_tip:.1f}"
          f" tip, {g_w:.1f} wide, 1:1")
    print(f"  Hub (PETG) from outside, flange in a counterbore; gear pressed on its"
          f" hex {hex_af:.0f} AF from inside;")
    print(f"    turns in a Ø{hub_d + 2 * hub_clear:.1f} hole: no bearings, no shaft,"
          f" captured by the wall alone (axial play {g_gap:.1f})")
    print("  PRINTED per cube: Deck x2 (the same part, one turned over), and per wall:")
    print("           Wall (outer face down), ServoGear, OutputGear, OutputHub (PETG)")
    print("  PURCHASED per wall: MG90D or MG90S 360,")
    print("           2x M2 tab screws, 4 magnets Ø4x2;  per cube: 4x M3x10 countersunk")
    print(f"  Servo gear holds the servo's horn (arms CUT to {horn_arm_len:.0f} mm), no"
          f" horn screw; its centre boss Ø{2 * sg_boss_r:.1f} stands {y_sg1 - y_g1:.2f} proud,")
    print(f"    the wall recessed {sg_recess:.2f} mm behind it"
          f" (wall left there {wall_t - sg_recess:.2f} mm)")
    print(f"  Walls held by a {tongue_t:.1f} x {tongue_h:.1f} tongue top and bottom in"
          " the decks' grooves; open the top deck to swap one")
    print(f"  BRAIN (replaces the top deck): box {2 * y_wo:.1f} x {2 * y_wo:.1f}, cube + brain"
          f" {y_wo + z_top:.1f} tall (1.5 cubes); inside {2 * b_in:.1f} x {2 * b_in:.1f} x"
          f" {z_lid0 - y_wo:.1f}")
    print("    ESP32-C3 SuperMini, TP4056 USB-C, 5 V boost on the floor; KCD11 3 A rocker in +Y;")
    print(f"    LiPo {battery[0]:.0f}x{battery[1]:.0f}x{battery[2]:.0f} under the lid;"
          f" cables up a {cable_hole:.0f} mm hole; USB-C x2 on the +X side")
    print("    boards snap into cradles on the floor; the battery hangs in a frame under")
    print("    the lid on foam tape. MEASURE the real boards (outline, PCB, USB position)")
    print("  OPEN: wiring")
    print("=" * 72)
else:
    print(f"Animo v1: geometry only, checks skipped. Cube side {2 * y_wo:.1f} mm")
