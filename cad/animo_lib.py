# Animo — shared library for the brick and its faces (not a macro)
#
# Words (internal, 2026-11-14 — the market names come later):
#   face      one side's module: a wall plate plus what it carries (motor,
#             output, swing gear, sensor); slides out on its own
#   brick     the frame: two decks with their corner columns, four faces on it
#   Motcore   one or more bricks with their faces — a configuration
#   Coremata  the model that goes on a Motcore
# In the code "Wall" stays the name of the printed plate inside a face.
#
# What differs between faces is the motor and where it sits; all the rest
# lives here, so every face follows one standard and any face can go on any
# side (user, 2026-11-14: frame + faces apart, files named for what they are,
# not by version).
#
#   animo_lib.py            this file: sizes, helpers, the parts every face has
#   animo_face_n20.py       face with a straight N20 gearmotor (being built)
#   animo_face_n20_worm.py  face with a worm N20 lying flat on its plate
#   animo_face_mg90d.py     face with a continuous MG90D (set aside)
#   animo_brick.py          the frame, which face goes on which side, checks
#
# Each face file opens on its own (a brick with four of that face);
# animo_brick.py mixes them. The history (v1 ... v5) is in git and cad/archive/.
#
# Coordinates: origin = cube centre, +Z up. A wall is built as the NORTH one:
# +Y outward, X across the wall, Z up; it is then rotated onto its side about
# Z — rotations only, never a mirror.
#
# The standard every wall shares (2026-11-13/14):
#   - output on a 6700ZZ (10 x 15 x 4) in a 5.6 wall, face flat; hub and gear
#     joined by one M3 from inside, a short hex for the torque;
#   - the model's coupling: a TRAPEZOID socket (a square with one side
#     shorter) in the hub's face, one position, torque only — no click: the
#     Coremata holds its turning parts on itself; ModelKey is the model's
#     template part;
#   - double swing gear: two z12 swing gears on an arm dragged by the pinion
#     (dry friction, no spring), the arm's stops are two wall posts; the
#     firmware parks the arm in the middle, where the output turns free;
#   - AS5600 on a sensor gear, 1:1 with the output, magnet on its back;
#   - no LEGO on the motor wall (it comes back as a wall of its own).

import math
import os

import FreeCAD as App
import Part

try:
    import FreeCADGui as Gui
    HAS_GUI = True
except ImportError:
    HAS_GUI = False

# ═══════════════════════════════════════════════════════════════════
# PARAMETERS — shared by every wall
# ═══════════════════════════════════════════════════════════════════

# ── Material: sets the fits that differ, and where the STLs go ─────────────
# PETG opens the ROUND holes ~0.1 (large holes print ~0.1 tighter: 688 16.2
# vs 16.1); flat faces and the M3 thread stay. ANIMO_MATERIAL=PLA switches.
MATERIAL = os.environ.get("ANIMO_MATERIAL", "PETG")

# ── Cube and frame ──────────────────────────────────────────────────────────
cube_side = 72.0    # mm — 9 LEGO modules (2026-11-11): the grid stays, LEGO is off
deck_t    = 3.0     # mm — deck thickness
clr       = 0.5     # mm — general clearance between parts that do not touch
col_w     = 7.5     # mm — corner columns of the frame, square
screw_in  = 4.0     # mm — deck screws in from both outer edges
col_foot_w = 7.0    # mm — each column's free end, where the other deck's M3 goes,
col_foot_h = 8.5    #      widens to this square for this long (45 deg flare)
foot_clear = 0.3    # mm — the walls' corners are notched round the feet
lip_w     = 1.2     # mm — lip behind the wall plate's edges: how far it reaches in
lip_t     = 1.0     # mm — its thickness
lip_gap_z = 20.2    # mm — side lips stop within this of the wall centre
plate_gap = 0.2     # mm — wall plate to columns and decks
skirt_in  = 1.5     # mm — the decks' skirt reaches this far inside the wall plate
grid      = 8.0     # mm — the 8 mm grid the wall screws sit on
WALL_SCREW_AT = (-3, -3)   # grid steps of the bottom deck's boss; the top deck
                           #      (the same part turned over) puts its own at (+3, +3)
ws_boss_w = 5.0     # mm — wall screw boss, width
ws_boss_d = 3.5     # mm — and depth behind the wall
ws_head_d = 3.8     # mm — M2 countersunk head
ws_clear  = 2.2     # mm — M2 clearance through the wall
ws_screw  = "M2 x 10 countersunk"
cable_hole = (-7.0, -0.5, -5.0, 5.0)   # mm — x0, x1, y0, y1 through the decks

# ── Holes ───────────────────────────────────────────────────────────────────
m2_pilot = 1.6      # mm — M2 self-tapping pilot
m3_pilot = 3.0      # mm — M3 self-tapping pilot (PLA and PETG alike)
m3_clear = 3.3      # mm

# ── Output: hub on a 6700ZZ in the wall, one M3 to the gear ────────────────
brg_od    = 15.0    # mm — 6700ZZ: 10 x 15 x 4 (catalogue; measure + 0.04 when here)
brg_id    = 10.0
brg_w     = 4.0
brg_in_r  = 5.6     # mm — what hub and gear may touch: the inner ring's face
brg_seat  = 15.2    # mm — seat in the wall, PETG (the 688's measured press carried
                    #      over: 16.1 in PLA, PETG ~0.1 tighter); to check
fl_r      = 6.6     # mm — the hub's flange, the model's face; its counterbore
fl_t      = 1.2     #      leaves a 0.7 floor the bearing stops on
fl_relief = 0.2     # mm — flange over the bearing's outer ring: this clear
cb_clear  = 0.3     # mm — counterbore round the flange
hub_recess = 0.2    # mm — flange face below the wall's face, so it never rubs
journal_d = 9.95    # mm — hub in the inner ring (prints ~+0.07: a snug slide)
hex_af    = 8.0     # mm — hub's hex into the gear: torque only, no press
hex_hole  = 8.15    # mm — the gear's hex socket (slides)
hex_len   = 2.0     # mm — how deep the hex goes into the gear
gb_boss_r = 3.5     # mm — gear's boss behind its teeth, for the screw's thread
gb_boss_l = 4.0
m3_head_d = 6.0     # mm — M3 countersunk head, 90 deg
wall_t    = hub_recess + fl_t + fl_relief + brg_w   # 5.6: flush face, bearing in

# ── The model's coupling: trapezoid socket, torque only ────────────────────
# A square with one side shorter (user, 2026-11-13; a D first, but a D puts the
# torque on one small flat): four faces share the torque, one position only,
# the firmware parks the output with the short side up.
# No click (user, 2026-11-14): the coupling only turns the part; holding it on
# the output is the Coremata's job — its own grip on the brick reaches round
# the turning part (a wheel held from both sides, not overhung).
key_a     = 4.6     # mm — the long side (down when parked)
key_b     = 3.2     # mm — the short side (up)
key_h     = 4.6     # mm — between them
key_depth = 2.8     # mm — socket depth from the hub's face
key_grow  = 0.15    # mm — socket over nominal (offset x 2)
key_play  = 0.15    # mm — the model's spigot under nominal

# ── Gears ───────────────────────────────────────────────────────────────────
g_mod   = 0.8       # mm — module (prints with a 0.4 nozzle)
g_z     = 18        # output and sensor gear teeth
g_alpha = 20.0      # deg — pressure angle
g_gap   = 0.2       # mm — axial gap between the stacked layers
g_w     = 3.7       # mm — gear face width (the slab)

# ── Double swing gear (shared; the pinion and its place are the wall's) ────
z_sw      = 12      # the two swing gears
tip_free  = 1.0     # mm — swing gear tips clear of the output's when parked
mesh_play = 0.1     # mm — centre distance over nominal at the stop (backlash)
arm_bore  = 0.30    # mm — arm's bore over the pinion's hub (sliding, a bit free)
arm_t     = 1.6     # mm — arm plate, behind the gear slab
arm_hub_r = 5.5     # mm — arm round the hub (grows with a bigger hub)
arm_lobe_r = 3.0    # mm — arm round each swing gear's pin
arm_web   = 5.0     # mm — the arm's webs to the pins
sw_pin_d  = 3.0     # mm — the swing gears' pins, printed with the arm
sw_bore   = 0.30    # mm — swing gear bore over its pin
stop_d    = 4.0     # mm — stop post diameter
slot_pin_r = 1.25   # mm — a mount's stop pin (slot stops), in an arc slot 0.15 wider
plate_t   = 1.5     # mm — the motor mount's plate, between the motor and the arm
sleeve_t  = 1.2     # mm — mount walls round the motor
sleeve_fit = 0.10   # mm — mount over the motor's flat faces (barely shrink)

# ── Sensor: gear 1:1 with the output, magnet on its back, AS5600 behind ────
sn_pin_d  = 4.0     # mm — the wall's pin it turns on
sn_bore   = 0.30
sn_boss_r = 4.0     # mm — boss behind the teeth that holds the magnet
sn_boss_l = 2.0
mag_d, mag_t = 4.00, 2.00   # magnet, measured 2026-11-12 (+0.04), diametral
mag_grow  = 0.30
mag_floor = 0.6
as_w, as_h = 22.90, 23.45   # AS5600 AYWHP board, measured 2026-11-12 (+0.04)
as_pcb    = 1.66
as_chip   = 3.25 - 1.66
as_hole_in = 3.50
as_hole_d = 3.60
as_air    = 0.8     # mm — magnet face to chip top (AS5600 wants 0.5-3)
as_post_d = 6.0     # mm — two posts at the holes farthest from the output
# Its pins (measured 2026-11-12): two rows parallel to the 22.90 side, 4.43 in
# from each of those edges, centred along them (assumed): 4 pins (VCC, OUT,
# GND, DIR) and 3 (SCL, SDA, GPO). Both rows are needed (VCC, GND / SDA, SCL;
# DIR bridged to GND). Wires soldered from behind: the board needs this much
# free behind each row — solder, the bend, a sleeve (2026-11-14; the v3 room
# was 4.1).
as_row_in = 4.43
as_pitch  = 2.54
as_pin_pad = 1.2    # mm — round a pin: solder, a wire's end
as_wire_back = 4.0  # mm — room behind each row
as_scr_d, as_scr_h = 5.5, 1.8     # mm — the board's M3 screws, heads behind it
post_in_d = 5.0     # mm — the posts' thin end, through the keeper, to the board
post_big_d = 10.0   # mm — the posts, wall side of the keeper
keeper_tube = 8.0
keeper_slide = 0.3
keeper_short = 0.1
keeper_r  = 8.0
keeper_hole = 0.3

# ── Inside the cube (ghosts, to check the room) ────────────────────────────
battery = (52.0, 33.55, 10.24)   # CITYORK 103450, measured
BOARD_TRY = (44.0, 40.0, 36.0, 32.0, 28.0, 24.0)   # mm — square placeholder for the
board_h   = 8.0                  #      brain's boards: the biggest that fits

if MATERIAL == "PETG":
    m2_pilot = 1.7
    ws_clear = 2.4
elif MATERIAL != "PLA":
    raise ValueError(f"MATERIAL must be PLA or PETG, not {MATERIAL!r}")

# ═══════════════════════════════════════════════════════════════════
# DERIVED — the stack every wall shares (north wall, +Y outward)
# ═══════════════════════════════════════════════════════════════════

r_pitch = g_mod * g_z / 2.0
r_tip   = r_pitch + g_mod
a_mesh  = 2 * r_pitch                   # output - sensor gear
r_sw_p  = g_mod * z_sw / 2.0
a_so    = r_sw_p + r_pitch              # swing gear - output, nominal

y_wo    = cube_side / 2.0               # wall outer face
y_wi    = y_wo - wall_t                 # wall inner face
half_in = y_wo - deck_t                 # cube centre to a deck's inner face
y_g1    = y_wi - g_gap                  # gear slab
y_g0    = y_g1 - g_w
y_a1    = y_g0 - g_gap                  # arm
y_a0    = y_a1 - arm_t
y_p1    = y_a0 - g_gap                  # motor mount plate (the posts end here)
y_m     = y_p1 - plate_t                # the motor's face (gearbox face / tower top)

y_fl1   = y_wo - hub_recess             # hub flange, outer face
y_fl0   = y_fl1 - fl_t
y_b1    = y_fl0 - fl_relief             # bearing, outer face = counterbore floor
y_b0    = y_b1 - brg_w                  # = wall inner face
assert abs(y_b0 - y_wi) < 1e-9
y_key0  = y_fl1 - key_depth             # socket floor
y_bt    = y_g0 - gb_boss_l              # gear boss's inner end: the M3's head
for m3_screw_l in (8.0, 10.0, 12.0, 14.0, 16.0, 20.0):   # shortest that bites 4
    if (y_bt + m3_screw_l) - (y_b0 - hex_len) >= 4.0:
        break
y_scr_end = y_bt + m3_screw_l

y_sb    = y_g0 - sn_boss_l              # sensor: magnet's face
y_chip  = y_sb - as_air
y_pcbf  = y_chip - as_chip
y_pcbb  = y_pcbf - as_pcb
y_kf    = y_g0 - g_gap                  # keeper
y_kb    = y_sb

col_in  = y_wo - col_w
lim_x   = col_in - plate_gap            # wall plate half width
WS_LOW  = (WALL_SCREW_AT[0] * grid, WALL_SCREW_AT[1] * grid)
WALL_SCREWS = [WS_LOW, (-WS_LOW[0], -WS_LOW[1])]

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


def pol(c, r, a):
    return (c[0] + r * math.cos(a), c[1] + r * math.sin(a))


def _inv(a):
    return math.tan(a) - a


def gear_outline(z, m, phase):
    """Involute spur gear outline, (u, w) points, closed; `phase` is the angle
    of tooth 0's centre line."""
    alpha = math.radians(g_alpha)
    r = m * z / 2.0
    ra, rf, rb = r + m, r - 1.25 * m, r * math.cos(alpha)
    psi_p = math.pi / (2 * z) + _inv(alpha)
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
        for rho, psi in flank:
            pts.append((rho * math.cos(th - psi), rho * math.sin(th - psi)))
        psi_tip = flank[-1][1]
        for j in range(1, 4):
            t = th - psi_tip + 2 * psi_tip * j / 4
            pts.append((ra * math.cos(t), ra * math.sin(t)))
        for rho, psi in reversed(flank):
            pts.append((rho * math.cos(th + psi), rho * math.sin(th + psi)))
        psi_root = flank[0][1]
        t0, t1 = th + psi_root, th + 2 * math.pi / z - psi_root
        for j in range(1, 4):
            t = t0 + (t1 - t0) * j / 4
            pts.append((rf * math.cos(t), rf * math.sin(t)))
    return pts


def mesh_phase(phase_a, za, a_xz, b_xz, zb):
    """Phase for gear B so it meshes with A (external mesh)."""
    th = math.atan2(b_xz[1] - a_xz[1], b_xz[0] - a_xz[0])
    pitch_a = 2 * math.pi / za
    e = (phase_a - th) % pitch_a
    if e > pitch_a / 2:
        e -= pitch_a
    return th + math.pi - e * za / zb + math.pi / zb


def make_gear(cx, cz, phase, y0, y1, bore=None, z=g_z):
    pts = [v(cx + u, y0, cz + w) for u, w in gear_outline(z, g_mod, phase)]
    g = Part.Face(Part.makePolygon(pts + [pts[0]])).extrude(v(0, y1 - y0, 0))
    return g.cut(bore) if bore is not None else g


def rot_xz(shape, c, deg):
    """A copy turned `deg` about the Y axis through (c, *), positive the way
    atan2(z, x) grows (the arm's + sense)."""
    s = shape.copy()
    s.rotate(v(c[0], 0, c[1]), Y_AXIS, -deg)
    return s


def bar_xz(p, q, w, y0, y1):
    """A straight web `w` wide from p to q in the wall plane, y0..y1."""
    L = math.hypot(q[0] - p[0], q[1] - p[1])
    b = box(0, L, y0, y1, -w / 2.0, w / 2.0)
    b.rotate(ORIGIN, Y_AXIS, -math.degrees(math.atan2(q[1] - p[1], q[0] - p[0])))
    b.translate(v(p[0], 0, p[1]))
    return b


def hex_y(af, y0, y1):
    """Hexagonal prism along Y on the wall centre, `af` across flats."""
    r = af / math.sqrt(3.0)
    pts = [v(r * math.cos(math.radians(60 * k)), y0, r * math.sin(math.radians(60 * k)))
           for k in range(6)]
    return Part.Face(Part.makePolygon(pts + [pts[0]])).extrude(v(0, y1 - y0, 0))


def d_shaft(cx, cz, y0, y1, grow=0.0, d=3.0, flat=2.5):
    """A D shaft along Y (or, grown, its hole), flat toward +x."""
    r = d / 2.0 + grow / 2.0
    f = flat - d / 2.0 + grow / 2.0
    return cyl_y(cx, cz, r, y0, y1).cut(
        box(cx + f, cx + r + 1, y0 - 1, y1 + 1, cz - r - 1, cz + r + 1))


def place(shape, rot):
    """A copy rotated onto its side about Z (rotations only, never a mirror)."""
    s = shape.copy()
    if rot:
        s.rotate(ORIGIN, Z_AXIS, rot)
    return s


# ═══════════════════════════════════════════════════════════════════
# LAYOUT — where a wall's motor, swing gears, stops and sensor sit
# ═══════════════════════════════════════════════════════════════════

class Layout:
    """Everything that follows from where the motor's shaft S sits round the
    output: the swing gears' places, the arm's travel and stops, the sensor
    gear and its board, the gear phases."""

    def __init__(self, S, z_pin, pin_hub_d, sg_dir, stop_r, stop_ang,
                 mount_posts=None, shaft_tip=None, hub_r=None, slot_r=None):
        self.S = S
        self.z_pin, self.pin_hub_d = z_pin, pin_hub_d
        self.arm_hub_r = hub_r if hub_r else max(arm_hub_r, pin_hub_d / 2.0 + 1.0)
        self.d = math.hypot(*S)
        self.a_ps = g_mod * z_pin / 2.0 + r_sw_p
        self.th_so = math.atan2(-S[1], -S[0])          # direction S -> output

        def ang(dist):
            return math.acos((self.a_ps ** 2 + self.d ** 2 - dist ** 2)
                             / (2 * self.a_ps * self.d))
        self.psi_rest = ang(a_so + 2 * g_mod + tip_free)
        self.psi_touch = ang(a_so + 2 * g_mod)
        self.psi_mesh = ang(a_so + mesh_play)
        self.arm_T = math.degrees(self.psi_rest - self.psi_mesh)
        self.free_T = math.degrees(self.psi_rest - self.psi_touch)
        self.PA = pol(S, self.a_ps, self.th_so - self.psi_rest)   # in when the arm turns +
        self.PB = pol(S, self.a_ps, self.th_so + self.psi_rest)   # in when it turns -
        self.tail = self.th_so + math.pi
        self.stop_r, self.stop_ang = stop_r, stop_ang
        self.slot_r = slot_r
        if slot_r is None:
            # STOP POSTS on the wall either side of a tab on the arm's tail. The
            # tab meets a post when the arm has turned arm_T: post centre to the
            # tab's centreline = half the tab + the post's radius.
            self.POSTS = [pol(S, stop_r, self.tail + s * math.radians(stop_ang)) for s in (1, -1)]
            self.tab_w = 2 * (stop_r * math.sin(math.radians(stop_ang - self.arm_T)) - stop_d / 2.0)
            self.tab_r1 = stop_r * math.cos(math.radians(stop_ang - self.arm_T)) + stop_d / 2.0 + 0.8
            self.stop_part = "Wall"
        else:
            # A PIN on the motor's mount in an arc SLOT of the arm, on the
            # output's side of the shaft, between the two swing gears' webs: the
            # slot's ends meet the pin when the arm has turned arm_T. For walls
            # whose stop posts would stand where the battery goes.
            self.POSTS = []
            self.slot_pin = pol(S, slot_r, self.th_so)
            self.slot_half = math.radians(self.arm_T) + math.asin(slot_pin_r / slot_r)
            self.tab_w = 2 * slot_pin_r       # (reported only)
            self.stop_part = "MotorMount"
        self.MPOSTS = self.POSTS if mount_posts is None else mount_posts   # carry the mount
        self.shaft_tip = shaft_tip            # y of a motor shaft's tip past the pinion
        self.sg_dir = sg_dir
        self.Q = (a_mesh * math.cos(math.radians(sg_dir)), a_mesh * math.sin(math.radians(sg_dir)))
        holes = [(self.Q[0] + sx * (as_w / 2.0 - as_hole_in), self.Q[1] + sz * (as_h / 2.0 - as_hole_in))
                 for sz in (-1, 1) for sx in (-1, 1)]
        holes.sort(key=lambda p: -math.hypot(p[0], p[1]))   # posts at the two farthest
        self.AS_HOLES = holes
        self.PH_P = 0.0
        self.PH_A = mesh_phase(self.PH_P, z_pin, S, self.PA, z_sw)
        self.PH_B = mesh_phase(self.PH_P, z_pin, S, self.PB, z_sw)
        self.PH_O = 0.0
        self.PH_Q = mesh_phase(self.PH_O, g_z, (0.0, 0.0), self.Q, g_z)
        assert self.slot_r or self.tab_w > 3.0, f"arm tab only {self.tab_w:.2f} wide: move the stop posts"


# ═══════════════════════════════════════════════════════════════════
# PARTS EVERY WALL HAS
# ═══════════════════════════════════════════════════════════════════

def make_wall(L):
    """The wall plate: flat face, the bearing's seat, the sensor gear's pin,
    the AS5600's posts, the arm's stop posts and the mount's posts. One part,
    printed outer face down."""
    Q = L.Q
    w = box(-lim_x, lim_x, y_wi, y_wo, -lim_x, lim_x)
    posts = [cyl_y(Q[0], Q[1], sn_pin_d / 2.0, y_sb + mag_t + mag_floor + 0.3, y_wi + 0.01)]
    for hx, hz in L.AS_HOLES[:2]:
        posts.append(cyl_y(hx, hz, as_post_d / 2.0, y_kf, y_wi + 0.01))
        posts.append(cyl_y(hx, hz, post_big_d / 2.0, y_kf, y_wi + 0.01).cut(
            cyl_y(Q[0], Q[1], r_tip + clr, y_kf - 1, y_wi + 1)))
        posts.append(cyl_y(hx, hz, post_in_d / 2.0, y_pcbf, y_kf + 0.01))
    for P in L.POSTS + [p for p in L.MPOSTS if p not in L.POSTS]:
        posts.append(cyl_y(P[0], P[1], stop_d / 2.0, y_p1, y_wi + 0.01))
    w = w.fuse(posts).removeSplitter()
    for sx in (1, -1):
        for top in (True, False):
            w = w.cut(_col_foot(sx, 1, top, foot_clear))
    w = w.cut(cyl_y(0, 0, brg_seat / 2.0, y_wi - 1, y_b1))           # bearing, from inside
    w = w.cut(cyl_y(0, 0, fl_r + cb_clear, y_b1 - 0.01, y_wo + 1))  # flange's counterbore
    if L.shaft_tip is not None:                                      # a shaft past the pinion
        w = w.cut(cyl_y(L.S[0], L.S[1], 1.9, y_wi - 1, L.shaft_tip + 0.4))
    for hx, hz in L.AS_HOLES[:2]:
        w = w.cut(cyl_y(hx, hz, m3_pilot / 2.0, y_pcbf - 1, y_pcbf + 8.0))
    for P in L.MPOSTS:
        w = w.cut(cyl_y(P[0], P[1], m2_pilot / 2.0, y_p1 - 1, y_p1 + 4.5))
    for cx, cz in WALL_SCREWS:
        w = w.cut(cyl_y(cx, cz, ws_clear / 2.0, y_wi - 1, y_wo + 1))
        h = ws_head_d / 2.0 + 0.1
        w = w.cut(Part.makeCone(h, 0.0, h, v(cx, y_wo + 0.001, cz), v(0, -1, 0)))
    return w


def make_arm(L):
    """The swing arm: a plate behind the gear slab, on the pinion's hub, with
    the swing gears' pins and a tab between the two stop posts."""
    S = L.S

    def disk(c, r):
        return cyl_y(c[0], c[1], r, y_a0, y_a1)
    a = disk(S, L.arm_hub_r)
    parts = [disk(L.PA, arm_lobe_r), disk(L.PB, arm_lobe_r),
             bar_xz(S, L.PA, arm_web, y_a0, y_a1), bar_xz(S, L.PB, arm_web, y_a0, y_a1)]
    if L.slot_r is None:
        parts.append(bar_xz(S, pol(S, L.tab_r1, L.tail), L.tab_w, y_a0, y_a1))
    else:
        w = L.slot_half + 0.25
        parts.append(_sector(S, 0.0, L.slot_r + slot_pin_r + 1.8, L.th_so - w, L.th_so + w, y_a0, y_a1))
    parts += [cyl_y(P[0], P[1], sw_pin_d / 2.0, y_a1 - 0.01, y_g1) for P in (L.PA, L.PB)]
    a = a.fuse(parts).removeSplitter()
    if L.slot_r is not None:
        a = a.cut(_sector(S, L.slot_r - slot_pin_r - 0.15, L.slot_r + slot_pin_r + 0.15,
                          L.th_so - L.slot_half, L.th_so + L.slot_half, y_a0 - 1, y_a1 + 1))
    return a.cut(cyl_y(S[0], S[1], (L.pin_hub_d + arm_bore) / 2.0, y_a0 - 1, y_a1 + 1))


def _sector(c, r0, r1, a0, a1, y0, y1):
    """An annular sector round c in the wall plane, angles a0..a1 (radians)."""
    n = 24
    pts = [pol(c, r1, a0 + (a1 - a0) * i / n) for i in range(n + 1)]
    pts += [pol(c, r0, a1 - (a1 - a0) * i / n) for i in range(n + 1)] if r0 > 0 else [c]
    vs = [v(x, y0, z) for x, z in pts]
    return Part.Face(Part.makePolygon(vs + [vs[0]])).extrude(v(0, y1 - y0, 0))


def make_swing_gear(L, P, phase):
    return make_gear(P[0], P[1], phase, y_g0, y_g1,
                     cyl_y(P[0], P[1], (sw_pin_d + sw_bore) / 2.0, y_g0 - 1, y_g1 + 1), z_sw)


def make_output_gear(L):
    """Output gear: hex socket for the hub, a boss behind for the M3 (head in
    its countersink), a 0.2 ring that reaches the bearing's inner ring."""
    g = make_gear(0, 0, L.PH_O, y_g0, y_g1)
    g = g.fuse([cyl_y(0, 0, gb_boss_r, y_bt, y_g0 + 0.01),
                cyl_y(0, 0, brg_in_r, y_g1 - 0.01, y_b0)]).removeSplitter()
    g = g.cut(hex_y(hex_hole, y_b0 - hex_len - 0.2, y_b0 + 1))
    g = g.cut(cyl_y(0, 0, m3_clear / 2.0, y_bt - 1, y_b0))
    return g.cut(Part.makeCone(m3_head_d / 2.0 + 0.1, 0.0, m3_head_d / 2.0 + 0.1,
                               v(0, y_bt - 0.001, 0), Y_AXIS))


def _trap_pts():
    """The trapezoid's corners in (x, z), shifted so its four corners are the
    same distance from the axis."""
    c = (key_a ** 2 - key_b ** 2) / (8.0 * key_h)
    return [(-key_a / 2.0, -key_h / 2.0 + c), (key_a / 2.0, -key_h / 2.0 + c),
            (key_b / 2.0, key_h / 2.0 + c), (-key_b / 2.0, key_h / 2.0 + c)]


key_r = max(math.hypot(x, z) for x, z in _trap_pts())


def trap_key(off, y0, y1):
    """The trapezoid along Y on the wall centre, sides moved out by `off`."""
    pts = [v(x, y0, z) for x, z in _trap_pts()]
    f = Part.Face(Part.makePolygon(pts + [pts[0]]))
    if off:
        f = f.makeOffset2D(off, 2)
    return f.extrude(v(0, y1 - y0, 0))


def make_output_hub():
    """Hub: the flange (the model's face), a ring on the inner ring only, the
    journal through the bearing, a short hex into the gear, the M3's pilot,
    the trapezoid socket. Printed flange down."""
    h = cyl_y(0, 0, fl_r, y_fl0, y_fl1).fuse([
        cyl_y(0, 0, brg_in_r, y_b1, y_fl0 + 0.01),
        cyl_y(0, 0, journal_d / 2.0, y_b0 - 0.01, y_b1 + 0.01),
        hex_y(hex_af, y_b0 - hex_len, y_b0 + 0.01)]).removeSplitter()
    h = h.cut(cyl_y(0, 0, m3_pilot / 2.0, y_b0 - hex_len - 1, y_key0 - 0.4))
    return h.cut(trap_key(key_grow / 2.0, y_key0, y_fl1 + 1))


def make_model_key():
    """The model's side, a template part: disc in the counterbore, a plate 0.2
    clear of the face with a 24 mm arm, the trapezoid spigot."""
    k = cyl_y(0, 0, fl_r - 0.2, y_fl1, y_wo + 0.2).fuse([
        cyl_y(0, 0, 8.0, y_wo + 0.2, y_wo + 2.2),
        box(-3.0, 3.0, y_wo + 0.2, y_wo + 2.2, 0.0, 24.0)])
    sp = trap_key(-key_play / 2.0, y_key0 + 0.2, y_fl1 + 0.01)
    return k.fuse(sp).removeSplitter()


def make_bearing():
    return cyl_y(0, 0, brg_od / 2.0, y_b0, y_b1).cut(cyl_y(0, 0, brg_id / 2.0, y_b0 - 1, y_b1 + 1))


def make_screw():
    head = Part.makeCone(m3_head_d / 2.0, 0.0, m3_head_d / 2.0, v(0, y_bt, 0), Y_AXIS)
    return head.fuse(cyl_y(0, 0, 1.4, y_bt + 0.5, y_scr_end))


def make_sensor_gear(L):
    Q = L.Q
    g = make_gear(Q[0], Q[1], L.PH_Q, y_g0, y_g1)
    g = g.fuse(cyl_y(Q[0], Q[1], sn_boss_r, y_sb, y_g0 + 0.01)).removeSplitter()
    g = g.cut(cyl_y(Q[0], Q[1], (sn_pin_d + sn_bore) / 2.0, y_sb + mag_t + mag_floor, y_g1 + 1))
    return g.cut(cyl_y(Q[0], Q[1], (mag_d + mag_grow) / 2.0, y_sb - 1, y_sb + mag_t))


def make_keeper(L):
    Q = L.Q
    k = cyl_y(Q[0], Q[1], keeper_r, y_kb, y_kf)
    for hx, hz in L.AS_HOLES[:2]:
        k = k.fuse(cyl_y(hx, hz, keeper_tube / 2.0, y_pcbf + keeper_short, y_kf))
        k = k.fuse(bar_xz(Q, (hx, hz), as_post_d, y_kb, y_kf))
    k = k.removeSplitter()
    k = k.cut(cyl_y(Q[0], Q[1], sn_boss_r + keeper_hole, y_kb - 1, y_kf + 1))
    for hx, hz in L.AS_HOLES[:2]:
        k = k.cut(cyl_y(hx, hz, (post_in_d + keeper_slide) / 2.0, y_pcbf - 1, y_kf + 1))
    return k


def make_magnet(L):
    return cyl_y(L.Q[0], L.Q[1], mag_d / 2.0, y_sb, y_sb + mag_t)


def make_as5600(L):
    Q = L.Q
    b = box(Q[0] - as_w / 2.0, Q[0] + as_w / 2.0, y_pcbb, y_pcbf, Q[1] - as_h / 2.0, Q[1] + as_h / 2.0)
    for hx, hz in L.AS_HOLES:
        b = b.cut(cyl_y(hx, hz, as_hole_d / 2.0, y_pcbb - 1, y_pcbf + 1))
    b = b.fuse(box(Q[0] - 2.5, Q[0] + 2.5, y_pcbf - 0.01, y_chip, Q[1] - 2.0, Q[1] + 2.0))
    return b.fuse(as5600_room(L)).removeSplitter()


def as5600_room(L):
    """What the board needs round it beyond its own shape: room behind both pin
    rows for the wires, the screws' heads behind it."""
    Q = L.Q
    p = as_pin_pad
    room = []
    for zr, n in ((Q[1] + as_h / 2.0 - as_row_in, 4), (Q[1] - as_h / 2.0 + as_row_in, 4)):
        w = (n - 1) / 2.0 * as_pitch + p
        room.append(box(Q[0] - w, Q[0] + w, y_pcbb - as_wire_back, y_pcbb + 0.01, zr - p, zr + p))
    for hx, hz in L.AS_HOLES[:2]:
        room.append(cyl_y(hx, hz, as_scr_d / 2.0, y_pcbb - as_scr_h, y_pcbb + 0.01))
    return room[0].fuse(room[1:])


def common_parts(L):
    """The parts every wall has, as (name, shape, colour, transparency)."""
    return [("Wall", make_wall(L), (0.95, 0.75, 0.30), 0),
            ("Arm", make_arm(L), (0.55, 0.40, 0.15), 0),
            ("SwingGearA", make_swing_gear(L, L.PA, L.PH_A), (0.90, 0.80, 0.55), 0),
            ("SwingGearB", make_swing_gear(L, L.PB, L.PH_B), (0.90, 0.80, 0.55), 0),
            ("OutputGear", make_output_gear(L), (0.90, 0.90, 0.90), 0),
            ("OutputHub", make_output_hub(), (0.85, 0.45, 0.20), 0),
            ("Bearing", make_bearing(), (0.70, 0.70, 0.75), 0),
            ("Screw", make_screw(), (0.30, 0.30, 0.30), 0),
            ("SensorGear", make_sensor_gear(L), (0.80, 0.90, 0.80), 0),
            ("Magnet", make_magnet(L), (0.60, 0.60, 0.65), 0),
            ("AS5600", make_as5600(L), (0.15, 0.50, 0.25), 0),
            ("Keeper", make_keeper(L), (0.95, 0.60, 0.20), 0),
            ("ModelKey", make_model_key(), (0.30, 0.70, 0.90), 40)]


COMMON_PRINTED = ("Wall", "Arm", "OutputGear", "OutputHub", "SensorGear", "Keeper", "ModelKey")
SWING = ("Arm", "SwingGearA", "SwingGearB")
ALLOWED = {("SensorGear", "OutputGear"), ("SwingGear", "Pinion"), ("SwingGear", "Arm"),
           ("Arm", "Pinion"), ("MotorMount", "Wall"), ("Keeper", "Wall"), ("AS5600", "Wall"),
           ("OutputHub", "OutputGear"), ("Bearing", "Wall"), ("Bearing", "OutputHub"),
           ("Bearing", "OutputGear"), ("Screw", "OutputHub"), ("Screw", "OutputGear"),
           ("ModelKey", "OutputHub")}


# ═══════════════════════════════════════════════════════════════════
# FRAME
# ═══════════════════════════════════════════════════════════════════

def _col_foot(sx, sy, top=True, grow=0.0):
    """A column's widened end (its free end takes the other deck's M3); the
    walls' corner notches are this shape grown by foot_clear."""
    W, a0 = col_foot_w + grow, col_w + grow
    c = W - a0
    k = 1 if top else -1
    z_end, z_in = k * (half_in + grow), k * (half_in - col_foot_h - grow)
    sq = box(*sorted((sx * (y_wo - W), sx * y_wo)), *sorted((sy * (y_wo - W), sy * y_wo)),
             *sorted((z_in, z_end)))

    def ring(a, z):
        x_in, y_in = sx * (y_wo - a), sy * (y_wo - a)
        x_out, y_out = sx * y_wo, sy * y_wo
        pts = [v(x_in, y_in, z), v(x_out, y_in, z), v(x_out, y_out, z), v(x_in, y_out, z)]
        return Part.makePolygon(pts + [pts[0]])
    return sq.fuse(Part.makeLoft([ring(a0, z_in - k * c), ring(W, z_in + k * 0.01)], True))


def make_deck():
    """One deck — top and bottom are the SAME part, the top one turned over:
    plate, skirt framing the walls, a wall-screw boss behind each wall, two
    corner columns on one diagonal, two countersunk M3 holes on the other."""
    d = box(-y_wo, y_wo, -y_wo, y_wo, -y_wo, -half_in)
    cxy = y_wo - screw_in
    for sx, sy in ((1, 1), (-1, -1)):
        d = d.fuse(box(*sorted((sx * col_in, sx * y_wo)), *sorted((sy * col_in, sy * y_wo)),
                       -half_in - 0.01, half_in))
        d = d.fuse([_col_foot(sx, sy, top) for top in (True, False)])
        d = d.cut(cyl_z(sx * cxy, sy * cxy, m3_pilot / 2.0, half_in - 8.0, half_in + 1))
    bx, bz = WS_LOW
    for rot in (0, 90, 180, 270):
        sk = box(-col_in, col_in, y_wi - skirt_in, y_wo, -half_in - 0.01, -col_in)
        bs = box(bx - ws_boss_w / 2.0, bx + ws_boss_w / 2.0, y_wi - ws_boss_d, y_wi,
                 -half_in - 0.01, bz + ws_boss_w / 2.0)
        pilot = cyl_y(bx, bz, m2_pilot / 2.0, y_wi - ws_boss_d - 1, y_wi + 0.01)
        lp = [box(-col_in, col_in, y_wi - lip_t, y_wi, -col_in - 0.01, -(lim_x - lip_w))]
        for sx in (1, -1):
            corner = App.Rotation(Z_AXIS, rot).multVec(v(sx * col_in, y_wo, 0))
            if corner.x * corner.y > 0:
                x_in, x_col = sx * (lim_x - lip_w), sx * (col_in + 0.3)
                lp.append(box(*sorted((x_in, x_col)), y_wi - lip_t, y_wi,
                              -(lim_x - lip_w), -lip_gap_z))
                pts = [v(x_col, y_wi - lip_t, lip_gap_z - (col_in + 0.3 - lim_x + lip_w)),
                       v(x_in, y_wi - lip_t, lip_gap_z), v(x_in, y_wi - lip_t, lim_x - lip_w),
                       v(x_col, y_wi - lip_t, lim_x - lip_w)]
                lp.append(Part.Face(Part.makePolygon(pts + [pts[0]])).extrude(v(0, lip_t, 0)))
        for part in [sk, bs, pilot] + lp:
            part.rotate(ORIGIN, Z_AXIS, rot)
        d = d.fuse([sk, bs] + lp).cut(pilot)
    for sx, sy in ((1, -1), (-1, 1)):
        room = _col_foot(sx, sy, False, foot_clear).common(
            box(-y_wo - 1, y_wo + 1, -y_wo - 1, y_wo + 1, -half_in, 0))
        d = d.cut(room)
    d = d.cut(box(*cable_hole, -y_wo - 1, -half_in + 1))
    for sx, sy in ((1, -1), (-1, 1)):
        x, y = sx * cxy, sy * cxy
        d = d.cut(cyl_z(x, y, m3_clear / 2.0, -y_wo - 1, -half_in + 1))
        d = d.cut(Part.makeCone(3.2, m3_clear / 2.0, 1.6, v(x, y, -y_wo - 0.001), Z_AXIS))
    return d.removeSplitter()


def deck_top(deck):
    s = deck.copy()
    s.rotate(ORIGIN, X_AXIS, 180)
    return s


# ═══════════════════════════════════════════════════════════════════
# A WALL TYPE, AND A CUBE MADE OF WALLS
# ═══════════════════════════════════════════════════════════════════

class WallType:
    """What a wall file hands over: its layout, its parts (north wall), the
    pairs of its own parts that are meant to touch, its motor's parts (for the
    neighbour report) and its printable parts."""

    def __init__(self, name, L, motor_parts, allowed=(), motor_names=(), printed=(),
                 notes=(), buy=()):
        self.name, self.L = name, L
        self.parts = common_parts(L) + list(motor_parts)
        self.allowed = set(ALLOWED) | set(allowed)
        self.motor_names = tuple(motor_names)
        self.printed = tuple(COMMON_PRINTED) + tuple(printed)
        self.notes, self.buy = tuple(notes), tuple(buy)


SIDES = (("N", 0), ("W", 90), ("S", 180), ("E", 270))
SIDE_LABEL = {"N": "Pared N", "W": "Pared O", "S": "Pared S", "E": "Pared E"}


def _allowed(walls, a, b):
    pa, wa = a.rsplit("_", 1) if "_" in a else (a, "")
    pb, wb = b.rsplit("_", 1) if "_" in b else (b, "")
    if wa != wb or wa not in walls:
        return False
    for x, y in walls[wa].allowed:
        if (pa.startswith(x) and pb.startswith(y)) or (pa.startswith(y) and pb.startswith(x)):
            return True
    return False


def _hits(shape, others, by):
    out = []
    for o in others:
        if shape.BoundBox.intersect(by[o].BoundBox):
            vol = shape.common(by[o]).Volume
            if vol > 1e-3:
                out.append((o, vol))
    return out


def assemble(walls):
    """walls: {"N": WallType, ...}. Returns the parts list (name, shape, colour,
    transparency), the deck, the battery's height and the board's size."""
    parts = []
    for side, rot in SIDES:
        for n, sh, col, tr in walls[side].parts:
            parts.append((f"{n}_{side}", place(sh, rot), col, tr))
    deck = make_deck()
    parts.append(("DeckBottom", deck, (0.45, 0.55, 0.75), 60))
    parts.append(("DeckTop", deck_top(deck), (0.45, 0.55, 0.75), 60))
    # The battery lies on whatever reaches highest under it — the arms and
    # swing gears at both their stops too.
    foot = box(-battery[0] / 2, battery[0] / 2, -battery[1] / 2, battery[1] / 2, -half_in, half_in)
    swept = []
    for side, rot in SIDES:
        W = walls[side]
        north = dict((n, sh) for n, sh, c, t in W.parts)
        for sgn in (1, -1):
            swept += [place(rot_xz(north[p], W.L.S, sgn * W.L.arm_T), rot) for p in SWING]
    under = [sh.common(foot) for sh in [p[1] for p in parts
                                         if not p[0].startswith(("Deck", "ModelKey"))] + swept
             if sh.BoundBox.intersect(foot.BoundBox)]
    z0 = clr + max(s.BoundBox.ZMax for s in under if s.Volume > 1e-3)
    parts.append(("Battery", box(-battery[0] / 2, battery[0] / 2, -battery[1] / 2, battery[1] / 2,
                                 z0, z0 + battery[2]), (0.85, 0.20, 0.20), 40))
    # The boards: the biggest square placeholder that fits on the bottom deck.
    zb = -half_in + clr
    board = None
    for side_mm in BOARD_TRY:
        b = box(-side_mm / 2, side_mm / 2, -side_mm / 2, side_mm / 2, zb, zb + board_h)
        if all(not b.BoundBox.intersect(sh.BoundBox) or b.common(sh).Volume < 1e-3
               for n, sh, c, t in parts if not n.startswith("Deck")):
            near = min((b.distToShape(sh)[0] for n, sh, c, t in parts
                        if not n.startswith("Deck") and sh.BoundBox.ZMin < zb + board_h + 1),
                       default=99)
            if near > 0.3:
                board = side_mm
                break
    if board:
        parts.append(("MainBoard", box(-board / 2, board / 2, -board / 2, board / 2, zb, zb + board_h),
                      (0.15, 0.50, 0.25), 40))
    return parts, deck, z0, board


def show(doc_name, parts, show_only=None):
    """Put the parts in a FreeCAD document named like the file, grouped by side."""
    if App.ActiveDocument and App.ActiveDocument.Name == doc_name:
        App.closeDocument(doc_name)
    for d in list(App.listDocuments().values()):
        if d.Name == doc_name:
            App.closeDocument(doc_name)
    doc = App.newDocument(doc_name)
    groups = {}
    for n, sh, col, tr in parts:
        obj = doc.addObject("Part::Feature", n)
        obj.Shape = sh
        if HAS_GUI and obj.ViewObject is not None:
            obj.ViewObject.ShapeColor = col
            obj.ViewObject.Transparency = tr
        side = n.rsplit("_", 1)[-1] if "_" in n else ""
        label = SIDE_LABEL.get(side, "Marco")
        if n.startswith(("Battery", "MainBoard")):
            label = "Interior (fantasmas)"
        if label not in groups:
            groups[label] = doc.addObject("App::DocumentObjectGroup", "G_" + label.replace(" ", "_"))
            groups[label].Label = label
        groups[label].addObject(obj)
    doc.recompute()
    if HAS_GUI and getattr(App, "GuiUp", 0):
        if show_only:
            for g in groups.values():
                vis = False
                for obj in g.Group:
                    on = obj.Name.endswith(show_only)
                    obj.ViewObject.Visibility = on
                    vis = vis or on
                g.ViewObject.Visibility = vis
                for obj in g.Group:
                    obj.ViewObject.Visibility = obj.Name.endswith(show_only)
        Gui.SendMsgToActiveView("ViewFit")
        Gui.activeDocument().activeView().viewIsometric()
    return doc


def export_stl(here, folder, wall, deck=None):
    """The wall's printable parts (as the north wall; orient in the slicer)."""
    pdir = os.path.join(here, "stl", folder, MATERIAL)
    os.makedirs(pdir, exist_ok=True)
    by = dict((n, sh) for n, sh, c, t in wall.parts)
    for n in wall.printed:
        by[n].exportStl(os.path.join(pdir, f"{folder}_{n}.stl"))
    by["SwingGearA"].exportStl(os.path.join(pdir, f"{folder}_SwingGear_x2.stl"))
    if deck is not None:
        deck.exportStl(os.path.join(pdir, f"{folder}_Deck.stl"))
    return pdir


def check(walls, parts, z_batt, board, slide_sides=None, title=""):
    """Every check, for every side: interference, the arm at both stops, the
    output turning free, a wall sliding out. Prints the report; returns the
    number of failures."""
    checks, bad = [], []
    names = [p[0] for p in parts]
    by = dict((p[0], p[1]) for p in parts)
    for i in range(len(parts)):
        for j in range(i + 1, len(parts)):
            if _allowed(walls, names[i], names[j]):
                continue
            if parts[i][1].BoundBox.intersect(parts[j][1].BoundBox):
                vol = parts[i][1].common(parts[j][1]).Volume
                if vol > 1e-3:
                    bad.append((names[i], names[j], vol))

    def gap(a, b):
        return by[a].distToShape(by[b])[0]

    done = set()
    for side, rot in SIDES:
        W = walls[side]
        L = W.L
        first = W.name not in done            # per-type numbers only once
        done.add(W.name)
        if first:
            for nm, P in (("A", L.PA), ("B", L.PB)):
                tips = math.hypot(*P) - (r_sw_p + g_mod) - r_tip
                checks.append((f"{W.name}: swing gear {nm} parked, tips clear by", tips,
                               f"= {tip_free}", abs(tips - tip_free) < 1e-6))
        # the arm at its two stops, on this side
        north = dict((n, sh) for n, sh, c, t in W.parts)
        stay = [n for n in names if not (n.endswith("_" + side) and n.split("_")[0] in SWING)]
        for sgn, nm in ((1, "A"), (-1, "B")):
            moved = {p: place(rot_xz(north[p], L.S, sgn * L.arm_T), rot) for p in SWING}
            g = moved["Arm"].distToShape(by[f"{L.stop_part}_{side}"])[0]
            ok_tab = g < 0.02
            hit = []
            for p in SWING:
                others = [o for o in stay if o not in (f"Pinion_{side}", f"OutputGear_{side}")]
                hit += [f"{p}/{o}" for o, vol in _hits(moved[p], others, by)]
            checks.append((f"{side} ({W.name}): arm at the {nm} stop, on its stop and clear"
                           f"{': ' + ', '.join(hit) if hit else ''}",
                           g, "tab < 0.02, no hits", ok_tab and not hit))
        for a, b in (("OutputHub", "Wall"), ("OutputGear", "Wall"), ("OutputGear", "Keeper"),
                     ("OutputGear", "Arm"), ("ModelKey", "Wall"), ("SwingGearA", "OutputGear"),
                     ("SwingGearB", "OutputGear")):
            g = gap(f"{a}_{side}", f"{b}_{side}")
            checks.append((f"{side}: {a} / {b} turn free", g, "> 0.15", g > 0.15))
    # neighbours: the closest any motor-side parts of two sides come
    for (s1, r1), (s2, r2) in zip(SIDES, SIDES[1:] + SIDES[:1]):
        cand1 = [f"{n}_{s1}" for n in walls[s1].motor_names + ("AS5600",)]
        cand2 = [f"{n}_{s2}" for n in walls[s2].motor_names + ("AS5600",)]
        g = min(gap(a, b) for a in cand1 for b in cand2)
        checks.append((f"neighbours {s1} ({walls[s1].name}) / {s2} ({walls[s2].name})", g,
                       "> 0.3", g > 0.3))
    g = min(gap("Battery", n) for n in names if not n.startswith(("Battery", "Deck", "ModelKey")))
    checks.append(("battery: clear of everything under it", g, "> 0.3", g > 0.3))
    g = gap("Battery", "DeckTop")
    checks.append(("battery under the top deck", g, "> 0.3", g > 0.3))
    checks.append(("boards' placeholder fits", board or 0, f"one of {BOARD_TRY}", bool(board)))
    checks.append(("M3 from inside: tip under the socket's floor", y_key0 - y_scr_end, ">= 0.4",
                   y_key0 - y_scr_end >= 0.4 - 1e-9))
    checks.append(("M3 thread in the hub", y_scr_end - (y_b0 - hex_len), ">= 4",
                   y_scr_end - (y_b0 - hex_len) >= 4.0 - 1e-9))
    wd = journal_d / 2.0 - (key_r + key_grow / 2.0)
    checks.append(("hub wall round the socket", wd, ">= 1.5", wd >= 1.5 - 1e-9))
    lip = brg_seat / 2.0 - (fl_r + cb_clear)
    checks.append(("bearing stops on the counterbore's floor", lip, ">= 0.5", lip >= 0.5))
    checks.append(("column foot longer than the M3 pilot (8)", col_foot_h, "> 8", col_foot_h > 8))
    # a wall comes out without opening the cube
    out = {"N": (0, 1), "W": (-1, 0), "S": (0, -1), "E": (1, 0)}
    for side in (slide_sides or [s for s, r in SIDES]):
        mine = [n for n in names if n.endswith("_" + side) and not n.startswith("ModelKey")]
        rest = [n for n in names if not n.endswith("_" + side) and not n.startswith("ModelKey")]
        hits = set()
        dx, dy = out[side]
        for m in mine:
            for k in range(1, 40, 2):
                sh = by[m].copy()
                sh.translate(v(dx * k, dy * k, 0))
                for o, vol in _hits(sh, rest, by):
                    hits.add(f"{m}/{o}")
        checks.append((f"{side} ({walls[side].name}) slides out 39 mm"
                       f"{': ' + ', '.join(sorted(hits)) if hits else ''}",
                       len(hits), "0 in the way", not hits))

    print("=" * 76)
    print(title)
    print("-" * 76)
    for label, value, target, ok in checks:
        print(f"  [{'OK ' if ok else 'FAIL'}] {label}:  {value:.3f}  ({target})")
    for a, b, vol in bad:
        print(f"  [FAIL] interference {a} / {b}: {vol:.3f} mm3")
    if not bad:
        print(f"  [OK ] no interference among {len(parts)} parts (designed contacts excepted)")
    print("-" * 76)
    print(f"  CUBE {cube_side:.0f} mm: walls {wall_t:.1f}, decks {deck_t:.1f}; battery z {z_batt:.1f}.."
          f"{z_batt + battery[2]:.1f}; boards' room {board} x {board} x {board_h:.0f} on the bottom deck")
    print(f"  OUTPUT: 6700ZZ, hub flange Ø{2 * fl_r:.1f} flush, M3 x {m3_screw_l:.0f} countersunk"
          f" from inside; trapezoid socket {key_a}/{key_b} x {key_h}, {key_depth} deep")
    for W in {w.name: w for w in walls.values()}.values():
        L = W.L
        print(f"  {W.name}: shaft {L.d:.1f} off the output at {math.degrees(math.atan2(L.S[1], L.S[0])):.0f}"
              f" deg; pinion z{L.z_pin} -> z{z_sw} -> z{g_z} ({g_z / L.z_pin:.2f}:1); arm travel"
              f" {L.arm_T:.1f} deg, parks within +-{L.free_T:.1f}; "
              + (f"stops r {L.stop_r} +-{L.stop_ang:.0f} deg, tab {L.tab_w:.2f}" if L.slot_r is None
                 else f"stop: mount pin in an arc slot r {L.slot_r}")
              + f"; sensor at {L.sg_dir:.0f} deg")
        for n in W.notes:
            print(f"    {n}")
        if W.buy:
            print(f"    buy per wall: {', '.join(W.buy)}")
    print(f"  per wall, all types: 6700ZZ, M3 x {m3_screw_l:.0f} countersunk, 2 x {ws_screw} (wall),"
          f" 2 x M3 x 8 (AS5600), AS5600 + magnet;  per cube: 2 decks, 4 x M3 x 10 countersunk")
    print("=" * 76)
    return sum(1 for c in checks if not c[3]) + len(bad)


def run_cube(walls, doc_name, here=None, stl_wall=None, stl_folder=None,
             slide_sides=None, show_only="_N", run_checks=None, title=""):
    """Assemble, show, export the STLs and (headless, or when asked) check."""
    parts, deck, z0, board = assemble(walls)
    show(doc_name, parts, show_only)
    if here and stl_wall is not None:
        export_stl(here, stl_folder, stl_wall, deck)
    if run_checks is None:
        run_checks = not getattr(App, "GuiUp", 0)
    if run_checks:
        return check(walls, parts, z0, board, slide_sides, title or doc_name)
    print(f"{doc_name}: geometry only, checks skipped (they run headless)")
    return 0


def here_of(g):
    """The folder a macro runs from (FreeCAD's GUI may give no __file__)."""
    f = g.get("__file__")
    return os.path.dirname(os.path.abspath(f)) if f else App.getUserMacroDir(True)
