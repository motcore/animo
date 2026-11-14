# Animo v5 — the 72 mm cube: v4b1's walls on a BALL BEARING, no LEGO
# FreeCAD Python Macro
#
# v5 (2026-11-13) = v4b1 (continuous MG90D + double swing gear) with an output
# of its own:
#   - LEGO is OFF the motor wall (user, 2026-11-13): no Technic cross, no
#     Technic pin holes. The cube stays 72 (the LEGO grid); LEGO comes back
#     later as a wall of its own or an adapter — who has no LEGO pays nothing.
#   - the output turns on a 6700ZZ (10 x 15 x 4, common and cheap): the wall
#     plate grows from 3 to 5.6 to hold it with the face still flat (a 688,
#     5 wide, asked for 6.6 and closed the servos' pinwheel). Decks stay 3. Modules of this wall no longer mix with v4/v4c ones
#     (user: accepted, mixing is a bonus, not a rule).
#   - hub and gear held together by ONE M3 countersunk screw down the axis,
#     not by the pressed hex that came loose: the screw clamps the bearing's
#     inner ring between the hub's flange and the gear's boss, so nothing can
#     walk out; a short hex (no press) carries the torque.
#   - the model's coupling on the hub's flange is NOT drawn yet (next step).
#
# --- v4b1's notes, still true unless v5 says otherwise ---
# v4b1 (2026-11-13) = v4b with the user's MG90D of CONTINUOUS rotation (the
# ones in the drawer, ~8): same body, same horn, so every part is v4b's. What
# changes is how it is driven: PWM sets speed and sense, not angle, so
#   - the output turns WITHOUT LIMIT (1:1, ~100 rpm like the MG90S 360);
#   - the servo no longer knows its angle: parking goes back to v4's way —
#     reverse slowly until the AS5600 sees the output stop following (the
#     swing gear just left), then a few degrees more by time; dry friction
#     leaves the arm there, free both ways within +-4.8 deg;
#   - stopped (PWM at its dead-band centre) the servo's gearbox holds: a swing
#     gear left in holds its way, as on the N20.
#
# --- v4b's notes, still true unless v4b1 says otherwise ---
#
# v4b (2026-11-13) = v4's double swing gear driven by the MG90D (position
# servo) instead of an N20. The servo gear becomes the PINION: z18 with the
# servo's own horn in it, as in v3; the arm turns on the pinion's hub; two z12
# swing gears; output z18, so 1:1 and the output gets the servo's ~180 deg,
# less the arm's travel at each reversal.
# What it adds over v3: the FREE state (the servo parks the arm in the middle:
# it knows its own angle, so parking is just a commanded step back, no
# AS5600 trick), and a servo needs no H-bridge.
# What it does not do: turn without limit (MG90D ~180 deg).
# The servo is held by a SLEEVE over its gear tower and case top, screwed to
# its tabs from below and hung from the two stop posts, like v4's N20 mount.
# The sensor gear moves to +30 deg (the lower swing gear sits where v4 had it)
# and the Technic pins move with it.
#
# --- v4's notes, still true unless v4b says otherwise ---
# v4 (2026-11-13) = v3's frame, output and AS5600, with the MG90S 360 swapped
# for an N20 gearmotor driving the output through a DOUBLE SWING GEAR:
#   - a z10 pinion on the N20's D shaft;
#   - an ARM that turns on the pinion's hub and carries two z12 swing gears,
#     both always in mesh with the pinion, one either side of the line from
#     the pinion to the output;
#   - the pinion's turning drags the arm (the swing gears' own pin friction,
#     dry: no spring, no grease) until the swing gear on that side meshes
#     with the output; its mesh force then pushes the arm into its STOP, and
#     the stop sets the mesh depth;
#   - turning the other way swings the other gear in: output follows the
#     pinion's sense, 1.8:1 reduction, no reversing of the gear train needed;
#   - FREE: the firmware parks the arm in the middle (reverse slowly until the
#     AS5600 sees the output stop following, then a few degrees more by time);
#     dry friction leaves it there and both swing gears stand clear of the
#     output by tip_free: the output turns free both ways with the motor off.
#     Stopped with a gear in and the load pushing back, that gear stays in and
#     holds one way (the N20's gearbox, no current); overrunning frees it.
# The two arm stops are the posts that carry the N20's mount.
# Why no centring spring: whatever angle dry drag carries the arm to against a
# spring, the same drag holds it there when the motor stops (static >= dynamic
# friction), so a spring can never re-centre it (2026-11-13). A viscous drag
# would, but the user rejected grease.
# The AS5600 reads the output directly (sensor gear 1:1, as in v3), so the
# servo's board is gone: the C3 closes the loop through a DRV8833 per two walls.
#
# --- v3's notes, still true unless v4 says otherwise ---
#
# v3 (2026-11-12) = v2's wall module and frame, grown to 72 mm (9 LEGO modules),
# plus an AS5600 per wall, on a SENSOR GEAR 1:1 under the output with the
# magnet on its back; the board hangs behind it on two posts; a KEEPER plate
# 0.2 behind the sensor gear, trapped between the posts' step and the board.
#
# The rules this file is built on (user, 2026-10-14/15):
#   - every output shaft on the CENTRE of its wall: symmetry is fundamental;
#   - the outer face is FLAT and LEGO TECHNIC compatible: a Technic cross in
#     the hub, three Technic pin holes on the 8 mm grid;
#   - all four wall modules are IDENTICAL, only rotated about Z;
#   - a wall comes out WITHOUT OPENING THE CUBE: the module slides straight
#     out; two M2x6 countersunk screws in the face hold it to deck bosses.
#
# Coordinates: origin = cube centre, +Z up. One module is built for the +Y
# (north) wall: +Y outward, X across the wall, Z up. It is then rotated onto
# its wall about Z — rotations only, never a mirror.
#
# Run headless to read the checks:
#   "C:/Program Files/FreeCAD 1.0/bin/freecadcmd.exe" animo_v4_cube.py
# or from FreeCAD: Macro -> Macros -> animo_v4_cube.py -> Execute

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

# ── Material: sets the fits that differ between them, and where the STLs go ─
# The fits below are the PLA ones, tested 2026-10-17. PETG opens the ROUND
# holes 0.1 mm (its coupons, T1.2, printed large holes ~0.1 tighter: 688 16.2
# vs 16.1, 6805 37.3 vs 37.2). Flat faces (hex, grooves) and the M3 thread
# stay. Headless, ANIMO_MATERIAL=PETG picks it without editing the file.
MATERIAL = os.environ.get("ANIMO_MATERIAL", "PETG")    # "PETG" (now) or "PLA"

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
horn_lift    = 1.65   # mm — the hub's underside above the tower. Measured 2026-10-20:
                      #      tower to the horn's top 6.34 (6.30 + 0.04), so 6.34 - hub
                      #      4.69. The cube follows it
horn_grow    = 0.2    # mm — pocket over the horn's ARMS (holes print ~0.2 small)
horn_hub_grow = 0.2   # mm — pocket over its hub: a PRESS fit — it needs a firm
                      #      push and then seats perfectly (tested 2026-10-17)

# ── Double swing gear (v4) ──────────────────────────────────────────────────
z_pin    = 18       # pinion = the servo gear, holding the horn (1:1 to the output)
z_sw     = 12       # the two swing gears
x_sv     = -12.47   # mm — the servo shaft's X, as v3: the servo (and its sleeve)
                    #      stays clear of the AS5600 board; the shaft BELOW the
                    #      output, the case standing up from it
n20_d    = 17.9     # mm — servo shaft to output: the least arm travel for tip_free
tip_free = 1.0      # mm — swing gear tips clear of the output's tips when parked:
                    #      the arm may rest anywhere in +-6.5 deg and stay free
mesh_play = 0.1     # mm — centre distance over nominal at the stop (backlash)
pin_hub_d = 9.0     # mm — the pinion's hub, the arm's bearing: round the horn's hub
arm_bore  = 0.30    # mm — arm's bore over the hub (PETG sliding +0.25, freer)
arm_t     = 1.6     # mm — arm plate, behind the gear slab
arm_hub_r = 5.5     # mm — arm round the hub
arm_lobe_r = 3.0    # mm — arm round each swing gear's pin
arm_web   = 5.0     # mm — the arm's webs to the pins
sw_pin_d  = 3.0     # mm — the swing gears' pins, printed with the arm
sw_bore   = 0.30    # mm — swing gear bore over its pin (sliding, PETG)
stop_r    = 10.5    # mm — the stop posts' radius round the servo shaft: clear
                    #      of the z18 pinion's tips (8.0) by 0.5
stop_ang  = 45.0    # deg — each post off the arm's tail
stop_d    = 4.0     # mm — stop post diameter (also carries the N20 mount)
plate_t   = 1.5     # mm — the servo mount's plate, between tower top and arm
sleeve_t  = 1.2     # mm — its sleeve round the gearbox
sleeve_len = 8.0    # mm — sleeve length along the gearbox (9 long)
sleeve_fit = 0.10   # mm — sleeve over the gearbox's flats (flat faces barely
                    #      shrink: hex 8.1 on 8.0 was a press, 2026-10-17)
shaft_fit = 0.05    # mm — pinion's D bore over the shaft (PETG press +0.05)

# ── Gears: two identical spur gears per wall, 1:1 ───────────────────────────
g_mod   = 0.8    # mm — module (prints with a 0.4 nozzle)
g_z     = 18     # teeth. Their centre distance sets where the servo stands:
                 #      the pinwheel opens up if it has to
g_alpha = 20.0   # deg — pressure angle
g_gap   = 0.2    # mm — axial gap: servo pack to the gears, gears to the wall
                 #      (0.3 in v1; tightened for the 56 mm LEGO cube)
g_w_min = 3.7    # mm — face width at least; the gears take whatever the slab gives
wall_min = 0.6   # mm — skin left behind the servo gear's boss recess: 3 layers,
                 #      printed on the bed, so it comes out clean (user: as thin
                 #      as it has to be, 2026-10-20). It only closes the face and
                 #      keeps the gear on the spline. 1.6 in v1

# ── v3: cube size and the angle sensor ──────────────────────────────────────
cube_side  = 72.0   # mm — 9 LEGO modules (decided 2026-11-11); odd, so the output
                    #      stays on the Technic grid
# Sensor gear: same module and teeth as the output gear (1:1), straight under it.
sg_dir     = 30.0   # deg (v4b: up-right; -60 in v3/v4) — where it sits round the output, in the wall plane (x, z):
                    #      clear of the servo gear and, with its board, of its servo
sn_pin_d   = 4.0    # mm — the wall's pin it turns on
sn_bore    = 0.30   # mm — bore over the pin (PETG sliding fit +0.25, a bit freer)
sn_boss_r  = 4.0    # mm — boss behind the teeth that holds the magnet
sn_boss_l  = 2.0    # mm — its length behind the teeth
# Magnet (AS5600 kit), measured 2026-11-12 (+0.04): Ø4.00 x 2.00, diametral.
mag_d      = 4.00
mag_t      = 2.00
mag_grow   = 0.30   # mm — its pocket, from the back of the boss: a snug push
                    #      (PETG holes ~0.27 small); a drop of glue if loose
mag_floor  = 0.6    # mm — boss left between the pocket and the pin's bore
# AS5600 AYWHP board, measured 2026-11-12 (+0.04): 22.90 x 23.45 x 1.66 PCB,
# 3.25 over the chip (chip side 1.59), nothing on the back, chip centred,
# 4 screw holes 3.50 in from both edges, pins in two rows (4 and 3) parallel
# to the 22.90 sides, 4.43 in from them: row of 3 = VCC OUT GND, row of 4 =
# DIR SCL SDA GPO. Wired by I2C: VCC (3.3 V), GND, SCL, SDA; DIR bridged to
# GND on the chip side; OUT and GPO unused. Chip side toward the magnet.
# Laid with 22.90 across (x). The servo lying along the wall still passes
# behind the board (v3: the servo lying along the wall; v4: the neighbour N20)
# the box's wall and the neighbour's sleeve. Wires come in from the chip side.
as_w, as_h = 22.90, 23.45   # mm — x, z
as_pcb     = 1.66
as_chip    = 3.25 - 1.66    # mm — chip side, PCB face to the chip's top
as_hole_in = 3.50   # mm — screw holes in from both edges
as_hole_d  = 3.60   # mm — measured 2026-11-12 (3.56 + 0.04): M3 self-tappers
as_air     = 0.8    # mm — magnet face to chip top (AS5600 wants 0.5-3)
as_post_d  = 6.0    # mm — two posts at the LOWER holes, M3 pilots; the upper
                    #      holes stay free (the output gear sweeps there)
post_in_d  = 5.0    # mm — the posts' thin end, through the keeper, to the board
                    #      (M3 pilot 3.0: 1.0 of wall)
post_big_d = 10.0   # mm — the posts, wall side of the keeper: this round, less a
                    #      bite where the sensor gear sweeps, so the keeper rests
                    #      on a broad face, not a 0.35 ring (user, 2026-11-12)
keeper_tube = 8.0   # mm — its tubes round the posts' thin ends
keeper_slide = 0.3  # mm — tube bore over the post (PETG sliding +0.25)
keeper_short = 0.1  # mm — tubes stop short of the board: the board seats on
                    #      the posts, the keeper keeps this much play
keeper_r   = 8.0    # mm — its disc round the sensor gear (the gear's tip radius)
keeper_hole = 0.3   # mm — round the magnet's boss

# ── Clearances and walls ────────────────────────────────────────────────────
clr      = 0.5   # mm — general clearance between parts that do not touch
pack_gap = 0.5   # mm — between neighbouring servos in the pinwheel, at least
deck_t   = 3.0   # mm — deck thickness (v5: no longer the wall's)
col_w_max = 7.5  # mm — corner columns of the frame, square: this, or as big as
                 #      the servo gear passing the corner lets them
screw_in = 4.0   # mm — deck screws in from both outer edges (margin round the
                 #      countersunk head)
col_foot_w = 7.0 # mm — each column's free end, where the other deck's M3 screw
col_foot_h = 8.5 #      goes in, widens to this square for this long, under it a
                 #      45 deg flare from the column: a 5.1 column left the M3
                 #      pilot breaking out of its inner faces (user, 2026-10-20).
                 #      7.0 clears the modules sliding out (0.34); 7.5 does not.
                 #      As long as the M3 pilot (8) plus 0.5: at 5 the pilot broke
                 #      out under the foot (slicer: floating bits, 2026-10-20)
foot_clear = 0.3 # mm — the walls' corners are notched round the feet
# A lip behind the wall plate's edges, from the frame, so the plate gap does not
# show through and the plate stops on it going in. It cannot stop a module
# coming OUT. At the gears' height the side lips break off: the servo gear
# passes there (and with one deck part a column's side faces a servo-gear side
# on one wall or another).
lip_w = 1.2      # mm — how far the lip reaches in behind the plate's edge
lip_t = 1.0      # mm — its thickness, behind the plate
lip_gap_z = 20.2 # mm — side lips stop within this of the wall centre (v3: 8.4 in
                 #      v2; the (-24,-16) Technic pin's boss sits there now)
plate_gap = 0.2  # mm — wall plate to columns and decks
# The wall plate is SQUARE (user, 2026-10-17): every side is framed by a band
# as wide as a column — the columns at the sides, a skirt of each deck at top
# and bottom. v1 trapped every edge in a groove; v2 has none, so the module
# slides out through its window.
skirt_in = 1.5   # mm — the decks' skirt reaches this far inside the wall plate
# Walls held by two screws through the face, at two grid corners on one
# diagonal (WALL_SCREWS): M2x6 countersunk into bosses that rise from the
# decks behind the wall. The decks stay ONE part: the top deck is the
# bottom one turned over, which takes each wall's lower-corner boss to the
# opposite wall's upper corner on the other diagonal.
WALL_SCREW_AT = (-3, -3)  # grid steps (x, z) of the bottom deck's boss; the top
                          #      deck's lands at (+2, +2) on every wall
ws_boss_w   = 5.0   # mm — boss width (measured room: 5 wide clears the module
                    #      sliding out by 0.6, 2026-10-20)
ws_boss_d   = 3.5   # mm — boss depth behind the wall (more meets the next wall
                    #      at the upper corner)
ws_head_d   = 3.8   # mm — M2 countersunk head
ws_clear    = 2.2   # mm — M2 clearance through the wall

# ── v5 output: hub on a 6700ZZ in the wall, one M3 to the gear ─────────────
brg_od     = 15.0   # mm — 6700ZZ: 10 x 15 x 4 (catalogue; measure + 0.04 when here)
brg_id     = 10.0
brg_w      = 4.0
brg_in_r   = 5.6    # mm — what the hub and gear may touch: the inner ring's face
brg_seat   = 15.2   # mm — seat in the wall, PETG: the 688's measured press (16.1 in
                    #      PLA, PETG ~0.1 tighter) carried over to 15; to check
fl_r       = 6.6    # mm — the hub's flange: the model's face; the counterbore round
                    #      it leaves a 0.7 floor for the bearing to stop on
fl_t       = 1.2    # mm — its thickness
fl_relief  = 0.2    # mm — flange over the bearing's outer ring: this clear
cb_clear   = 0.3    # mm — counterbore round the flange
journal_d  = 9.95   # mm — hub in the inner ring (prints ~+0.07: a snug slide)
hex_af     = 8.0    # mm — hub's hex into the gear: torque only, no press
hex_hole   = 8.15   # mm — gear's hex socket, AF (slides)
hex_len    = 2.0    # mm — how deep the hex goes into the gear
gb_boss_r  = 3.5    # mm — gear's boss behind its teeth, for the screw's thread
gb_boss_l  = 4.0
m3_head_d  = 6.0    # mm — M3 countersunk head, 90 deg
m3_screw_l = None   # mm — M3 countersunk, from the kit: picked below
# The model's coupling (2026-11-13): a TRAPEZOID socket in the hub's face —
# a square with one side shorter (user's idea; a D first, but a D carries the
# torque on one small flat): four faces share the torque, one position only
# (the firmware parks the output with the short side up, so a model goes on
# without aiming), flat faces print true. A groove inside it the model's split
# spigot clicks into. The M3 comes in from INSIDE (head in the gear's boss), so
# the hub's face is free for this.
key_a      = 4.6    # mm — the long side (down when parked)
key_b      = 3.2    # mm — the short side (up)
key_h      = 4.6    # mm — between them
key_depth  = 2.8    # mm — socket depth from the hub's face
key_grow   = 0.15   # mm — socket over nominal, on every side's offset x 2
key_play   = 0.15   # mm — the model's spigot under nominal
groove_at  = 1.8    # mm — groove's centre, down from the hub's face
groove_w   = 0.6    # mm — its width (along the axis)
groove_d   = 0.3    # mm — how far it cuts into the socket's wall
bump_h     = 0.25   # mm — the spigot's click bump (into the groove)
slit_w     = 1.0    # mm — two slits across the spigot let it flex
hub_recess = 0.2    # mm — flange face below the outer face, so it never rubs
wall_t     = hub_recess + fl_t + fl_relief + brg_w    # 5.6: flush face, bearing in
wall_screw_l = 10.0 # mm — M2 countersunk through the thicker wall into the deck boss

# ── (v4b1) Output: a printed hub with a Technic cross hole, no bearings ─────
# Two parts, captured by the wall alone: the HUB comes in from outside, its
# flange sitting in a counterbore flush with the face (it cannot go in); the
# GEAR is pressed onto the hub's hex from inside (it cannot come out — its
# face meets the wall). The hub turns in a plain hole.
hub_d      = 10.0   # mm — hub diameter, runs in the wall
hub_clear  = 0.2    # mm — radial clearance: hole 10.4. Printed OD comes ~+0.07 (PETG cross,
                    #      4.6 -> 4.67 avg of 10, 2026-10-15) and a turning fit wants +0.3 (T1.1)
hub_recess = 0.2    # mm — flange face below the outer face, so it never rubs
flange_d   = 14.0   # mm — the flange is a 45 deg CONE (hub_d to flange_d), and so
                    #      is its seat in the wall: a flat step printed as a ring
                    #      in the air (strings, 2026-10-17); a cone prints clean
                    #      and centres the hub

# ── LEGO Technic (LEGO's nominal sizes; the holes grow by hole_grow) ─────────
tech_pitch   = 8.0    # mm — the grid
tech_pin_d   = 4.8    # mm — pin hole
tech_cb_d    = 6.2    # mm — counterbore at the outer face, for the pin's collar
tech_cb_h    = 0.8    # mm
tech_len     = 7.8    # mm — a beam's thickness: the boss's length. Counterbored
                      #      0.8 at both ends like a beam; the pin goes 7.1 past
                      #      its collar and its ridge clicks into the far one
tech_boss_d  = 7.6    # mm — boss round each pin hole, inside the wall
TECH_PINS    = []   # v5: LEGO off the motor wall   # v3: clear of the servo, the gears
                      # and the AS5600 posts; (-2, +-1) hit the servo gear
                      # pin holes in grid steps (x, z) from the output: two
                      # under it at +-8 and one in the upper corner on the
                      # servo gear's side (user, 2026-10-20). The top row and
                      # the +16 column put the pin's tip into the neighbour
                      # servo; the other corners take the wall screws
TECH_SHORT   = {}     # mm — bosses cut short where a full 7.8 won't fit
tech_cross   = 4.8    # mm — axle's cross, tip to tip
tech_cross_w = 1.8    # mm — width of its arms
cross_try    = []     # extra hubs to print with other cross grows (PETG)
hole_grow    = 0.10   # mm — on holes that take LEGO parts: what printed holes lose
                      #      (PLA, arcs off: the 6.0 socket printed 5.90)
pin_grow     = hole_grow   # mm — LEGO pin holes (PETG: measured, below)

# ── Holes (calibrated in PLA, test-status T1.1, 2026-10) ────────────────────
m2_pilot   = 1.6    # mm — M2 self-tapping pilot (as the v6 servo cradle)
m3_pilot   = 3.0    # mm — M3 self-tapping pilot (T1.1: 3.0 in PLA; 2.5 needed
                    #      the holes opened up by hand, 2026-10-17)
m3_clear   = 3.3    # mm

# Cable passage through the decks (kept from v2; the brain box is gone:
# battery and boards now live INSIDE the 72 mm cube).
cable_hole = (-7.0, -0.5, -5.0, 5.0)   # mm — x0, x1, y0, y1, inside the pack's hole
if MATERIAL == "PETG":   # fit coupon in PETG, arcs off, scarf seam (2026-10-20):
                         # press +0.05, sliding +0.25; holes print ~0.27 small
    hub_clear = 0.125        # hub's hole: sliding, +0.25 on the diameter
    hole_grow = 0.10         # LEGO axle's cross: 4.90 / 1.90. The coupon's 4.97
                             #      was too loose in the hub (6.5 deep); of two hubs,
                             #      4.92 and 4.87, the 4.87 won (2026-10-21), but
                             #      three more at 4.87 came out too tight
                             #      (2026-10-28): prints scatter, take the middle
    pin_grow = 0.15          # LEGO pin holes: on the Technic coupon (2026-10-20)
                             #      4.95 felt most like a real beam (5.00 close)
    m2_pilot = 1.7           # M2 self-tapping: PETG holes print small (M3 needed
                             #      3.1 in PETG, 3.0 in PLA: T1.2)
    ws_clear = 2.4
    # M3 pilots stay 3.0: in the PETG decks (2026-10-20) they went in just right
elif MATERIAL != "PLA":
    raise ValueError(f"MATERIAL must be PLA or PETG, not {MATERIAL!r}")

EXPORT_STL = True
RUN_CHECKS = None   # None: checks when headless, skipped in the GUI

# ═══════════════════════════════════════════════════════════════════
# DERIVED GEOMETRY  (everything for the north module, +Y wall)
# ═══════════════════════════════════════════════════════════════════

r_pitch = g_mod * g_z / 2.0
r_tip   = r_pitch + g_mod
a_mesh  = 2 * r_pitch                  # centre distance, equal gears

r_pin_p = g_mod * z_pin / 2.0           # pinion pitch radius
r_sw_p  = g_mod * z_sw / 2.0            # swing gears' pitch radius
a_ps    = r_pin_p + r_sw_p              # pinion - swing gear
a_so    = r_sw_p + r_pitch              # swing gear - output, nominal

# Stack from the wall in: wall, gear slab, arm, the N20's mount plate, the
# N20's gearbox face. The output, the sensor gear, the pinion and both swing
# gears share the slab.
y_wo    = cube_side / 2.0               # wall outer face = cube half side
y_wi    = y_wo - wall_t                 # wall inner face
half_in = y_wo - deck_t                 # cube centre to a deck's inner face (v5)
y_g1    = y_wi - g_gap                  # gear slab, outer face
y_g0    = y_g1 - g_w_min                # gear slab, inner face
g_w     = y_g1 - y_g0                   # gears' face width
y_a1    = y_g0 - g_gap                  # arm, outer face
y_a0    = y_a1 - arm_t                  # arm, inner face
y_p1    = y_a0 - g_gap                  # mount plate, outer face (the posts end here)
y_m     = y_p1 - plate_t                # N20's gearbox face

# Shafts in the wall plane (x, z).
S_xz = (x_sv, -math.sqrt(n20_d ** 2 - x_sv ** 2))   # servo shaft, pinion, arm pivot
n20_dir = math.degrees(math.atan2(S_xz[1], S_xz[0]))
sv_z    = S_xz[1]
# The servo, standing (case length along Z), the shaft at its lower end.
y_tower = y_m                           # tower top under the mount plate
y_base  = y_tower - sv_tower_h
y_tab0  = y_base + sv_tab_z0
y_tab1  = y_tab0 + sv_tab_t
sv_x0, sv_x1 = x_sv - sv_case_t / 2.0, x_sv + sv_case_t / 2.0
case_z0 = sv_z - sv_shaft_end
case_z1 = case_z0 + sv_case_len
tab_z0  = case_z0 - sv_tab_out
tab_z1  = case_z1 + sv_tab_out
tab_hole_z = (tab_z0 + sv_tab_hole, tab_z1 - sv_tab_hole)
y_horn  = y_tower + horn_lift + horn_hub_h      # horn top
assert y_horn + horn_grow < y_g1 - 0.5, "horn pokes out of the pinion"
O_xz = (0.0, 0.0)                       # output, wall centre
Q_xz = (a_mesh * math.cos(math.radians(sg_dir)),
        a_mesh * math.sin(math.radians(sg_dir)))   # sensor gear
th_so = math.atan2(O_xz[1] - S_xz[1], O_xz[0] - S_xz[0])   # direction N20 -> output


def _arm_angle(dist):
    """Arm angle off the N20->output line at which a swing gear's centre is
    `dist` from the output's (law of cosines in the triangle S, P, O)."""
    return math.acos((a_ps ** 2 + n20_d ** 2 - dist ** 2) / (2 * a_ps * n20_d))


psi_rest  = _arm_angle(a_so + 2 * g_mod + tip_free)   # parked: tips tip_free apart
psi_touch = _arm_angle(a_so + 2 * g_mod)              # tips just touch
psi_mesh  = _arm_angle(a_so + mesh_play)              # at the stop
arm_T     = math.degrees(psi_rest - psi_mesh)         # arm travel, park -> stop
free_T    = math.degrees(psi_rest - psi_touch)        # parking window, each way


def _pol(c, r, a):
    return (c[0] + r * math.cos(a), c[1] + r * math.sin(a))


PA = _pol(S_xz, a_ps, th_so - psi_rest)   # swing gear A: in when the arm turns +
PB = _pol(S_xz, a_ps, th_so + psi_rest)   # swing gear B: in when the arm turns -
tail = th_so + math.pi                     # the arm's tab, between the two posts
POSTS = [_pol(S_xz, stop_r, tail + s * math.radians(stop_ang)) for s in (1, -1)]
# The tab's flat sides meet the posts exactly when the arm has turned arm_T:
# post centre to the tab's centreline = half the tab + the post's radius.
tab_w  = 2 * (stop_r * math.sin(math.radians(stop_ang - arm_T)) - stop_d / 2.0)
tab_r1 = (stop_r * math.cos(math.radians(stop_ang - arm_T))   # v4b: the tab ends just
          + stop_d / 2.0 + 0.8)                  #      past where it meets the post,
                                                 #      clear of the deck's screw boss

# Sensor stack, from the gear slab inward: boss, magnet in its back, air, chip, PCB.
y_sb    = y_g0 - sn_boss_l              # boss's back face = magnet's face
y_chip  = y_sb - as_air                 # chip top
y_pcbf  = y_chip - as_chip              # PCB, chip side
y_pcbb  = y_pcbf - as_pcb               # PCB, back
y_kf    = y_g0 - g_gap                  # keeper, gear side: the posts end here
y_kb    = y_sb                          # keeper, board side
keeper_t = y_kf - y_kb
AS_HOLES = [(Q_xz[0] + sx * (as_w / 2.0 - as_hole_in), Q_xz[1] + sz * (as_h / 2.0 - as_hole_in))
            for sz in (-1, 1) for sx in (-1, 1)]
AS_HOLES.sort(key=lambda p: -math.hypot(p[0], p[1]))   # v4b: posts at the two
                                        #      holes farthest from the output

# Corner columns of the frame occupy |x|,|y| in [col_in, y_wo].
col_w  = col_w_max
col_in = y_wo - col_w
lim_x  = col_in - plate_gap             # wall plate half width
TECH_XZ = [(ix * tech_pitch, iz * tech_pitch) for ix, iz in TECH_PINS]
TECH_DEPTH = [TECH_SHORT.get(p, tech_len) for p in TECH_PINS]
WS_LOW = (WALL_SCREW_AT[0] * tech_pitch, WALL_SCREW_AT[1] * tech_pitch)
WALL_SCREWS = [WS_LOW, (-WS_LOW[0], -WS_LOW[1])]   # bottom deck's, top deck's
assert all(max(abs(x), abs(z)) + tech_cb_d / 2.0 < lim_x for x, z in TECH_XZ),     "pin holes off the plate"
assert tab_w > 3.0, f"arm tab only {tab_w:.2f} wide: move the stop posts"

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

def mesh_phase(phase_a, za, a_xz, b_xz, zb):
    """Phase for gear B so it meshes with A (external mesh): a tooth of A at
    theta + e (theta = direction A->B) meets a GAP of B at theta + pi - e*za/zb."""
    th = math.atan2(b_xz[1] - a_xz[1], b_xz[0] - a_xz[0])
    pitch_a = 2 * math.pi / za
    e = (phase_a - th) % pitch_a
    if e > pitch_a / 2:
        e -= pitch_a
    return th + math.pi - e * za / zb + math.pi / zb


def make_gear(cx, cz, phase, y0, y1, bore=None, z=g_z):
    pts = [v(cx + u, y0, cz + w) for u, w in gear_outline(z, g_mod, phase)]
    face = Part.Face(Part.makePolygon(pts + [pts[0]]))
    g = face.extrude(v(0, y1 - y0, 0))
    if bore is not None:
        g = g.cut(bore)
    return g


def rot_xz(shape, c, deg):
    """A copy turned `deg` about the Y axis through (c, *) — positive the way
    atan2(z, x) grows, i.e. the arm's + sense."""
    s = shape.copy()
    s.rotate(v(c[0], 0, c[1]), Y_AXIS, -deg)
    return s


def bar_xz(p, q, w, y0, y1):
    """A straight web `w` wide from p to q in the wall plane, y0..y1 thick."""
    L = math.hypot(q[0] - p[0], q[1] - p[1])
    b = box(0, L, y0, y1, -w / 2.0, w / 2.0)
    b.rotate(ORIGIN, Y_AXIS, -math.degrees(math.atan2(q[1] - p[1], q[0] - p[0])))
    b.translate(v(p[0], 0, p[1]))
    return b


def d_shaft(cx, cz, y0, y1, grow=0.0):
    """The N20's D shaft (or, grown, its hole), flat toward +x."""
    r = n20_shaft_d / 2.0 + grow / 2.0
    flat = n20_flat - n20_shaft_d / 2.0 + grow / 2.0     # flat's distance from the axis
    return cyl_y(cx, cz, r, y0, y1).cut(
        box(cx + flat, cx + r + 1, y0 - 1, y1 + 1, cz - r - 1, cz + r + 1))


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
    spline = cyl_y(x_sv, sv_z, sv_spline_d / 2.0, y_tower - 0.01, y_tower + sv_spline_h)
    return case.fuse([tower, tabs, spline]).removeSplitter()


def _horn(grow):
    """The horn (or, grown, its pocket): hub + two arms along X, arms flush
    with the hub's top."""
    y0 = y_horn - horn_hub_h
    hub_g = grow if grow == 0.0 else horn_hub_grow
    hub = cyl_y(x_sv, sv_z, horn_hub_d / 2.0 + hub_g, y0 - grow, y_horn + grow)
    half = horn_arm_len / 2.0 + grow
    arms = box(x_sv - half, x_sv + half, y_horn - horn_arm_t - grow, y_horn + grow,
               sv_z - horn_arm_w / 2.0 - grow, sv_z + horn_arm_w / 2.0 + grow)
    return hub.fuse(arms)


def make_horn():
    h = _horn(0.0).removeSplitter()
    return h.cut(cyl_y(x_sv, sv_z, sv_spline_d / 2.0, y_tower - 1, y_horn - 1.0))


def make_pinion():
    """The servo gear as the pinion: z18 teeth in the slab, a hub behind them
    (the arm's bearing), the horn's pocket open toward the servo, a floor over
    the horn that keeps it on the spline. Printed floor down."""
    g = make_gear(S_xz[0], S_xz[1], PH_P, y_g0, y_g1, z=z_pin)
    g = g.fuse(cyl_y(S_xz[0], S_xz[1], pin_hub_d / 2.0, y_p1 + 0.1, y_g0 + 0.01)).removeSplitter()
    g = g.cut(_horn(horn_grow))
    return g.cut(cyl_y(x_sv, sv_z, horn_hub_d / 2.0 + horn_hub_grow, y_p1 - 1, y_horn))


def make_arm():
    """The swing arm: a plate behind the gear slab, turning on the pinion's
    hub, with the two swing gears' pins and a tab that swings between the
    two stop posts. Printed flat, pins up."""
    def disk(c, r):
        return cyl_y(c[0], c[1], r, y_a0, y_a1)
    a = disk(S_xz, arm_hub_r)
    parts = [disk(PA, arm_lobe_r), disk(PB, arm_lobe_r),
             bar_xz(S_xz, PA, arm_web, y_a0, y_a1), bar_xz(S_xz, PB, arm_web, y_a0, y_a1),
             bar_xz(S_xz, _pol(S_xz, tab_r1, tail), tab_w, y_a0, y_a1)]
    parts += [cyl_y(P[0], P[1], sw_pin_d / 2.0, y_a1 - 0.01, y_g1) for P in (PA, PB)]
    a = a.fuse(parts).removeSplitter()
    return a.cut(cyl_y(S_xz[0], S_xz[1], (pin_hub_d + arm_bore) / 2.0, y_a0 - 1, y_a1 + 1))


def make_swing_gear(P, phase):
    """A z12 swing gear on its pin of the arm, always in mesh with the pinion."""
    return make_gear(P[0], P[1], phase, y_g0, y_g1,
                     cyl_y(P[0], P[1], (sw_pin_d + sw_bore) / 2.0, y_g0 - 1, y_g1 + 1), z_sw)


def make_mount():
    """The servo's mount: a plate over the tower top (under the arm), a
    sleeve down over the tower and the case's top to the tab plate, two M2
    self-tappers up through the tabs into it, and two webs to the stop posts
    (M2 from below into the posts). Printed plate down."""
    f = sleeve_fit / 2.0
    ox0, ox1 = sv_x0 - f - sleeve_t, sv_x1 + f + sleeve_t
    m = box(ox0, ox1, y_tab1, y_p1, tab_z0, tab_z1)
    parts = []
    for P in POSTS:
        parts.append(cyl_y(P[0], P[1], stop_d / 2.0 + 1.0, y_m, y_p1))
        parts.append(bar_xz(S_xz, P, stop_d + 2.0, y_m, y_p1))
    m = m.fuse(parts).removeSplitter()
    m = m.cut(box(sv_x0 - f, sv_x1 + f, y_tab1 - 1, y_base + sv_case_h + f,
                  case_z0 - f, case_z1 + f))
    m = m.cut(box(sv_x0 - f, sv_x1 + f, y_tab1 - 1, y_tower + f,
                  case_z0 - f, case_z0 + sv_tower_len + f))
    m = m.cut(cyl_y(x_sv, sv_z, sv_spline_d / 2.0 + 0.6, y_m - 1, y_p1 + 1))
    for hz in tab_hole_z:
        m = m.cut(cyl_y(x_sv, hz, m2_pilot / 2.0, y_tab1 - 1, y_tab1 + 5.0))
    for P in POSTS:
        m = m.cut(cyl_y(P[0], P[1], ws_clear / 2.0, y_m - 1, y_p1 + 1))
    # v5: the next wall's servo (its base at x = y_base here, the pinwheel)
    # passes 0.5 from the sleeve's side: trim the sleeve there
    return m.cut(box(y_base - 0.5, y_wo, -y_wo, -sv_x0 + 0.5, -y_wo, y_wo))


def hex_y(af, y0, y1):
    """Hexagonal prism along Y on the wall centre, `af` across flats, flats
    top and bottom."""
    r = af / math.sqrt(3.0)
    pts = [v(r * math.cos(math.radians(60 * k)), y0, r * math.sin(math.radians(60 * k)))
           for k in range(6)]
    return Part.Face(Part.makePolygon(pts + [pts[0]])).extrude(v(0, y1 - y0, 0))

y_fl1 = y_wo - hub_recess                # flange's outer face
y_fl0 = y_fl1 - fl_t                    # its underside
y_b1  = y_fl0 - fl_relief               # bearing's outer face = counterbore floor
y_b0  = y_b1 - brg_w                    # bearing's inner face = wall's inner face
assert abs(y_b0 - y_wi) < 1e-9
y_key0 = y_fl1 - key_depth              # D socket's floor
y_bt  = y_g0 - gb_boss_l                # gear boss's inner end: the screw's head
# The screw, from inside: the shortest kit length that bites >= 4 into the hub
# (from its hex's end); its tip must stay 0.4 under the socket's floor.
for m3_screw_l in (8.0, 10.0, 12.0, 14.0, 16.0, 20.0):
    if (y_bt + m3_screw_l) - (y_b0 - hex_len) >= 4.0:
        break
y_scr_end = y_bt + m3_screw_l           # the screw's tip


def make_output_gear():
    """Output gear: a short hex socket for the hub on its outer side, a boss
    on its inner side for the M3's thread, a 0.2 ring that reaches the
    bearing's inner ring. Printed outer face down."""
    g = make_gear(0, 0, PH_O, y_g0, y_g1)
    g = g.fuse([cyl_y(0, 0, gb_boss_r, y_g0 - gb_boss_l, y_g0 + 0.01),
                cyl_y(0, 0, brg_in_r, y_g1 - 0.01, y_b0)]).removeSplitter()
    g = g.cut(hex_y(hex_hole, y_b0 - hex_len - 0.2, y_b0 + 1))
    g = g.cut(cyl_y(0, 0, m3_clear / 2.0, y_bt - 1, y_b0))
    return g.cut(Part.makeCone(m3_head_d / 2.0 + 0.1, 0.0, m3_head_d / 2.0 + 0.1,
                               v(0, y_bt - 0.001, 0), Y_AXIS))


def make_output_hub():
    """Hub: the flange (the model's face, flush), a ring under it that bears
    on the inner ring only, the journal through the bearing, a short hex into
    the gear; the M3's pilot from inside; the model's D socket with its click
    groove in the face. Printed flange down."""
    fl = cyl_y(0, 0, fl_r, y_fl0, y_fl1)
    ring = cyl_y(0, 0, brg_in_r, y_b1, y_fl0 + 0.01)
    journal = cyl_y(0, 0, journal_d / 2.0, y_b0 - 0.01, y_b1 + 0.01)
    spigot = hex_y(hex_af, y_b0 - hex_len, y_b0 + 0.01)
    h = fl.fuse([ring, journal, spigot]).removeSplitter()
    h = h.cut(cyl_y(0, 0, m3_pilot / 2.0, y_b0 - hex_len - 1, y_key0 - 0.4))
    h = h.cut(trap_key(key_grow / 2.0, y_key0, y_fl1 + 1))
    yg = y_fl1 - groove_at
    return h.cut(cyl_y(0, 0, key_r + key_grow / 2.0 + groove_d,
                       yg - groove_w / 2.0, yg + groove_w / 2.0))


def _trap_pts():
    """The trapezoid's corners in (x, z), shifted up so its far corners are all
    the same distance from the axis (the smallest round it fits)."""
    c = (key_a ** 2 - key_b ** 2) / (8.0 * key_h)
    return [(-key_a / 2.0, -key_h / 2.0 + c), (key_a / 2.0, -key_h / 2.0 + c),
            (key_b / 2.0, key_h / 2.0 + c), (-key_b / 2.0, key_h / 2.0 + c)]


key_r = max(math.hypot(x, z) for x, z in _trap_pts())   # its corners' radius


def trap_key(off, y0, y1):
    """The trapezoid along Y on the wall centre, every side moved out by `off`
    (negative: in), y0..y1."""
    pts = [v(x, y0, z) for x, z in _trap_pts()]
    f = Part.Face(Part.makePolygon(pts + [pts[0]]))
    if off:
        f = f.makeOffset2D(off, 2)              # 2: sharp corners
    return f.extrude(v(0, y1 - y0, 0))


def make_model_key():
    """The model's side, as a template part (and a first test piece): a disc
    in the counterbore on the hub's face, a plate 0.2 clear of the wall's face
    with an arm, and the split trapezoid spigot with its click bump."""
    k = cyl_y(0, 0, fl_r - 0.2, y_fl1, y_wo + 0.2)
    k = k.fuse([cyl_y(0, 0, 8.0, y_wo + 0.2, y_wo + 2.2),
                box(-3.0, 3.0, y_wo + 0.2, y_wo + 2.2, 0.0, 24.0)])
    sp = trap_key(-key_play / 2.0, y_key0 + 0.2, y_fl1 + 0.01)
    yg = y_fl1 - groove_at
    bump = trap_key(-key_play / 2.0 + bump_h, yg - groove_w / 2.0 + 0.1,
                    yg + groove_w / 2.0 - 0.1)
    sp = sp.fuse(bump)
    for ang in (0.0, 90.0):     # two slits across: four fingers flex in
        sl = box(-slit_w / 2.0, slit_w / 2.0, y_key0, y_fl1 - 0.6, -5, 5)
        sl.rotate(ORIGIN, Y_AXIS, ang)
        sp = sp.cut(sl)
    return k.fuse(sp).removeSplitter()


def make_bearing():
    """6700ZZ envelope: a ring 10 x 15 x 4."""
    return cyl_y(0, 0, brg_od / 2.0, y_b0, y_b1).cut(
        cyl_y(0, 0, brg_id / 2.0, y_b0 - 1, y_b1 + 1))


def make_screw():
    """M3 countersunk from inside, head flush with the gear boss's end."""
    head = Part.makeCone(m3_head_d / 2.0, 0.0, m3_head_d / 2.0, v(0, y_bt, 0), Y_AXIS)
    return head.fuse(cyl_y(0, 0, 1.4, y_bt + 0.5, y_scr_end))


def make_wall():
    """The wall module: flat plate, the output's hole, the sensor gear's pin,
    the AS5600's posts, the two stop posts that also carry the N20's mount,
    the Technic pin holes on bosses. One part, printed outer face down."""
    plate = box(-lim_x, lim_x, y_wi, y_wo, -lim_x, lim_x)     # square
    posts = [cyl_y(Q_xz[0], Q_xz[1], sn_pin_d / 2.0, y_sb + mag_t + mag_floor + 0.3,
                   y_wi + 0.01)]
    for hx, hz in AS_HOLES[:2]:
        posts.append(cyl_y(hx, hz, as_post_d / 2.0, y_kf, y_wi + 0.01))
        posts.append(cyl_y(hx, hz, post_big_d / 2.0, y_kf, y_wi + 0.01).cut(
            cyl_y(Q_xz[0], Q_xz[1], r_tip + clr, y_kf - 1, y_wi + 1)))
        posts.append(cyl_y(hx, hz, post_in_d / 2.0, y_pcbf, y_kf + 0.01))
    for P in POSTS:                             # v4: arm stops + N20 mount
        posts.append(cyl_y(P[0], P[1], stop_d / 2.0, y_p1, y_wi + 0.01))
    lbosses = [cyl_y(cx, cz, tech_boss_d / 2.0, y_wo - d, y_wi + 0.01)
               for (cx, cz), d in zip(TECH_XZ, TECH_DEPTH)]
    w = plate.fuse(posts + lbosses).removeSplitter()
    for sx in (1, -1):
        for top in (True, False):
            w = w.cut(_col_foot(sx, 1, top, foot_clear))   # north wall's corners
    # Output: the hub's plain bearing hole, with a 45 deg conical seat.
    # v5: the 688's seat, open to the inside (it goes in from there and stops
    # on the counterbore's floor); the flange's counterbore from outside.
    w = w.cut(cyl_y(0, 0, brg_seat / 2.0, y_wi - 1, y_b1))
    w = w.cut(cyl_y(0, 0, fl_r + cb_clear, y_b1 - 0.01, y_wo + 1))
    for hx, hz in AS_HOLES[:2]:                 # board screws, from behind
        w = w.cut(cyl_y(hx, hz, m3_pilot / 2.0, y_pcbf - 1, y_pcbf + 8.0))
    for P in POSTS:                             # the mount's M2 screws
        w = w.cut(cyl_y(P[0], P[1], m2_pilot / 2.0, y_p1 - 1, y_p1 + 4.5))
    for cx, cz in WALL_SCREWS:
        w = w.cut(cyl_y(cx, cz, ws_clear / 2.0, y_wi - 1, y_wo + 1))
        h = ws_head_d / 2.0 + 0.1
        w = w.cut(Part.makeCone(h, 0.0, h, v(cx, y_wo + 0.001, cz), v(0, -1, 0)))
    for (cx, cz), d in zip(TECH_XZ, TECH_DEPTH):
        w = w.cut(cyl_y(cx, cz, (tech_pin_d + pin_grow) / 2.0,
                        y_wo - d - 1, y_wo + 1))
        w = w.cut(cyl_y(cx, cz, (tech_cb_d + pin_grow) / 2.0,
                        y_wo - tech_cb_h, y_wo + 1))
        if d >= tech_len:
            w = w.cut(cyl_y(cx, cz, (tech_cb_d + pin_grow) / 2.0,
                            y_wo - tech_len - 1, y_wo - tech_len + tech_cb_h))
    return w
def make_sensor_gear():
    """Sensor gear: the output's twin, meshing under it, on the wall's pin;
    a boss behind with the magnet's pocket. Printed teeth face down."""
    g = make_gear(Q_xz[0], Q_xz[1], PH_Q, y_g0, y_g1)
    g = g.fuse(cyl_y(Q_xz[0], Q_xz[1], sn_boss_r, y_sb, y_g0 + 0.01)).removeSplitter()
    g = g.cut(cyl_y(Q_xz[0], Q_xz[1], (sn_pin_d + sn_bore) / 2.0,
                    y_sb + mag_t + mag_floor, y_g1 + 1))
    return g.cut(cyl_y(Q_xz[0], Q_xz[1], (mag_d + mag_grow) / 2.0, y_sb - 1, y_sb + mag_t))


def make_keeper():
    """Keeper: a disc behind the sensor gear's teeth (0.2 off them), a hole for
    the magnet's boss, and two tubes that slide over the posts' thin ends,
    down to just short of the board. Trapped between the posts' step and the
    board; not screwed. Printed flat, the gear side down."""
    k = cyl_y(Q_xz[0], Q_xz[1], keeper_r, y_kb, y_kf)
    for hx, hz in AS_HOLES[:2]:
        k = k.fuse(cyl_y(hx, hz, keeper_tube / 2.0, y_pcbf + keeper_short, y_kf))
        # web from the tube to the disc, as wide as the tube
        ang = math.atan2(hz - Q_xz[1], hx - Q_xz[0])
        mx, mz = (hx + Q_xz[0]) / 2.0, (hz + Q_xz[1]) / 2.0
        L = math.hypot(hx - Q_xz[0], hz - Q_xz[1])
        web = box(-L / 2.0, L / 2.0, y_kb, y_kf, -as_post_d / 2.0, as_post_d / 2.0)
        web.rotate(ORIGIN, Y_AXIS, -math.degrees(ang))
        web.translate(v(mx, 0, mz))
        k = k.fuse(web)
    k = k.removeSplitter()
    k = k.cut(cyl_y(Q_xz[0], Q_xz[1], sn_boss_r + keeper_hole, y_kb - 1, y_kf + 1))
    for hx, hz in AS_HOLES[:2]:
        k = k.cut(cyl_y(hx, hz, (post_in_d + keeper_slide) / 2.0, y_pcbf - 1, y_kf + 1))
    return k


def make_magnet():
    return cyl_y(Q_xz[0], Q_xz[1], mag_d / 2.0, y_sb, y_sb + mag_t)


def make_as5600():
    """The AS5600 board: PCB with its four holes, the chip on the axis."""
    b = box(Q_xz[0] - as_w / 2.0, Q_xz[0] + as_w / 2.0, y_pcbb, y_pcbf,
            Q_xz[1] - as_h / 2.0, Q_xz[1] + as_h / 2.0)
    for hx, hz in AS_HOLES:
        b = b.cut(cyl_y(hx, hz, as_hole_d / 2.0, y_pcbb - 1, y_pcbf + 1))
    chip = box(Q_xz[0] - 2.5, Q_xz[0] + 2.5, y_pcbf - 0.01, y_chip,
               Q_xz[1] - 2.0, Q_xz[1] + 2.0)
    return b.fuse(chip)


# Inside the cube (ghosts, to check the room): the LiPo flat over the motor
# band, the brain's boards under it.
battery = (52.0, 33.55, 10.24)  # CITYORK 103450, measured
board_box = (44.0, 44.0, 8.0)   # PLACEHOLDER: one board, C3 + DAOKAI + TCA9548A + gyro + 2 DRV8833


def make_battery(z0):
    L, W, H = battery
    return box(-L / 2, L / 2, -W / 2, W / 2, z0, z0 + H)


def make_main_board():
    X, Y, H = board_box
    z0 = -half_in + clr
    return box(-X / 2, X / 2, -Y / 2, Y / 2, z0, z0 + H)
def cross_y(span, arm, y0, y1):
    """A Technic cross along Y on the wall centre."""
    a = box(-span / 2.0, span / 2.0, y0, y1, -arm / 2.0, arm / 2.0)
    b = box(-arm / 2.0, arm / 2.0, y0, y1, -span / 2.0, span / 2.0)
    return a.fuse(b).removeSplitter()


def cross_depth():
    """The output's cross hole: down the hub, a floor left at the gear."""
    return y_wo - y_g0 - 0.6


def make_tech_pin(cx, cz):
    """Ghost of a LEGO friction pin in a wall hole: its inner half in the
    hole, its outer half where a beam would sit, the collar between."""
    pin = cyl_y(cx, cz, tech_pin_d / 2.0, y_wo - tech_len, y_wo + tech_len)
    return pin.fuse(cyl_y(cx, cz, tech_cb_d / 2.0 - 0.1, y_wo - tech_cb_h + 0.05,
                          y_wo + tech_cb_h - 0.05))


def make_tech_axle():
    """Ghost of a LEGO axle in the output, as deep as the cross hole."""
    return cross_y(tech_cross, tech_cross_w, y_wo - cross_depth(), y_wo + 2 * tech_len)


def make_deck():
    """One deck — top and bottom are the SAME part. Drawn as the bottom one:
    a plate with a skirt round its edges (framing the walls' bottom edges),
    a boss behind each wall for its lower screw, two full-height corner
    columns on one diagonal and two countersunk M3 holes
    on the other. The top deck is this part turned over
    (180 deg about X): its columns come down onto the bottom one's holes and
    the other way round. Four M3x10 countersunk, two from each face, into
    pilots in the column ends."""
    d = box(-y_wo, y_wo, -y_wo, y_wo, -y_wo, -half_in)
    cxy = y_wo - screw_in
    for sx, sy in ((1, 1), (-1, -1)):            # columns
        x0, x1 = sorted((sx * col_in, sx * y_wo))
        y0, y1 = sorted((sy * col_in, sy * y_wo))
        d = d.fuse(box(x0, x1, y0, y1, -half_in - 0.01, half_in))
        d = d.fuse([_col_foot(sx, sy, top) for top in (True, False)])
        d = d.cut(cyl_z(sx * cxy, sy * cxy, m3_pilot / 2.0, half_in - 8.0,
                        half_in + 1))
    bx, bz = WS_LOW
    for rot in (0, 90, 180, 270):
        # The skirt along this edge: frames the wall plate's bottom edge, a
        # column's width from the outer face.
        sk = box(-col_in, col_in, y_wi - skirt_in, y_wo, -half_in - 0.01, -col_in)
        # The wall screw's boss: from the deck up behind the wall, against its
        # inner face, an M2 pilot along the screw.
        bs = box(bx - ws_boss_w / 2.0, bx + ws_boss_w / 2.0, y_wi - ws_boss_d, y_wi,
                 -half_in - 0.01, bz + ws_boss_w / 2.0)
        pilot = cyl_y(bx, bz, m2_pilot / 2.0, y_wi - ws_boss_d - 1, y_wi + 0.01)
        # Lip behind the plate's bottom edge.
        lp = [box(-col_in, col_in, y_wi - lip_t, y_wi, -col_in - 0.01,
                  -(lim_x - lip_w))]
        # ...and behind its side edges, on the columns this deck carries.
        for sx in (1, -1):
            corner = v(sx * col_in, y_wo, 0)
            corner = App.Rotation(Z_AXIS, rot).multVec(corner)
            if corner.x * corner.y > 0:          # this deck's columns: x*y > 0
                x_in, x_col = sx * (lim_x - lip_w), sx * (col_in + 0.3)
                lp.append(box(*sorted((x_in, x_col)), y_wi - lip_t, y_wi,
                              -(lim_x - lip_w), -lip_gap_z))
                # The upper stretch starts in mid air on the deck as printed:
                # it grows out of the column on a 45 deg underside instead.
                pts = [v(x_col, y_wi - lip_t, lip_gap_z - (col_in + 0.3 - lim_x + lip_w)),
                       v(x_in, y_wi - lip_t, lip_gap_z), v(x_in, y_wi - lip_t, lim_x - lip_w),
                       v(x_col, y_wi - lip_t, lim_x - lip_w)]
                lp.append(Part.Face(Part.makePolygon(pts + [pts[0]])).extrude(v(0, lip_t, 0)))
        for part in [sk, bs, pilot] + lp:
            part.rotate(ORIGIN, Z_AXIS, rot)
        d = d.fuse([sk, bs] + lp).cut(pilot)
    # Room for the OTHER deck's column feet: at the corners where this deck
    # has no column, the other deck's columns land with their 7.0 feet over
    # this skirt and lip (2026-10-20: they would not seat).
    for sx, sy in ((1, -1), (-1, 1)):
        room = _col_foot(sx, sy, False, foot_clear)
        room = room.common(box(-y_wo - 1, y_wo + 1, -y_wo - 1, y_wo + 1, -half_in, 0))
        d = d.cut(room)          # above the plate only: the column lands on it
    d = d.cut(box(*cable_hole, -y_wo - 1, -half_in + 1))   # cables up to the brain
    for sx, sy in ((1, -1), (-1, 1)):            # countersunk holes
        x, y = sx * cxy, sy * cxy
        d = d.cut(cyl_z(x, y, m3_clear / 2.0, -y_wo - 1, -half_in + 1))
        d = d.cut(Part.makeCone(3.2, m3_clear / 2.0, 1.6,
                                v(x, y, -y_wo - 0.001), Z_AXIS))
    return d.removeSplitter()


def _col_foot(sx, sy, top=True, grow=0.0):
    """A widened end of a column: a col_foot_w square, col_foot_h long, a 45
    deg flare toward the column's middle. The free (top) end takes the other
    deck's M3 screw; the one at the base just fills the walls' corner
    notches, so every corner of every wall is closed. `grow` enlarges it all
    round: the walls' notches are this shape grown by foot_clear."""
    W, a0 = col_foot_w + grow, col_w + grow
    c = W - a0
    k = 1 if top else -1                         # built along +z, no mirror
    z_end, z_in = k * (half_in + grow), k * (half_in - col_foot_h - grow)
    sq = box(*sorted((sx * (y_wo - W), sx * y_wo)), *sorted((sy * (y_wo - W), sy * y_wo)),
             *sorted((z_in, z_end)))

    def ring(a, z):
        x_in, y_in = sx * (y_wo - a), sy * (y_wo - a)
        x_out, y_out = sx * y_wo, sy * y_wo
        pts = [v(x_in, y_in, z), v(x_out, y_in, z), v(x_out, y_out, z), v(x_in, y_out, z)]
        return Part.makePolygon(pts + [pts[0]])
    flare = Part.makeLoft([ring(a0, z_in - k * c), ring(W, z_in + k * 0.01)], True)
    return sq.fuse(flare)


def deck_top(deck):
    s = deck.copy()
    s.rotate(ORIGIN, X_AXIS, 180)
    return s


# Gear phases: pinion arbitrary, the swing gears follow it; the output is
# free of both when parked, so it starts at 0 and the sensor gear follows it.
PH_P = 0.0
PH_A = mesh_phase(PH_P, z_pin, S_xz, PA, z_sw)
PH_B = mesh_phase(PH_P, z_pin, S_xz, PB, z_sw)
PH_O = 0.0
PH_Q = mesh_phase(PH_O, g_z, O_xz, Q_xz, g_z)

# ═══════════════════════════════════════════════════════════════════
# ASSEMBLY
# ═══════════════════════════════════════════════════════════════════

# (name, rotation about Z). All four modules identical.
WALLS = [("N", 0), ("W", 90), ("S", 180), ("E", 270)]

MODULE = [("Wall", make_wall(), (0.95, 0.75, 0.30), 0),
          ("Servo", make_servo(), (0.20, 0.35, 0.80), 0),
          ("Horn", make_horn(), (0.95, 0.95, 0.95), 0),
          ("MotorMount", make_mount(), (0.95, 0.60, 0.20), 0),
          ("Pinion", make_pinion(), (0.90, 0.90, 0.90), 0),
          ("Arm", make_arm(), (0.55, 0.40, 0.15), 0),
          ("SwingGearA", make_swing_gear(PA, PH_A), (0.90, 0.80, 0.55), 0),
          ("SwingGearB", make_swing_gear(PB, PH_B), (0.90, 0.80, 0.55), 0),
          ("OutputGear", make_output_gear(), (0.90, 0.90, 0.90), 0),
          ("OutputHub", make_output_hub(), (0.85, 0.45, 0.20), 0),
          ("SensorGear", make_sensor_gear(), (0.80, 0.90, 0.80), 0),
          ("Magnet", make_magnet(), (0.60, 0.60, 0.65), 0),
          ("AS5600", make_as5600(), (0.15, 0.50, 0.25), 0),
          ("Keeper", make_keeper(), (0.95, 0.60, 0.20), 0)]
MODULE += [("Bearing", make_bearing(), (0.70, 0.70, 0.75), 0),
           ("Screw", make_screw(), (0.30, 0.30, 0.30), 0),
           ("ModelKey", make_model_key(), (0.30, 0.70, 0.90), 40)]
SWING = ("Arm", "SwingGearA", "SwingGearB")     # what turns with the arm


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
PARTS.append(("DeckTop", deck_top(DECK), (0.45, 0.55, 0.75), 60))

# The battery lies on whatever of the four modules reaches highest under it.
_foot = box(-battery[0] / 2, battery[0] / 2, -battery[1] / 2, battery[1] / 2, -half_in, half_in)
_under = [sh.common(_foot) for n, sh, c, t in PARTS
          if not n.startswith(("Tech", "Deck")) and sh.BoundBox.intersect(_foot.BoundBox)]
batt_z0 = clr + max(s.BoundBox.ZMax for s in _under if s.Volume > 1e-3)
PARTS.append(("Battery", make_battery(batt_z0), (0.85, 0.20, 0.20), 40))
PARTS.append(("MainBoard", make_main_board(), (0.15, 0.50, 0.25), 40))

DOC_NAME = "Animo_v5"
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
    if _obj.Name.startswith(("Battery", "MainBoard")):
        _label = "Interior (fantasmas)"
    if _obj.Name.startswith("Tech"):
        _label = "LEGO (fantasmas)"
    if _label not in _groups:
        _groups[_label] = doc.addObject("App::DocumentObjectGroup",
                                        "G_" + _label.replace(" ", "_"))
        _groups[_label].Label = _label
    _groups[_label].addObject(_obj)
doc.recompute()
# Only the north wall shows when the macro opens (we are working on the wall,
# 2026-11-12): everything else is hidden, not removed — select a group in the
# tree and press Space to show it again.
SHOW_ONLY = "_N"     # suffix of the parts left visible; None shows everything
if HAS_GUI and getattr(App, "GuiUp", 0) and SHOW_ONLY:
    for _g in _groups.values():
        _vis = False
        for _obj in _g.Group:
            _on = _obj.Name.endswith(SHOW_ONLY)
            _obj.ViewObject.Visibility = _on
            _vis = _vis or _on
        _g.ViewObject.Visibility = _vis
        for _obj in _g.Group:            # showing the group may show them all
            _obj.ViewObject.Visibility = _obj.Name.endswith(SHOW_ONLY)
if HAS_GUI and getattr(App, "GuiUp", 0):
    Gui.SendMsgToActiveView("ViewFit")
    Gui.activeDocument().activeView().viewIsometric()


# ── Printable parts to STL (north module's; orient in the slicer) ───────────
if EXPORT_STL:
    _here = os.path.dirname(os.path.abspath(__file__)) if "__file__" in dir() \
        else App.getUserMacroDir(True)
    _pdir = os.path.join(_here, "stl", "animo_v5", MATERIAL)
    os.makedirs(_pdir, exist_ok=True)
    _by = dict((n, sh) for n, sh, c, t in MODULE)
    for _n in ("Wall", "MotorMount", "Pinion", "Arm", "OutputGear", "OutputHub", "ModelKey",
               "SensorGear", "Keeper"):
        _by[_n].exportStl(os.path.join(_pdir, f"animo_v5_{_n}.stl"))
    _by["SwingGearA"].exportStl(os.path.join(_pdir, "animo_v5_SwingGear_x2.stl"))
    DECK.exportStl(os.path.join(_pdir, "animo_v5_Deck.stl"))

# ═══════════════════════════════════════════════════════════════════
# CHECKS
# ═══════════════════════════════════════════════════════════════════

if RUN_CHECKS is None:
    RUN_CHECKS = not getattr(App, "GuiUp", 0)

# Pairs that are MEANT to touch or nest (prefix match, same wall).
ALLOWED = {("SensorGear", "OutputGear"),  # meshing
           ("SwingGear", "Pinion"),       # always in mesh
           ("SwingGear", "Arm"),          # on its pin
           ("Horn", "Servo"),             # horn on the spline
           ("Horn", "Pinion"),            # horn in its pocket
           ("Pinion", "Servo"),           # spline into the pocket
           ("Arm", "Pinion"),             # on its hub
           ("MotorMount", "Servo"),       # the sleeve, on the tabs
           ("MotorMount", "Wall"),        # on the posts (contact)
           ("Keeper", "Wall"),            # on the posts' step (contact)
           ("AS5600", "Wall"),            # on the posts' ends (contact)
           ("OutputHub", "OutputGear"),   # hex, clamped by the screw
           ("Bearing", "Wall"),           # pressed in its seat
           ("Bearing", "OutputHub"),      # on the journal, the ring on its face
           ("Bearing", "OutputGear"),     # the gear's ring on its inner face
           ("Screw", "OutputHub"),        # threads into the hub
           ("Screw", "OutputGear"),       # through it, head in the boss's countersink
           ("ModelKey", "OutputHub"),     # spigot in its socket, the bump in the groove
           ("TechPin", "Wall"),           # in its hole
           ("TechAxle", "OutputHub")}     # in the cross


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


def _hits(shape, others, by):
    out = []
    for o in others:
        if shape.BoundBox.intersect(by[o].BoundBox):
            vol = shape.common(by[o]).Volume
            if vol > 1e-3:
                out.append((o, vol))
    return out


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
    checks.append(("servo shaft - output", d_so, f"= {n20_d:.2f}", abs(d_so - n20_d) < 1e-6))
    d_q = math.hypot(Q_xz[0], Q_xz[1])
    checks.append(("sensor gear - output centre distance", d_q, f"= {a_mesh:.2f}",
                   abs(d_q - a_mesh) < 1e-6))
    checks.append(("cube side", 2 * y_wo, f"= {cube_side}", abs(2 * y_wo - cube_side) < 1e-6))
    checks.append(("magnet face to chip", y_sb - y_chip, "0.5 - 3", 0.5 <= y_sb - y_chip <= 3))
    for nm, P in (("A", PA), ("B", PB)):
        tips = math.hypot(P[0], P[1]) - (r_sw_p + g_mod) - r_tip
        checks.append((f"swing gear {nm} parked: tip to the output's tips", tips,
                       f"= {tip_free}", abs(tips - tip_free) < 1e-6))
    # Arm at each stop: that gear meshes at the design centre distance, the tab
    # sits on its post, and nothing that turns with the arm hits anything.
    stay = [n for n in names if not (n.endswith("_N") and n.split("_")[0] in SWING)]
    for sgn, nm, P in ((1, "A", PA), (-1, "B", PB)):
        Pe = _pol(S_xz, a_ps, math.atan2(P[1] - S_xz[1], P[0] - S_xz[0]) + sgn * math.radians(arm_T))
        de = math.hypot(Pe[0], Pe[1])
        checks.append((f"arm at +{arm_T:.1f} deg, gear {nm} - output", de,
                       f"= {a_so + mesh_play:.2f}", abs(de - a_so - mesh_play) < 1e-6))
        arm_e = rot_xz(by["Arm_N"], S_xz, sgn * arm_T)
        g = arm_e.distToShape(by["Wall_N"])[0]
        checks.append((f"arm at the {nm} stop: tab on its post", g, "< 0.02", g < 0.02))
        hit = []
        for p in SWING:
            sh = rot_xz(by[f"{p}_N"], S_xz, sgn * arm_T)
            others = [o for o in stay if o not in ("Pinion_N", "OutputGear_N")]
            hit += [f"{p}/{o}" for o, vol in _hits(sh, others, by)]
        checks.append((f"arm at the {nm} stop: clear of everything{': ' + ', '.join(hit) if hit else ''}",
                       len(hit), "0", not hit))
    for a, b, tgt in (("Servo_N", "Servo_E", "> 0, pinwheel neighbours"),
                      ("Servo_N", "Servo_W", "> 0"),
                      ("Servo_N", "Servo_S", "> 0, opposite"),
                      ("Servo_E", "AS5600_N", "> 0, the servo beside the board"),
                      ("Servo_W", "AS5600_N", "> 0"),
                      ("MotorMount_N", "AS5600_N", "> 0"),
                      ("Horn_N", "Wall_N", "> 0"),
                      ("Servo_N", "MainBoard", "> 0"),
                      ("MotorMount_E", "AS5600_N", "> 0"),
                      ("MotorMount_N", "Keeper_N", "> 0"),
                      ("Servo_N", "Arm_N", "> 0"),
                      ("Pinion_N", "MotorMount_N", "> 0"),
                      ("Pinion_N", "OutputGear_N", "> 0"),
                      ("Arm_N", "MotorMount_N", "> 0"),
                      ("Arm_N", "Wall_N", "> 0, parked: tab between the posts"),
                      ("Arm_N", "Keeper_N", "> 0"),
                      ("SwingGearA_N", "OutputGear_N", f"> 0, parked ({tip_free} at the tips)"),
                      ("SwingGearB_N", "OutputGear_N", "> 0, parked"),
                      ("SwingGearA_N", "SwingGearB_N", "> 0"),
                      ("SwingGearA_N", "SensorGear_N", "> 0"),
                      ("SwingGearB_N", "Wall_N", "> 0, the posts"),
                      ("Battery", "DeckTop", "> 0"),
                      ("MainBoard", "AS5600_N", "> 0"),
                      ("MainBoard", "DeckBottom", "> 0")):
        g = gap(a, b)
        checks.append((f"gap {a} / {b}", g, tgt, g > 0.05))
    g = min(gap("Battery", f"{p}_{w}") for p in ("Servo", "MotorMount", "Arm") for w, r in WALLS)
    checks.append(("battery over the motor band", g, "> 0.3", g > 0.3))
    for i, (cx, cz) in enumerate(TECH_XZ):
        n = f"TechPin{i}_N"
        g = min(gap(n, o) for o in names
                if o != n and o != "Wall_N" and not o.startswith("Tech"))
        checks.append((f"{n} at ({cx:+.0f}, {cz:+.0f}): pin clear of everything", g,
                       "> 0.3", g > 0.3))
    checks.append(("column foot longer than the M3 pilot (8)", col_foot_h, "> 8",
                   col_foot_h > 8.0))
    left = col_foot_w - screw_in - m3_pilot / 2.0
    checks.append(("column foot: plastic round the M3 pilot, inner faces", left,
                   ">= 1.0", left >= 1.0))
    floor = y_key0 - y_scr_end
    checks.append((f"M3 x {m3_screw_l:.0f} from inside: tip under the socket's floor", floor,
                   ">= 0.4", floor >= 0.4 - 1e-9))
    bite = y_scr_end - (y_b0 - hex_len)
    checks.append(("M3 thread in the hub", bite, ">= 4", bite >= 4.0 - 1e-9))
    wall_d = journal_d / 2.0 - (key_r + key_grow / 2.0 + groove_d)
    checks.append(("hub wall round the socket's groove", wall_d, ">= 1.5", wall_d >= 1.5 - 1e-9))
    g = gap("ModelKey_N", "Wall_N")
    checks.append(("gap ModelKey_N / Wall_N (turns over the face)", g, "> 0.15", g > 0.15))
    lip = brg_seat / 2.0 - (fl_r + cb_clear)
    checks.append(("bearing's stop: counterbore floor annulus", lip, ">= 0.5", lip >= 0.5))
    for a, b in (("OutputHub_N", "Wall_N"), ("OutputGear_N", "Wall_N"),
                 ("OutputGear_N", "Keeper_N"), ("OutputGear_N", "Arm_N")):
        g = gap(a, b)
        checks.append((f"gap {a} / {b}", g, "> 0.15, turns free", g > 0.15))
    checks.append(("arm tab width", tab_w, "> 3", tab_w > 3.0))
    # A wall comes out without opening the cube: the north module slides out
    # along +Y past everything that stays (decks, their bosses, the others).
    stay_out = [n for n in names if not n.endswith("_N") and not n.startswith("Tech")]
    hits = set()
    for m in [n for n in names if n.endswith("_N") and not n.startswith("Tech")]:
        for dy in range(1, 40, 2):
            sh = by[m].copy()
            sh.translate(v(0, dy, 0))
            for o, vol in _hits(sh, stay_out, by):
                hits.add(f"{m}/{o}")
    checks.append((f"north module slides out 39 mm{': ' + ', '.join(sorted(hits)) if hits else ''}",
                   len(hits), "0 parts in the way", not hits))

    print("=" * 72)
    print("Animo v5 — 72 mm cube, continuous MG90D + double swing gear, 688 output, no LEGO")
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
    print(f"  MATERIAL {MATERIAL}: STLs in stl/animo_v5/{MATERIAL}/")
    print(f"  CUBE side {2 * y_wo:.1f} mm  (walls {wall_t:.1f}, decks {deck_t:.1f})")
    print(f"  OUTPUT: 6700ZZ in the wall (seat Ø{brg_seat}), hub flange Ø{2 * fl_r:.0f} x {fl_t}"
          f" flush, journal Ø{journal_d}, hex {hex_af:.0f} AF x {hex_len:.0f} into the gear,"
          f" M3 x {m3_screw_l:.0f} countersunk down the axis")
    print(f"    model's coupling: trapezoid socket {key_a} / {key_b} x {key_h} (corners r"
          f" {key_r:.2f}), {key_depth} deep in the hub's face, short side up when parked;"
          f" click groove {groove_d} deep at {groove_at};"
          f" M3 x {m3_screw_l:.0f} from inside; ModelKey = the model's template part")
    print(f"  DRIVE: MG90D 360 standing, shaft at {n20_dir:.0f} deg, {n20_d:.1f} off the output; pinion z{z_pin} ->"
          f" swing gears z{z_sw} -> output z{g_z}, m{g_mod}: {g_z / z_pin:.2f}:1")
    print(f"    swing gears at +-{math.degrees(psi_rest):.1f} deg off the N20->output line"
          f" when parked, mesh at +-{math.degrees(psi_mesh):.1f}: arm travel {arm_T:.1f} deg;")
    print(f"    tips touch after {free_T:.1f} deg: park anywhere within +-{free_T:.1f} and the"
          f" output is free both ways")
    print(f"    stops = the mount's posts at +-{stop_ang:.0f} deg off the tab, r {stop_r:.0f};"
          f" tab {tab_w:.2f} wide")
    print(f"    stack: gears y {y_g0:.2f}..{y_g1:.2f}, arm {y_a0:.2f}..{y_a1:.2f},"
          f" mount plate {y_m:.2f}..{y_p1:.2f}, servo base at {y_base:.2f}")
    print(f"  INSIDE: LiPo {battery[0]:.0f}x{battery[1]:.2f}x{battery[2]:.2f}, z {batt_z0:.1f}.."
          f"{batt_z0 + battery[2]:.1f} over the motor band; boards (placeholder {board_box}) under it")
    print(f"  SENSOR: gear z{g_z} 1:1 at {sg_dir:.0f} deg; AS5600 PCB y {y_pcbb:.2f}..{y_pcbf:.2f},"
          f" air {y_sb - y_chip:.2f}")
    print("  PRINTED per wall: Wall, MotorMount, Pinion, Arm, SwingGear x2, OutputGear,"
          " OutputHub, SensorGear, Keeper;  per cube: Deck x2")
    print("  PURCHASED per wall: MG90D 360 continuous (its horn, arms cut to 9), 2x M2 (tabs), 2x M2 (mount),"
          " 2x M2x10 countersunk (wall), 2x M3x8 (AS5600), AS5600 + magnet,"
          " 6700ZZ, M3 countersunk (hub to gear, length printed above);")
    print("           per cube: 4x M3x10 countersunk (no H-bridge)")
    print(f"  FIRMWARE: park = reverse slowly until the AS5600 sees the output stop following,"
          f" then ~{free_T:.0f} deg more by time (window +-{free_T:.1f}); turns without limit")
    print("  OPEN: wiring; USB-C, switch; ports")
    print("=" * 72)
else:
    print(f"Animo v5: geometry only, checks skipped. Cube side {2 * y_wo:.1f} mm")
