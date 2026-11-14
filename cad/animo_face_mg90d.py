# Animo face — continuous MG90D + double swing gear
# SET ASIDE (user, 2026-11-14): the next face's servo and its sleeve pass 0.5-1.7
# behind this face's AS5600, no room for its wires (it was 4.1 with a 3.0 wall;
# the bearing's 5.6 wall took it). The straight N20 face is the one being built.
# FreeCAD Python Macro. Opens a brick with four of this face; animo_brick.py mixes
# face types. What every wall shares (plate, bearing output, trapezoid
# coupling, swing gears, sensor, frame) lives in animo_lib.py.
#
# The servo stands (case length along Z) under the output, its shaft toward
# the wall; the servo gear becomes the PINION: z18 holding the servo's own
# horn (arms cut to 9), 1:1 to the output, endless rotation (the user's ~8
# continuous MG90D). The servo hangs from a SLEEVE over its gear tower and
# the case's top, screwed to its tabs from below and to the two stop posts.
# Park: reverse slowly until the AS5600 sees the output stop following, then
# a few degrees more by time. Stopped, the servo's gearbox holds: a swing gear
# left in mesh holds its way. No H-bridge: PWM straight from the C3.
# History: v3 (servo gear 1:1), v4b/v4b1 (swing gear), v5 (bearing,
# coupling, no LEGO) — in git and cad/archive/.
#
# Headless:  "C:/Program Files/FreeCAD 1.0/bin/freecadcmd.exe" animo_face_mg90d.py

import importlib
import math
import os
import sys

import FreeCAD as App
import Part

_HERE = os.path.dirname(os.path.abspath(
    globals().get("__file__") or os.path.join(App.getUserMacroDir(True), "_")))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import animo_lib as lib          # noqa: E402
importlib.reload(lib)
from animo_lib import box, cyl_y, bar_xz, make_gear   # noqa: E402

# ── Servo MG90D (datasheet; horn measured 2026-10-15/20, +0.04) ─────────────
sv_case_len  = 22.4   # mm — case length (here along Z)
sv_case_t    = 12.5   # mm — case thickness (along X)
sv_case_h    = 22.4   # mm — base to the top of the main case
sv_tower_h   = 28.2   # mm — base to the top of the gear tower
sv_tower_len = 12.2   # mm — the tower's length, at the shaft's end
sv_spline_h  = 4.0
sv_spline_d  = 4.5
sv_shaft_end = 6.25   # mm — shaft axis from the case's end
sv_tab_z0    = 18.0   # mm — base to the underside of the tab plate
sv_tab_t     = 2.5
sv_tab_out   = 4.7
sv_tab_hole  = 2.35
sv_tab_hole_d = 2.0
horn_hub_d   = 6.82
horn_hub_h   = 4.69
horn_arm_t   = 2.08
horn_arm_w   = 5.13
horn_arm_len = 9.0    # mm — tip to tip, CUT to this
horn_lift    = 1.65   # mm — the hub's underside above the tower
horn_grow    = 0.2    # mm — pocket over the arms
horn_hub_grow = 0.25  # mm — pocket over its hub: a press (0.2 in PLA, tested 2026-10-17)

# ── Where it sits ───────────────────────────────────────────────────────────
x_sv     = -12.47   # mm — shaft's X: servo and sleeve clear of the AS5600 board
d_out    = 17.9     # mm — shaft to output: the least arm travel for a z18 pinion
z_pin    = 18
pin_hub_d = 9.0     # mm — pinion's hub round the horn's hub: the arm's bearing
sg_dir   = 30.0     # deg — sensor gear up and to the right (the lower swing gear
                    #      sits where v3 had it)
stop_r   = 10.5     # mm — stop posts: clear of the z18 pinion's tips by 0.5
stop_ang = 45.0

S = (x_sv, -math.sqrt(d_out ** 2 - x_sv ** 2))   # under the output, case standing up
L = lib.Layout(S, z_pin, pin_hub_d, sg_dir, stop_r, stop_ang)

y_tower = lib.y_m
y_base  = y_tower - sv_tower_h
y_tab0  = y_base + sv_tab_z0
y_tab1  = y_tab0 + sv_tab_t
sv_x0, sv_x1 = x_sv - sv_case_t / 2.0, x_sv + sv_case_t / 2.0
sv_z    = S[1]
case_z0 = sv_z - sv_shaft_end
case_z1 = case_z0 + sv_case_len
tab_z0  = case_z0 - sv_tab_out
tab_z1  = case_z1 + sv_tab_out
tab_hole_z = (tab_z0 + sv_tab_hole, tab_z1 - sv_tab_hole)
y_horn  = y_tower + horn_lift + horn_hub_h
assert y_horn + horn_grow < lib.y_g1 - 0.5, "horn pokes out of the pinion"


def make_servo():
    case = box(sv_x0, sv_x1, y_base, y_base + sv_case_h, case_z0, case_z1)
    tower = box(sv_x0, sv_x1, y_base + sv_case_h - 0.01, y_tower, case_z0, case_z0 + sv_tower_len)
    tabs = box(sv_x0, sv_x1, y_tab0, y_tab1, tab_z0, tab_z1)
    for hz in tab_hole_z:
        tabs = tabs.cut(cyl_y(x_sv, hz, sv_tab_hole_d / 2.0, y_tab0 - 1, y_tab1 + 1))
    spline = cyl_y(x_sv, sv_z, sv_spline_d / 2.0, y_tower - 0.01, y_tower + sv_spline_h)
    return case.fuse([tower, tabs, spline]).removeSplitter()


def _horn(grow):
    y0 = y_horn - horn_hub_h
    hub_g = grow if grow == 0.0 else horn_hub_grow
    hub = cyl_y(x_sv, sv_z, horn_hub_d / 2.0 + hub_g, y0 - grow, y_horn + grow)
    half = horn_arm_len / 2.0 + grow
    arms = box(x_sv - half, x_sv + half, y_horn - horn_arm_t - grow, y_horn + grow,
               sv_z - horn_arm_w / 2.0 - grow, sv_z + horn_arm_w / 2.0 + grow)
    return hub.fuse(arms)


def make_horn():
    return _horn(0.0).removeSplitter().cut(
        cyl_y(x_sv, sv_z, sv_spline_d / 2.0, y_tower - 1, y_horn - 1.0))


def make_pinion():
    """The servo gear as the pinion: z18 teeth in the slab, a hub behind (the
    arm's bearing), the horn's pocket from below, a floor over the horn."""
    g = make_gear(S[0], S[1], L.PH_P, lib.y_g0, lib.y_g1, z=z_pin)
    g = g.fuse(cyl_y(S[0], S[1], pin_hub_d / 2.0, lib.y_p1 + 0.1, lib.y_g0 + 0.01)).removeSplitter()
    g = g.cut(_horn(horn_grow))
    return g.cut(cyl_y(x_sv, sv_z, horn_hub_d / 2.0 + horn_hub_grow, lib.y_p1 - 1, y_horn))


def make_mount():
    """A plate over the tower top (under the arm), a sleeve down over the tower
    and the case's top to the tabs, 2 x M2 up through the tabs, two webs to
    the stop posts (M2 from below). Trimmed where a neighbouring servo passes."""
    f = lib.sleeve_fit / 2.0
    t = lib.sleeve_t
    m = box(sv_x0 - f - t, sv_x1 + f + t, y_tab1, lib.y_p1, tab_z0, tab_z1)
    parts = []
    for P in L.MPOSTS:
        parts.append(cyl_y(P[0], P[1], lib.stop_d / 2.0 + 1.0, lib.y_m, lib.y_p1))
        parts.append(bar_xz(S, P, lib.stop_d + 2.0, lib.y_m, lib.y_p1))
    m = m.fuse(parts).removeSplitter()
    m = m.cut(box(sv_x0 - f, sv_x1 + f, y_tab1 - 1, y_base + sv_case_h + f, case_z0 - f, case_z1 + f))
    m = m.cut(box(sv_x0 - f, sv_x1 + f, y_tab1 - 1, y_tower + f, case_z0 - f,
                  case_z0 + sv_tower_len + f))
    m = m.cut(cyl_y(x_sv, sv_z, sv_spline_d / 2.0 + 0.6, lib.y_m - 1, lib.y_p1 + 1))
    for hz in tab_hole_z:
        m = m.cut(cyl_y(x_sv, hz, lib.m2_pilot / 2.0, y_tab1 - 1, y_tab1 + 5.0))
    for P in L.MPOSTS:
        m = m.cut(cyl_y(P[0], P[1], lib.ws_clear / 2.0, lib.y_m - 1, lib.y_p1 + 1))
    # the next wall's servo base passes here (the pinwheel): trim the sleeve
    return m.cut(box(y_base - 0.5, lib.y_wo, -lib.y_wo, -sv_x0 + 0.5, -lib.y_wo, lib.y_wo))


def wall():
    return lib.WallType(
        "mg90d", L,
        [("Servo", make_servo(), (0.20, 0.35, 0.80), 0),
         ("Horn", make_horn(), (0.95, 0.95, 0.95), 0),
         ("Pinion", make_pinion(), (0.90, 0.90, 0.90), 0),
         ("MotorMount", make_mount(), (0.95, 0.60, 0.20), 0)],
        allowed={("Horn", "Servo"), ("Horn", "Pinion"), ("Pinion", "Servo"),
                 ("MotorMount", "Servo")},
        motor_names=("Servo", "MotorMount"),
        printed=("Pinion", "MotorMount"),
        notes=("continuous MG90D standing, horn (arms cut to 9) in the z18 pinion; sleeve on the"
               " tabs; park with the AS5600; no H-bridge",),
        buy=("MG90D continuous", "2 x M2 (tabs)", "2 x M2 (mount)"))


if not getattr(sys, "_animo_importing", False):
    W = wall()
    lib.run_cube({s: W for s, r in lib.SIDES}, "animo_face_mg90d", here=_HERE,
                 stl_wall=W, stl_folder="animo_face_mg90d", slide_sides=["N"],
                 title="Animo face mg90d — four of them on a brick")
