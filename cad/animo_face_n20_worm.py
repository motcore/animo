# Animo face — worm N20 lying flat on the wall + double swing gear
# FreeCAD Python Macro. Opens a brick with four of this face; animo_brick.py mixes
# face types. The shared parts live in animo_lib.py.
#
# A worm N20 whose output comes out at 90 deg lies FLAT against its wall,
# upright (motor axis along Z, running down from the shaft): the module is
# much shallower than with a straight motor and the middle of the cube stays
# empty. The worm holds itself, so a swing gear left in mesh holds solidly.
# The cost: a worm is slow or weak — the 6 V 190 rpm one with the 1.8:1 gives
# ~0.75 kg.cm at the output (estimated), under half an MG90S.
# Motor: CNMAWAY 1218-N20 (AliExpress 1005007320463674, 2.82 EUR), from the
# seller's drawing: gearbox 18 x 11.8 x 10, 44.3 overall, shaft D3 x 8 at 3.96
# from the gearbox's far end, 2 x Ø1.5 holes (M1.6) 8.5 apart at 1.70 from it.
# Not measured: + 0.04 once one is here. Its mount has two posts of its own,
# low beside the can (on the stop posts the battery would not fit).
# History: v4c — in git and cad/archive/.
#
# Headless:  "C:/Program Files/FreeCAD 1.0/bin/freecadcmd.exe" animo_face_n20_worm.py

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
from animo_lib import box, cyl_y, bar_xz, make_gear, d_shaft, v   # noqa: E402

# ── Worm N20, output at 90 deg ──────────────────────────────────────────────
n20_w      = 11.8   # mm — gearbox across (X); the can is round, Ø12 with flats
n20_t      = 10.0   # mm — gearbox and can thickness, along the output shaft (Y)
n20_gb_len = 18.0   # mm — gearbox length, along the motor axis (Z)
n20_can_len = 24.3  # mm — 44.3 overall less the gearbox and the cap
n20_cap_len = 2.0
n20_shaft_end = 3.96
n20_shaft_len = 8.0
n20_boss_d = 4.0
n20_boss_h = 0.5
n20_scr_end = 1.70
n20_scr_pitch = 8.5
n20_scr_d  = 1.9
rail_d     = 6.0    # mm — how deep the mount's side rails hold the motor
shaft_fit  = 0.05

# ── Where it sits ───────────────────────────────────────────────────────────
n20_low  = -26.8    # mm — the motor's lowest point: 0.3 over the bottom deck's lip
d_out    = 17.5     # mm — shaft to output: the mount clears the output gear's boss
                    #      (the M3's thread) by 0.3
z_pin    = 10
pin_hub_d = 6.0
sg_dir   = -40.0    # deg — so the AS5600 board clears the motor's rails
stop_r   = None     # no stop posts: they would stand where the battery goes
stop_ang = None
slot_r   = 6.5      # mm — the stop: a pin on the mount in an arc slot of the arm,
                    #      this far from the shaft toward the output
mount_dx = -7.0     # mm — the mount's own posts: this far across from the shaft
                    #      (the lower one clears the deck's wall-screw boss)
mount_dz = (-22.1, -36.1)   # and this far down, beside the can

_sz = n20_low + n20_gb_len - n20_shaft_end + n20_can_len + n20_cap_len
S = (-math.sqrt(d_out ** 2 - _sz ** 2), _sz)
L = lib.Layout(S, z_pin, pin_hub_d, sg_dir, stop_r, stop_ang,
               mount_posts=[(S[0] + mount_dx, S[1] + dz) for dz in mount_dz],
               shaft_tip=lib.y_m + n20_shaft_len, slot_r=slot_r)
y_m = lib.y_m


def make_n20():
    sx, sz = S
    hw = n20_w / 2.0
    yc = y_m - n20_t / 2.0
    z_top = sz + n20_shaft_end
    z_gb0 = z_top - n20_gb_len
    z_c0 = z_gb0 - n20_can_len
    gb = box(sx - hw, sx + hw, y_m - n20_t, y_m, z_gb0, z_top)
    can = Part.makeCylinder(hw, n20_can_len + 0.01, v(sx, yc, z_c0), lib.Z_AXIS).common(
        box(sx - hw - 1, sx + hw + 1, y_m - n20_t, y_m, z_c0 - 1, z_gb0 + 1))
    cap = box(sx - hw + 1.5, sx + hw - 1.5, y_m - n20_t + 1, y_m - 1, z_c0 - n20_cap_len, z_c0 + 0.01)
    boss = cyl_y(sx, sz, n20_boss_d / 2.0, y_m - 0.01, y_m + n20_boss_h)
    shaft = d_shaft(sx, sz, y_m - 0.01, y_m + n20_shaft_len)
    return gb.fuse([can, cap, boss, shaft]).removeSplitter()


def make_pinion():
    g = make_gear(S[0], S[1], L.PH_P, lib.y_g0, lib.y_g1, z=z_pin)
    g = g.fuse(cyl_y(S[0], S[1], pin_hub_d / 2.0, lib.y_p1 + 0.1, lib.y_g0 + 0.01)).removeSplitter()
    return g.cut(d_shaft(S[0], S[1], lib.y_p1 - 1, lib.y_g1 + 1, shaft_fit))


def make_mount():
    """A plate between the gearbox face and the arm, two side rails and an end
    lip that hold the motor flat on it, 2 x M1.6 into the gearbox face, two M2
    into the mount's own wall posts."""
    sx, sz = S
    f, t = lib.sleeve_fit / 2.0, lib.sleeve_t
    hw = n20_w / 2.0 + f
    ow = hw + t
    z_top = sz + n20_shaft_end + f
    z_bot = z_top - n20_gb_len - n20_can_len - lib.sleeve_fit
    m = box(sx - ow, sx + ow, y_m, lib.y_p1, z_bot, z_top + t)
    parts = [box(sx - ow, sx - hw, y_m - rail_d, y_m + 0.01, z_bot, z_top + t),
             box(sx + hw, sx + ow, y_m - rail_d, y_m + 0.01, z_bot, z_top + t),
             box(sx - ow, sx + ow, y_m - rail_d, y_m + 0.01, z_top, z_top + t)]
    for P in L.MPOSTS:
        parts.append(cyl_y(P[0], P[1], lib.stop_d / 2.0 + 1.0, y_m, lib.y_p1))
        parts.append(bar_xz((sx, P[1]), P, lib.stop_d + 2.0, y_m, lib.y_p1))
    pin = L.slot_pin                      # the arm's stop, through its arc slot
    parts.append(cyl_y(pin[0], pin[1], lib.slot_pin_r, lib.y_p1 - 0.01, lib.y_a1 - 0.2))
    m = m.fuse(parts).removeSplitter()
    m = m.cut(cyl_y(sx, sz, n20_boss_d / 2.0 + 0.3, y_m - 1, lib.y_p1 + 1))
    hz = sz + n20_shaft_end - n20_scr_end
    for k in (1, -1):
        hx = sx + k * n20_scr_pitch / 2.0
        m = m.cut(cyl_y(hx, hz, n20_scr_d / 2.0, y_m - 1, lib.y_p1 + 1))
        m = m.cut(Part.makeCone(1.6, 0.0, 1.6, v(hx, lib.y_p1 + 0.001, hz), v(0, -1, 0)))
    for P in L.MPOSTS:
        m = m.cut(cyl_y(P[0], P[1], lib.ws_clear / 2.0, y_m - 1, lib.y_p1 + 1))
    return m


def wall():
    return lib.WallType(
        "n20_worm", L,
        [("N20", make_n20(), (0.75, 0.75, 0.80), 0),
         ("Pinion", make_pinion(), (0.90, 0.90, 0.90), 0),
         ("MotorMount", make_mount(), (0.95, 0.60, 0.20), 0)],
        allowed={("Pinion", "N20"), ("MotorMount", "N20")},
        motor_names=("N20", "MotorMount"),
        printed=("Pinion", "MotorMount"),
        notes=(f"worm N20 lying flat, module {lib.y_wo - (y_m - n20_t):.1f} deep; z10 pinion (1.8:1);"
               " self-locking; DRV8833 per two walls",),
        buy=("CNMAWAY 1218-N20 6V 190 rpm", "2 x M1.6 countersunk (motor)", "2 x M2 (mount)"))


if not getattr(sys, "_animo_importing", False):
    W = wall()
    lib.run_cube({s: W for s, r in lib.SIDES}, "animo_face_n20_worm", here=_HERE,
                 stl_wall=W, stl_folder="animo_face_n20_worm", slide_sides=["N"],
                 title="Animo face n20_worm — four of them on a brick")
