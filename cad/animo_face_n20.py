# Animo face — straight N20 gearmotor + double swing gear (the one being
# built, user 2026-11-14: more torque than the worm, a 40 board, 1.5 between
# faces)
# FreeCAD Python Macro. Opens a brick with four of this face; animo_brick.py mixes
# face types. The shared parts live in animo_lib.py.
#
# The N20 points inward from its wall (shaft toward the wall), up and to the
# side of the output; a z10 pinion on its D shaft gives 1.8:1 to the output.
# Its mount: a plate between the gearbox face and the arm, a sleeve round the
# gearbox, 2 x M1.6 countersunk into the gearbox face, screwed to the two stop
# posts. Driven by a DRV8833 (one per two walls), from the 5 V rail.
# Reference motor: Pololu #3065, 6V HPCB 100:1 (330 rpm, 1.6 kg.cm stall
# extrapolated at 6 V); a generic "N20 6V 300 rpm" for ~3 EUR. Sizes from
# Pololu, not measured: + 0.04 once one is here.
# History: v4 (first swing-gear cube) — in git and cad/archive/.
#
# Headless:  "C:/Program Files/FreeCAD 1.0/bin/freecadcmd.exe" animo_face_n20.py

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

# ── N20 (Pololu micro metal gearmotor sizes) ────────────────────────────────
n20_w      = 10.0   # mm — gearbox and can across the flats (along X)
n20_h      = 12.0   # mm — gearbox height, the can's round (Z)
n20_gb_len = 9.0
n20_can_len = 15.0
n20_cap_len = 2.0
n20_shaft_len = 9.0 # mm — D shaft out of the gearbox face (Ø3, flat 2.5)
n20_boss_d = 4.0
n20_boss_h = 0.5
n20_scr_pitch = 9.0 # mm — two M1.6 holes in the gearbox face, along Z
n20_scr_d  = 1.9
sleeve_len = 8.0    # mm — sleeve along the gearbox (9 long)
shaft_fit  = 0.05   # mm — pinion's D bore over the shaft (PETG press)

# ── Where it sits ───────────────────────────────────────────────────────────
n20_dir  = 150.0    # deg — up and to the side, away from the sensor and the boards
d_out    = 15.9     # mm — shaft to output: the least arm travel for a z10 pinion
z_pin    = 10
pin_hub_d = 6.0
sg_dir   = -60.0
stop_r   = 11.0
stop_ang = 45.0

S = (d_out * math.cos(math.radians(n20_dir)), d_out * math.sin(math.radians(n20_dir)))
L = lib.Layout(S, z_pin, pin_hub_d, sg_dir, stop_r, stop_ang,
               shaft_tip=lib.y_m + n20_shaft_len)
y_m = lib.y_m


def make_n20():
    sx, sz = S
    hw, hh = n20_w / 2.0, n20_h / 2.0
    gb = box(sx - hw, sx + hw, y_m - n20_gb_len, y_m, sz - hh, sz + hh)
    y_c1 = y_m - n20_gb_len
    y_c0 = y_c1 - n20_can_len
    can = cyl_y(sx, sz, hh, y_c0, y_c1 + 0.01).common(
        box(sx - hw, sx + hw, y_c0 - 1, y_c1 + 1, sz - hh - 1, sz + hh + 1))
    cap = box(sx - hw + 1, sx + hw - 1, y_c0 - n20_cap_len, y_c0 + 0.01, sz - hh + 1.5, sz + hh - 1.5)
    boss = cyl_y(sx, sz, n20_boss_d / 2.0, y_m - 0.01, y_m + n20_boss_h)
    shaft = d_shaft(sx, sz, y_m - 0.01, y_m + n20_shaft_len)
    return gb.fuse([can, cap, boss, shaft]).removeSplitter()


def make_pinion():
    g = make_gear(S[0], S[1], L.PH_P, lib.y_g0, lib.y_g1, z=z_pin)
    g = g.fuse(cyl_y(S[0], S[1], pin_hub_d / 2.0, lib.y_p1 + 0.1, lib.y_g0 + 0.01)).removeSplitter()
    return g.cut(d_shaft(S[0], S[1], lib.y_p1 - 1, lib.y_g1 + 1, shaft_fit))


def make_mount():
    sx, sz = S
    f, t = lib.sleeve_fit / 2.0, lib.sleeve_t
    hw, hh = n20_w / 2.0 + f, n20_h / 2.0 + f
    m = box(sx - hw - t, sx + hw + t, y_m - sleeve_len, lib.y_p1, sz - hh - t, sz + hh + t)
    parts = []
    for P in L.MPOSTS:
        parts.append(cyl_y(P[0], P[1], lib.stop_d / 2.0 + 1.0, y_m, lib.y_p1))
        parts.append(bar_xz(S, P, lib.stop_d + 2.0, y_m, lib.y_p1))
    m = m.fuse(parts).removeSplitter()
    m = m.cut(box(sx - hw, sx + hw, y_m - sleeve_len - 1, y_m, sz - hh, sz + hh))
    m = m.cut(cyl_y(sx, sz, n20_boss_d / 2.0 + 0.3, y_m - 1, lib.y_p1 + 1))
    for k in (1, -1):
        hz = sz + k * n20_scr_pitch / 2.0
        m = m.cut(cyl_y(sx, hz, n20_scr_d / 2.0, y_m - 1, lib.y_p1 + 1))
        m = m.cut(Part.makeCone(1.6, 0.0, 1.6, v(sx, lib.y_p1 + 0.001, hz), v(0, -1, 0)))
    for P in L.MPOSTS:
        m = m.cut(cyl_y(P[0], P[1], lib.ws_clear / 2.0, y_m - 1, lib.y_p1 + 1))
    return m


def wall():
    return lib.WallType(
        "n20", L,
        [("N20", make_n20(), (0.75, 0.75, 0.80), 0),
         ("Pinion", make_pinion(), (0.90, 0.90, 0.90), 0),
         ("MotorMount", make_mount(), (0.95, 0.60, 0.20), 0)],
        allowed={("Pinion", "N20"), ("MotorMount", "N20")},
        motor_names=("N20", "MotorMount"),
        printed=("Pinion", "MotorMount"),
        notes=("straight N20 pointing inward, z10 pinion (1.8:1); DRV8833 per two walls",),
        buy=("N20 6V ~100:1", "2 x M1.6 countersunk (N20)", "2 x M2 (mount)"))


if not getattr(sys, "_animo_importing", False):
    W = wall()
    lib.run_cube({s: W for s, r in lib.SIDES}, "animo_face_n20", here=_HERE,
                 stl_wall=W, stl_folder="animo_face_n20", slide_sides=["N"],
                 title="Animo face n20 — four of them on a brick")
