# The open-loop yaw duck as a pushed swing (user, 04/11): the turn follows a
# fixed clock, the rocking answers it like a driven oscillator, and how far
# behind the clock it rocks decides where the legs' push falls.
#
# The rocking: yaw_profile_model.run with the clock (any sole). Its lag:
# the clock's fundamental, theta ~ A cos(2 pi (c - c_pk)), c the clock's
# cycles; the right foot is down around c_pk. The right leg pushes back for
# the half cycle centred at c = 1/4 + slide_phase - clock_phase
# (yaw_duck.leg_profile). Two half-cycle windows offset by D cycles overlap
# 1 - 4|D| (from +1 all in step to -1 all backward), so
#     v ~ 4 A_step f (1 - 4 |D|),   D = c_pk - (1/4 + slide_phase - clock_phase)
# A pushed swing lags its push by little below its own rhythm f0 and by
# about half a cycle above it: that sweep of D with f/f0 is what turns the
# walk forward or backward, and why the clock wants to sit just above f0.
#
#   python yaw_clock_model.py        equations vs yaw_duck.py, the clock swept

import math
import multiprocessing as mp

import numpy as np

from yaw_duck import YAW, release, summary
from yaw_rock_model import consts
import yaw_profile_model as pm

BEST = dict(hip=130, foot_y=10, dir=-1.0, turn_deg=3, servo_kp=300.0, servo_damp=0.02, servo_speed=1.0,
            legs_start_s=2.0, secs=14.0, slide_per_leg=1, slide_speed=1000,
            sole_r=250, sole_in=12.5, clock_hz=1.65, slide_amp=25, slide_phase=0.0)


def f0(p):
    k = consts(p)
    d = p["sole_r"] / 1000 - k["h"]
    return math.sqrt(k["g"] * d / (k["I"] / k["M"] + k["h"] ** 2)) / (2 * math.pi)


def lag(t, th, p, t0):
    """Amplitude (deg) and the clock cycle (0..1) where the rocking peaks
    on the right foot, from the steady part."""
    sel = t > t0
    c = t[sel] * p["clock_hz"] + p["clock_phase"]
    z = np.sum(th[sel] * np.exp(-2j * np.pi * c)) / sel.sum()
    return 2 * math.degrees(abs(z)), (-np.angle(z) / (2 * np.pi)) % 1.0


def overlap(c_pk, p):
    D = c_pk - (0.25 + p["slide_phase"] - p["clock_phase"])
    D = (D + 0.5) % 1.0 - 0.5
    return 1 - 4 * abs(D)


def job(f):
    p = dict(YAW, **dict(BEST, clock_hz=f, slide_amp=25 * 1.65 / f))
    a = release(p, 0.0)
    s = summary(p, a)
    A_s, c_s = lag(a[:, 0], a[:, 1], p, 6)
    e = pm.run(p, consts(p), th0=1e-3, secs=14.0)
    if e["fell"]:
        return f, s, (A_s, c_s), None
    o = e["out"]
    A_e, c_e = lag(o[:, 0], o[:, 1], p, 6)
    return f, s, (A_s, c_s), (A_e, c_e)


if __name__ == "__main__":
    p0 = dict(YAW, **BEST)
    print(f"R 250, cube at 130 mm: f0 = {f0(p0):.2f} Hz; legs at the same speed (step 25 x 1.65 / f)")
    print("clock  f/f0 | rocking peak at cycle: sim  eq | rocking: sim  eq | speed cm/s: sim  from eq's lag  from sim's lag")
    with mp.Pool() as pool:
        for f, s, (A_s, c_s), eq in pool.map(job, (1.2, 1.35, 1.5, 1.6, 1.65, 1.7, 1.8, 1.9)):
            p = dict(p0, clock_hz=f, slide_amp=25 * 1.65 / f)
            v_full = 4 * p["slide_amp"] * f / 10
            vs = s["fwd"] / 120 if not s["fell"] else float("nan")
            if eq is None:
                print(f"  {f:4.2f} {f / f0(p0):4.2f} | {c_s:4.2f}  FELL")
                continue
            A_e, c_e = eq
            print(f"  {f:4.2f} {f / f0(p0):4.2f} | {c_s:4.2f} {c_e:4.2f} | +-{A_s:4.1f} +-{A_e:4.1f} | "
                  f"{vs:5.1f}  {v_full * overlap(c_e, p):5.1f}  {v_full * overlap(c_s, p):5.1f}")
