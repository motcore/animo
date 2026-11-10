# The tall yaw duck on curved soles (yaw_duck.py with sole_r), as equations:
# a body rolling side to side on an arc, pushed by the servo's turn.
#
# Seen from the front the two soles are one arc of radius rho, its centre
# rho above the floor; the centre of mass sits d = rho - h below it (h: its
# height). Rolling without slipping through theta, the centre of mass is at
#     x = rho theta - d sin(theta),   z = rho - d cos(theta)
# so (Lagrange, I_G: roll inertia about the centre of mass)
#     (I_G + M (rho^2 + d^2 - 2 rho d cos theta)) theta''
#         + M rho d sin(theta) theta'^2 + M g d sin(theta) = tau - c theta'
# Small: w^2 = g d / (k^2 + h^2), k^2 = I_G / M: the rhythm set by the arc.
# No landings: it rolls, so the loss is small, a damping c fitted once on
# the simulator's free rocking (fit()).
# The push, as with point feet (yaw_model.py, sign fixed 04/11): the body
# (m_b, its centre of mass y_b ahead of the servo's axis, h_b up) turns by
# psi = -beta on the held bar, so its centre of mass goes dx = -y_b sin(psi)
# to the side:
#     tau = m_b (g dx - h_b dx'')
# The servo: ideal on its rate-limited command, switched by the tilt as
# yaw_duck.py (or, open loop, a given beta(t)).
#
#   python yaw_rock_model.py        free rocking, fit, then the checks

import math
import multiprocessing as mp
import sys

import mujoco
import numpy as np

from yaw_duck import YAW, build, release, summary

TALL = dict(hip=130, foot_y=10, sole_r=200, turn_deg=3, slide_mm_deg=5.0, dir=-1.0)
DT = 1e-4


def consts(p):
    """Mass, centre of mass height and roll inertia from the MuJoCo model
    itself (standing upright, the servo at 0), and the push's pieces."""
    m, d = build(p)
    mujoco.mj_forward(m, d)
    M = float(m.body_subtreemass[1])
    com = d.subtree_com[1].copy()
    # roll inertia about Y through the centre of mass: every body's own
    # (rotated to the world) plus m r^2
    I = 0.0
    for i in range(1, m.nbody):
        mi = m.body_mass[i]
        R = d.ximat[i].reshape(3, 3)
        Ib = R @ np.diag(m.body_inertia[i]) @ R.T
        r = d.xipos[i] - com
        I += Ib[1, 1] + mi * (r[0] ** 2 + r[2] ** 2)
    floor = com[2] - d.qpos[2] + p["hip"] / 1000           # body origin at hip
    h = com[2] - (d.qpos[2] - p["hip"] / 1000)
    mb = (p["cube_g"] + p["brain_g"]) / 1000
    return dict(M=M, h=h, I=I, rho=p["sole_r"] / 1000, g=9.81, mb=mb,
                yb=p["brain_g"] / 1000 * p["brain_y"] / 1000 / mb, hb=p["hip"] / 1000)


def push(k, psi, psid, psidd):
    dx = -k["yb"] * math.sin(psi)
    dxdd = -k["yb"] * (psidd * math.cos(psi) - psid * psid * math.sin(psi))
    return k["mb"] * (k["g"] * dx - k["hb"] * dxdd)


def run(p, k, c, th0=math.radians(8), secs=12.0, beta_of_t=None, servo=True):
    rho, M, g, I = k["rho"], k["M"], k["g"], k["I"]
    dd = rho - k["h"]
    T, dead, vmax = math.radians(p["turn_deg"]), math.radians(p["dead_deg"]), p["servo_speed"]
    x, v, beta, bd, cmd = th0, 0.0, 0.0, 0.0, 0.0
    out = []
    n = int(secs / DT)
    for i in range(n):
        t = i * DT
        if beta_of_t is not None:            # open loop: the simulator's servo
            b_new = beta_of_t(t)
        else:
            want = cmd
            if servo and x > dead:
                want = p["dir"] * T
            elif servo and x < -dead:
                want = -p["dir"] * T
            cmd += max(-vmax * DT, min(vmax * DT, want - cmd))
            b_new = cmd
        bd_new = (b_new - beta) / DT
        bdd, bd, beta = (bd_new - bd) / DT, bd_new, b_new
        Ieff = I + M * (rho * rho + dd * dd - 2 * rho * dd * math.cos(x))
        acc = (push(k, -beta, -bd, -bdd) - c * v - M * rho * dd * math.sin(x) * v * v
               - M * g * dd * math.sin(x)) / Ieff
        v += acc * DT
        x += v * DT
        if abs(x) > 0.7:
            return dict(fell=True, t=t, out=np.array(out))
        if i % 20 == 0:
            out.append((t, x, beta))
    return dict(fell=False, out=np.array(out))


def stats(t, th, t0):
    sel = t > t0
    th = np.degrees(th)
    z = np.where(np.sign(th[:-1]) != np.sign(th[1:]))[0]
    z = z[t[z] > t0]
    f = 1 / (2 * np.median(np.diff(t[z]))) if len(z) > 3 else 0.0
    return float(np.abs(th[sel]).max()), f


def peaks(t, th, n=6):
    th = np.degrees(th)
    z = np.where(np.sign(th[:-1]) != np.sign(th[1:]))[0]
    return [round(float(np.abs(th[a:b]).max()), 2) for a, b in zip(z, z[1:])][:n]


def sim_free(p):
    a = release(dict(p, turn_deg=0.0, slide_mm_deg=0.0, secs=6.0), math.radians(8))
    return a[:, 0], a[:, 1]


def fit(p, k):
    """The rolling loss: the damping c that makes the free decay match."""
    t, th = sim_free(p)
    ps = peaks(t, th, 8)
    best = None
    for c in np.geomspace(1e-5, 3e-2, 40):
        e = run(dict(p, turn_deg=0.0), k, c, secs=6.0, servo=False)["out"]
        pe = peaks(e[:, 0], e[:, 1], 8)
        n = min(len(ps), len(pe))
        if n < 4:
            continue
        err = np.sum((np.log(np.maximum(ps[:n], 1e-3)) - np.log(np.maximum(pe[:n], 1e-3))) ** 2)
        if best is None or err < best[0]:
            best = (err, c)
    return best[1], ps, t, th


def job_free(rho):
    p = dict(YAW, **dict(TALL, sole_r=rho))
    k = consts(p)
    t, th = sim_free(p)
    _, fs = stats(t, th, 0.3)
    e = run(dict(p, turn_deg=0.0), k, C_FIT, secs=6.0, servo=False)["out"]
    _, fe = stats(e[:, 0], e[:, 1], 0.3)
    dd = rho / 1000 - k["h"]
    ff = math.sqrt(k["g"] * dd / (k["I"] / k["M"] + k["h"] ** 2)) / (2 * math.pi)
    return rho, fs, fe, ff, peaks(t, th, 5), peaks(e[:, 0], e[:, 1], 5)


def job_open(over):
    p = dict(YAW, **dict(TALL, **over))
    k = consts(p)
    a = release(p)
    t, th, servo = a[:, 0], a[:, 1], a[:, 6]
    e = run(p, k, C_FIT, beta_of_t=lambda tt: float(np.interp(tt, t, servo)))
    o = e["out"]
    return over, stats(t, th, 6), (stats(o[:, 0], o[:, 1], 6) if not e["fell"] else None)


def job_closed(over):
    p = dict(YAW, **dict(TALL, **over))
    k = consts(p)
    a = release(p)
    s = summary(p, a)
    sim = None if s["fell"] else stats(a[:, 0], a[:, 1], 9)
    e = run(p, k, C_FIT)
    eq = None if e["fell"] else stats(e["out"][:, 0], e["out"][:, 1], 9)
    return over, sim, eq


C_FIT = None


def _init(c):                  # each worker gets the fitted damping
    global C_FIT
    C_FIT = c

if __name__ == "__main__":
    p0 = dict(YAW, **TALL)
    k0 = consts(p0)
    print(f"tall duck, R 200: M {k0['M'] * 1000:.0f} g, centre of mass {k0['h'] * 1000:.1f} mm up, "
          f"I_G {k0['I'] * 1e6:.0f} e-6 kg m2, y_b {k0['yb'] * 1000:.1f} mm")
    C_FIT, ps, _, _ = fit(p0, k0)
    print(f"fitted rolling damping c = {C_FIT:.2e} N m s/rad (free peaks in the sim {ps[:6]})")
    with mp.Pool(initializer=_init, initargs=(C_FIT,)) as pool:
        print("1. free rocking, released at 8 deg: rhythm sim | equations | small-angle formula; peaks")
        for rho, fs, fe, ff, pss, pee in pool.map(job_free, (150, 200, 250)):
            print(f"   R {rho}: {fs:4.2f} | {fe:4.2f} | {ff:4.2f} Hz   sim {pss}  eq {pee}")
        print("2. the equation fed the simulator's servo (rocking at 6-12 s, rhythm): sim | equation")
        for over, s, e in pool.map(job_open, (dict(), dict(turn_deg=2.5), dict(turn_deg=3.5), dict(sole_r=180), dict(sole_r=220))):
            es = "FELL" if e is None else f"+-{e[0]:4.1f} {e[1]:4.2f} Hz"
            print(f"   {str(over) or 'base':22s} +-{s[0]:4.1f} {s[1]:4.2f} Hz | {es}")
        print("3. equations alone, with the control (rocking at 9-12 s): sim | equations")
        jobs = [dict(turn_deg=T) for T in (2.0, 2.5, 3.0, 3.5, 5.0)] + [dict(sole_r=r) for r in (180, 220)] + [dict(dir=1.0)]
        for over, s, e in pool.map(job_closed, jobs):
            ss = "FELL" if s is None else f"+-{s[0]:4.1f} {s[1]:4.2f} Hz"
            es = "FELL" if e is None else f"+-{e[0]:4.1f} {e[1]:4.2f} Hz"
            print(f"   {str(over):22s} {ss:16s} | {es}")
