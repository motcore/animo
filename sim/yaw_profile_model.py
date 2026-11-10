# The yaw duck rocking on ANY sole seen from the front (user, 04/11): the
# equations built from the sole's drawing, the same points as the
# simulator's mesh (yaw_duck.sole_profile), or two point feet.
#
# Body frame, from the front: the centre of mass G at (0, h) above the
# sole's lowest point; the sole a set of points s = (x_s, z_s) (both feet).
# Tilted by theta, the point touching the floor is the one that maximises
#     H(theta) = x_s sin(theta) + (h - z_s) cos(theta)
# and H is the centre of mass's height. Rolling on it without slipping
# (Lagrange, J = I_G + M |G - C|^2 about the contact C):
#     J theta'' + 1/2 J'(theta) theta'^2 + M g H'(theta) = tau - c theta'
#     H'(theta) = x_s cos(theta) - (h - z_s) sin(theta)       (C fixed)
# A smooth arc gives back yaw_rock_model.py; two points give Housner's block.
# When the contact jumps (across the gap between the feet, onto an edge,
# from one point foot to the other) it is a landing: the angular momentum
# about the new contact C2 is kept,
#     theta'+ = theta'- (I_G + M (G - C2).(G - C1)) / (I_G + M |G - C2|^2)
# The push of the servo's turn as before: tau = m_b (g dx - h_b dx''),
# dx = -y_b sin(psi), psi = -beta. The rolling loss c: yaw_rock_model's fit.
#
#   python yaw_profile_model.py        the checks against yaw_duck.py

import math
import multiprocessing as mp
import sys

import numpy as np

import mujoco

from yaw_duck import YAW, build, leg_profile, release, sole_profile
from yaw_rock_model import consts, push, stats, peaks

DT = 1e-4
C_ROLL = 1.12e-3       # N m s/rad — rolling loss (fitted on R 200, yaw_rock_model)
JUMP = 0.003           # m — a contact moving farther than this in a step: a landing


def profile(p):
    """The sole from the front, both feet: x (m, right +), z (m, up)."""
    if p["sole_r"]:
        xa, za = sole_profile(p)
        x = np.r_[-np.asarray(xa)[::-1], xa] / 1000
        z = np.r_[np.asarray(za)[::-1], za] / 1000
    else:                                   # point feet: the balls' bottoms
        b = p["b"] / 1000
        x, z = np.array([-b, b]), np.zeros(2)
    return x, z


def yaw_consts(p):
    """Yaw inertias from the MuJoCo model: the body (cube + brain) about the
    servo's axis, and the whole duck about its centre of mass."""
    m, d = build(p)
    mujoco.mj_forward(m, d)
    com = d.subtree_com[1].copy()
    Iz, Jb = 0.0, 0.0
    axis = d.xpos[m.body("bar").id].copy()
    for i in range(1, m.nbody):
        R = d.ximat[i].reshape(3, 3)
        Ib = (R @ np.diag(m.body_inertia[i]) @ R.T)[2, 2]
        r = d.xipos[i] - com
        Iz += Ib + m.body_mass[i] * (r[0] ** 2 + r[1] ** 2)
        if i == 1:                     # the torso body: cube and brain
            ra = d.xipos[i] - axis
            Jb = Ib + m.body_mass[i] * (ra[0] ** 2 + ra[1] ** 2)
    return dict(Iz=Iz, Jb=Jb)


def run(p, k, c=C_ROLL, th0=math.radians(8), secs=12.0, beta_of_t=None, servo=True, legs=False):
    """legs: the legs' push too (yaw_duck slide_per_leg on the clock). The
    whole duck may turn on its planted foot (psi_f): its yaw momentum about
    the foot's contact P is p = I_P psi_f' + G, with G the internal motions'
    share (the body turning on the bar, -J_b beta'; the body sliding forward
    over the foot from the contact's side, (M - m_l) x_P s'_planted; the
    raised leg, m_l (x_sw - x_P) s'_raised). The foot's friction holds it up
    to mu M g L / 4 (a line of length L); past that it turns. Turning, the
    centre of mass (ahead of P by y_G - y_P) swings sideways: another push,
    M (g dx_f - h dx_f''), dx_f = -(y_G - y_P) sin psi_f."""
    xs, zs = profile(p)
    M, g, h, IG = k["M"], k["g"], k["h"], k["I"]
    if legs:
        ky = yaw_consts(p)
        ml = p["foot_g"] / 1000
        A = p["slide_amp"] / 1000
        tmax = p["friction"] * M * g * (p["foot_grip_mm"] / 1000 if p.get("foot_grip_mm") else (p["foot_len"] / 1000) / 4)
        yG = k["mb"] * k["yb"] / M
        xfoot = (p["sole_in"] + p["sole_out"]) / 2000 if p["sole_r"] else p["b"] / 1000
        psf, wf, pf, stick, G_prev = 0.0, 0.0, 0.0, True, None
        sl_prev = None
    hz = h - zs
    T, dead, vmax = math.radians(p["turn_deg"]), math.radians(p["dead_deg"]), p["servo_speed"]

    def contact(th):
        i = int(np.argmax(xs * math.sin(th) + hz * math.cos(th)))
        return i, xs[i], hz[i]

    def J(th):
        _, xc, hc = contact(th)
        return IG + M * (xc * xc + hc * hc)

    x, v, beta, bd, cmd = th0, 0.0, 0.0, 0.0, 0.0
    wf_prev = [0.0]
    psf_log = 0.0
    ic, xc, hc = contact(x)
    out, landings = [], 0
    for n in range(int(secs / DT)):
        t = n * DT
        if beta_of_t is not None:
            b_new = beta_of_t(t)
        elif p["clock_hz"]:            # open loop: the turn on a fixed clock,
            on_right = (t * p["clock_hz"] + p["clock_phase"]) % 1.0 < 0.5   # as
            want = p["dir"] * T if on_right else -p["dir"] * T            # yaw_duck
            cmd += max(-vmax * DT, min(vmax * DT, want - cmd))
            b_new = cmd
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
        extra = 0.0
        if legs:
            if t >= p["legs_start_s"]:
                ramp = min(1.0, (t - p["legs_start_s"]) / 1.0)
                base = t * p["clock_hz"] + p["clock_phase"] - p["slide_phase"]
                sl = np.array([ramp * leg_profile(base % 1.0, A), ramp * leg_profile((base + 0.5) % 1.0, A)])
            else:
                sl = np.zeros(2)
            sd_ = np.zeros(2) if sl_prev is None else (sl - sl_prev) / DT
            sl_prev = sl
            st, sw = (0, 1) if x >= 0 else (1, 0)
            sgn = 1.0 if x >= 0 else -1.0
            xP = xc                        # the contact, sideways from the middle
            yP = p["foot_y"] / 1000 + sl[st]
            xsw = -sgn * xfoot
            G = -ky["Jb"] * bd + (M - ml) * xP * sd_[st] + ml * (xsw - xP) * sd_[sw]
            IP = ky["Iz"] + M * (xP * xP + (yG - yP) ** 2)
            Gd = 0.0 if G_prev is None else (G - G_prev) / DT
            G_prev = G
            if stick:
                if abs(Gd) <= tmax:
                    wf, pf = 0.0, G
                else:                      # it breaks loose: the foot turns
                    stick = False
                    pf = G - Gd * DT + math.copysign(tmax, Gd) * DT
                    wf = (pf - G) / IP
            else:
                pf += -math.copysign(tmax, wf) * DT
                w_new = (pf - G) / IP
                if w_new * wf <= 0:
                    stick, w_new, pf = True, 0.0, G
                wf = w_new
            wfd = (wf - wf_prev[0]) / DT
            wf_prev[0] = wf
            psf += wf * DT
            lev = yG - yP                  # the centre of mass ahead of P
            dxf = -lev * math.sin(psf)
            dxfdd = -lev * (wfd * math.cos(psf) - wf * wf * math.sin(psf))
            extra = M * (g * dxf - h * dxfdd)
        Jc = IG + M * (xc * xc + hc * hc)
        eps = 1e-4
        dJ = (J(x + eps) - J(x - eps)) / (2 * eps)
        Hp = xc * math.cos(x) - hc * math.sin(x)
        acc = (push(k, -beta, -bd, -bdd) + extra - c * v - 0.5 * dJ * v * v - M * g * Hp) / Jc
        v += acc * DT
        x += v * DT
        i2, x2, h2 = contact(x)
        if i2 != ic and math.hypot(x2 - xc, h2 - hc) > JUMP:      # a landing
            # G - C in the body frame: (-x_c, h_c)
            v *= (IG + M * (x2 * xc + h2 * hc)) / (IG + M * (x2 * x2 + h2 * h2))
            landings += 1
        ic, xc, hc = i2, x2, h2
        if abs(x) > 0.7:
            return dict(fell=True, t=t, out=np.array(out), landings=landings)
        if n % 20 == 0:
            out.append((t, x, beta, psf if legs else 0.0))
    return dict(fell=False, out=np.array(out), landings=landings)


TALL = dict(hip=130, foot_y=10, dir=-1.0)
FREE = [("arc R200, gap 7", dict(sole_r=200)),
        ("arc R200, gap 20", dict(sole_r=200, sole_in=10, sole_out=66.5, b=38.25)),
        ("arc R200, gap 40", dict(sole_r=200, sole_in=20, sole_out=76.5, b=48.25)),
        ("arc R200, straight past 25 mm", dict(sole_r=200, sole_flat=25)),
        ("two points b 35", dict(sole_r=0.0, b=35))]


def job_free(case):
    name, over = case
    p = dict(YAW, **dict(TALL, **over, turn_deg=0.0, slide_mm_deg=0.0, secs=4.0))
    a = release(p, math.radians(8))
    k = consts(p)
    e = run(p, k, secs=4.0, servo=False)
    o = e["out"]
    return name, stats(a[:, 0], a[:, 1], 0.2)[1], peaks(a[:, 0], a[:, 1], 5), \
        (stats(o[:, 0], o[:, 1], 0.2)[1] if not e["fell"] else 0), peaks(o[:, 0], o[:, 1], 5)


def job_walk(case):
    name, over = case
    p = dict(YAW, **dict(TALL, **over, turn_deg=3, slide_mm_deg=0.0,
                         servo_kp=300.0, servo_damp=0.02, servo_speed=1.0))
    a = release(p)
    sim = stats(a[:, 0], a[:, 1], 9) if np.abs(np.degrees(a[:, 1])).max() < 30 else None
    k = consts(p)
    e = run(p, k)
    eq = None if e["fell"] else stats(e["out"][:, 0], e["out"][:, 1], 9)
    return name, sim, eq


if __name__ == "__main__":
    with mp.Pool() as pool:
        print("1. free rocking, released at 8 deg: sim Hz, peaks | equations Hz, peaks")
        for name, fs, ps, fe, pe in pool.map(job_free, FREE):
            print(f"   {name:30s} {fs:4.2f} {ps} | {fe:4.2f} {pe}")
        print("2. rocking with the servo (+-3 deg at 1 rad/s, stiff, no slide), 9-12 s: sim | equations")
        for name, sim, eq in pool.map(job_walk, FREE):
            ss = "FELL" if sim is None else f"+-{sim[0]:4.1f} {sim[1]:4.2f} Hz"
            es = "FELL" if eq is None else f"+-{eq[0]:4.1f} {eq[1]:4.2f} Hz"
            print(f"   {name:30s} {ss:16s} | {es}")
