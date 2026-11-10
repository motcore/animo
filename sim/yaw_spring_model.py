# The yaw duck with a spring in each leg (user, 04/11), as equations: no
# MuJoCo, so a choice of spring takes seconds, not a sweep.
#
# Seen from the front: the body (centre of mass h up, roll inertia about
# the floor under it I = I_cm + M h^2) on two feet b out. It moves down u
# and rolls theta (+: the right side down). Each foot's leg comes down by
#     w_i = u + s_i b theta          (s = +1 right, -1 left)
# and the floor pushes back F_i(w_i) >= 0, the leg's spring with a preload:
# stiff (k_hard) until the load reaches F0, then soft k (and damping c):
#     F = k_hard w                     w <= F0 / k_hard
#     F = F0 + k (w - F0 / k_hard)     beyond
# Each foot's force acts at s_i b - h theta from the centre of mass
# (the body leans over the feet), so:
#     M u''     = M g - sum F_i
#     I theta'' = -sum (s_i b - h theta) F_i + (the servo's push)
# The push, as yaw_model.py (sign fixed 04/11):
#     + m_b g dx - m_b h_b dx'',  dx = -y_b sin(psi), psi = -beta
# Small rocking, both feet down, no preload: w^2 = (2 k b^2 - M g h) / I.
# A foot lifts once its load is gone: standing, at theta ~ M g / (2 k b).
#
# The servo: ideal on its rate-limited command (the rocking sets it, as in
# yaw_duck.py). Forward: the planted leg's slide counts only while the other
# foot is off the floor (both down, the two slides fight).
#
#   python yaw_spring_model.py            the free rhythm vs yaw_duck.py
#   python yaw_spring_model.py walk       a map of spring and turn, quick

import math
import sys

import numpy as np

from yaw_duck import YAW
import yaw_model as ym

DT = 5e-5
K_HARD = 2.0e4         # N/m — a foot at its stop (the plastic, the floor)
C_HARD = 100.0         # N s/m — and its damping: critical for half the
                       #    duck (2 sqrt(K_HARD M / 2)), no bounce, as in MuJoCo


def spring_consts(p):
    k = ym.constants(p)
    I_cm = k["I0"] - k["M"] * k["R"] ** 2
    k["I"] = I_cm + k["M"] * k["h"] ** 2
    return k


def run(p, th0=math.radians(8), secs=12.0, servo=True):
    k = spring_consts(p)
    M, g, h, b, I = k["M"], k["g"], k["h"], k["b"], k["I"]
    kk, c = p["leg_k"], p["leg_c"]
    F0 = kk * p["leg_pre"] / 1000
    w0 = F0 / K_HARD
    T, dead = math.radians(p["turn_deg"]), math.radians(p["dead_deg"])
    vmax = p["servo_speed"]
    g_m = p["slide_mm_deg"] * 180 / math.pi / 1000
    # start: resting on the right foot, tilted th0 (its spring loaded by M g)
    u = 0.0
    x, v, ud = th0, 0.0, 0.0
    beta, bd, cmd, fwd = 0.0, 0.0, 0.0, 0.0
    out = []
    # settle u so the right foot carries M g at the start
    for _ in range(20000):
        wR = u + b * x
        F = force(wR, 0.0, kk, c, F0, w0)
        u += (M * g - F) * 1e-7
    for i in range(int(secs / DT)):
        want = cmd
        if servo and x > dead:
            want = p["dir"] * T
        elif servo and x < -dead:
            want = -p["dir"] * T
        cmd += max(-vmax * DT, min(vmax * DT, want - cmd))
        bd_new = (cmd - beta) / DT
        bdd, bd, beta = (bd_new - bd) / DT, bd_new, cmd
        Fs = []
        for s in (1, -1):
            w = u + s * b * x
            wd = ud + s * b * v
            Fs.append(force(w, wd, kk, c, F0, w0))
        udd = (M * g - sum(Fs)) / M
        tau = -sum((s * b - h * x) * F for s, F in zip((1, -1), Fs))
        tau += ym.push(k, -beta, -bd, -bdd)
        vdd = tau / I
        ud += udd * DT
        u += ud * DT
        v += vdd * DT
        x += v * DT
        one = (Fs[0] > 0) != (Fs[1] > 0)
        if one:
            fwd += g_m * abs(bd) * DT
        if abs(x) > 0.6:
            return dict(fell=True, t=i * DT)
        if i % 40 == 0:
            out.append((i * DT, x, beta, fwd, Fs[0], Fs[1]))
    return dict(fell=False, out=np.array(out))


def force(w, wd, kk, c, F0, w0):
    """The floor on one foot through its leg: 0 off the floor."""
    if w <= 0:
        return 0.0
    if w <= w0 or not kk:
        return max(0.0, K_HARD * w + C_HARD * wd)
    return max(0.0, F0 + kk * (w - w0) + c * wd)


def rhythm(o, t0):
    t, th = o[:, 0], o[:, 1]
    th = th - th[t > t0].mean()
    z = np.where(np.sign(th[:-1]) != np.sign(th[1:]))[0]
    z = z[t[z] > t0]
    return 1 / (2 * np.median(np.diff(t[z]))) if len(z) > 3 else 0.0


def free():
    print("free rocking (servo still), released at 8 deg: equations vs yaw_duck.py (spring_free runs)")
    for kk in (150, 300, 600):
        p = dict(YAW, leg_k=kk, leg_c=0.5, leg_pre=0.0)
        e = run(p, secs=4.0, servo=False)
        o = e["out"]
        k = spring_consts(p)
        net = 2 * kk * k["b"] ** 2 - k["M"] * k["g"] * k["h"]
        print(f"   k {kk}: equations {rhythm(o, 0.3):4.2f} Hz, formula {math.sqrt(net / k['I']) / (2 * math.pi):4.2f}")


def walk_one(job):
    kk, pre, T = job
    p = dict(YAW, leg_k=kk, leg_c=1.0, turn_deg=T)
    M, g = spring_consts(p)["M"], 9.81
    p["leg_pre"] = pre * M * g / kk * 1000
    e = run(p)
    if e["fell"]:
        return job, f"FELL at {e['t']:.1f} s"
    o = e["out"]
    late = o[:, 0] > 9
    one = np.mean(((o[:, 4] > 0) != (o[:, 5] > 0))[late]) * 100
    return job, (f"+-{math.degrees(np.abs(o[late, 1] - o[late, 1].mean()).max()):4.1f} deg "
                 f"{rhythm(o, 8):4.2f} Hz {o[-1, 3] * 1000:5.0f} mm, one foot {one:3.0f} %")


def walk():
    import itertools
    import multiprocessing as mp
    print("walk map (12 s): rocking at 9-12 s, Hz, forward, % of time on one foot")
    jobs = list(itertools.product((200, 300, 400, 600), (0.0, 0.5, 0.75), (5, 10, 15)))
    with mp.Pool() as pool:
        for (kk, pre, T), r in pool.map(walk_one, jobs):
            print(f"   k {kk:3d}, preload {pre:4.2f} M g, turn {T:2d}: {r}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        free()
    elif sys.argv[1] == "walk":
        walk()
