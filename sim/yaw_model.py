# The yaw duck (yaw_duck.py) as equations, and the two side by side.
#
# On one foot the bar is held, so the body turns by psi = -beta (beta: the
# servo, bar on body; k = 0) about the servo's axis, its centre of mass
# y_b ahead of it at height h_b. The rocking over the planted foot's line:
#     I0 theta'' = -M g R sin(alpha - |theta|) sgn(theta)
#                  + m_b (g dx - h_b dx''),   dx = -y_b sin(psi)
# (the turned body's centre of mass to the side: gravity on it, and its
# sideways acceleration, whose reaction rocks the rest the other way; small
# turns: +m_b h_b y_b psi''; the sign was wrong until 04/11), landing:
# theta' * r (Housner). The servo: a position loop on the body's
# yaw inertia J_b (the floor holds the bar), command rate-limited, torque
# limited, as in the simulator:
#     (J_b + M g^2) beta'' = clip(kp (cmd - beta) - c beta', +-tau_max)
# (g: the slide per rad; the servo also pushes the body along it)
# The leg's slide doesn't enter the rocking (it is along the foot's line);
# it carries the body forward: per step the planted leg goes from +g T to
# -g T, so ~2 g T forward a step.
#
#   python yaw_model.py         A: the equation fed the simulator's servo
#                               B: equations alone vs the simulator, swept

import math
import multiprocessing as mp
import sys

import numpy as np

from yaw_duck import YAW, release, summary

DT = 1e-4


def constants(p):
    g = 9.81
    mc, mbr = p["cube_g"] / 1000, p["brain_g"] / 1000
    mb = mc + mbr
    mf = (p["bar_g"] + 2 * p["foot_g"]) / 1000
    M = mb + mf
    hb = p["hip"] / 1000
    yb = mbr * p["brain_y"] / 1000 / mb
    b = p["b"] / 1000
    h = (mb * hb + mf * p["ball_r"] / 1000) / M
    I0 = (mc * 2 * 0.056 ** 2 / 12 + mbr * 2 * 0.056 ** 2 / 12 + mb * (b * b + hb * hb)
          + mf * b * b + p["foot_g"] / 1000 * (2 * b) ** 2)
    Jb = (mc * 2 * 0.056 ** 2 / 12 + mbr * (0.056 ** 2 + 0.028 ** 2) / 12
          + mbr * (p["brain_y"] / 1000) ** 2 + 0.0001)          # + the joint's armature
    return dict(g=g, M=M, mb=mb, hb=hb, yb=yb, b=b, h=h, I0=I0, Jb=Jb,
                R=math.hypot(b, h), al=math.atan2(b, h), r=1 - 2 * M * b * b / I0)


def push(k, psi, psid, psidd):
    """The turned body's pull on the rocking: its centre of mass sits
    -y_b sin(psi) to the side, so gravity pulls on it there and its sideways
    acceleration, h_b up, throws the rest."""
    dx = -k["yb"] * math.sin(psi)
    dxdd = -k["yb"] * (psidd * math.cos(psi) - psid * psid * math.sin(psi))
    return k["mb"] * (k["g"] * dx - k["hb"] * dxdd)


def rock(k, th0, psi_of_t, secs, dt):
    """The rocking equation, psi, psi', psi'' given: (t, theta) rows."""
    x, v, out = th0, 0.0, []
    for i in range(int(secs / dt)):
        s = 1 if x >= 0 else -1
        acc = (-k["M"] * k["g"] * k["R"] * math.sin(k["al"] - abs(x)) * s
               + push(k, *psi_of_t(i))) / k["I0"]
        v += acc * dt
        xn = x + v * dt
        if xn * x < 0:
            v *= k["r"]
        x = xn
        out.append((i * dt, x))
    return np.array(out)


def run(p, th0=math.radians(8), secs=12.0, ideal=False):
    """Equations alone: rocking, servo and the control, as yaw_duck.py.
    ideal: the servo exactly on its (rate-limited) command."""
    k = constants(p)
    T, dead = math.radians(p["turn_deg"]), math.radians(p["dead_deg"])
    kp, c, tmax, vmax = p["servo_kp"], p["servo_damp"], p["servo_torque"], p["servo_speed"]
    g_m = p["slide_mm_deg"] * 180 / math.pi / 1000
    x, v, beta, bd, cmd, fwd = th0, 0.0, 0.0, 0.0, 0.0, 0.0
    out = []
    for i in range(int(secs / DT)):
        want = cmd
        if x > dead:
            want = p["dir"] * T
        elif x < -dead:
            want = -p["dir"] * T
        cmd += max(-vmax * DT, min(vmax * DT, want - cmd))
        if ideal:                           # the servo right on its command
            bd_new = (cmd - beta) / DT
            bdd, bd, beta = (bd_new - bd) / DT, bd_new, cmd
        s = 1 if x >= 0 else -1             # the body turns, the bar held
        hous = -k["M"] * k["g"] * k["R"] * math.sin(k["al"] - abs(x)) * s
        if ideal:
            acc = (hous + push(k, -beta, -bd, -bdd)) / k["I0"]
        else:
            # rocking and servo together (Lagrange: they share the term
            # C theta' psi', C = m_b h_b y_b cos psi), psi = -beta:
            #   I0 theta'' + C beta'' = Housner + m_b g dx - m_b h_b y_b psi'^2 sin psi
            #   J  beta''  + C theta'' = servo torque + m_b g y_b cos(psi) sin(theta)
            tau = max(-tmax, min(tmax, kp * (cmd - beta) - c * bd))
            psi, psid = -beta, -bd
            C = k["mb"] * k["hb"] * k["yb"] * math.cos(psi)
            J = k["Jb"] + k["M"] * g_m * g_m    # + the body pushed along the slide
            F1 = (hous + k["mb"] * k["g"] * (-k["yb"] * math.sin(psi))
                  - k["mb"] * k["hb"] * k["yb"] * psid * psid * math.sin(psi))
            F2 = tau + k["mb"] * k["g"] * k["yb"] * math.cos(psi) * math.sin(x)
            det = k["I0"] * J - C * C
            acc = (F1 * J - C * F2) / det
            bdd = (k["I0"] * F2 - C * F1) / det
            bd += bdd * DT
            beta += bd * DT
        v += acc * DT
        xn = x + v * DT
        if xn * x < 0:
            v *= k["r"]
        x = xn
        fwd += g_m * abs(bd) * DT           # the planted leg carries the body
        if abs(x) > k["al"]:
            return dict(fell=True, t=i * DT)
        if i % 20 == 0:
            out.append((i * DT, x, beta, fwd))
    return dict(fell=False, out=np.array(out))


def stats(t, th, t0):
    sel = t > t0
    z = np.where(np.sign(th[:-1]) != np.sign(th[1:]))[0]
    z = z[t[z] > t0]
    hz = 1 / (2 * np.median(np.diff(t[z]))) if len(z) > 3 else 0.0
    return math.degrees(np.abs(th[sel]).max()), hz


def part_a():
    print("A. the rocking equation fed the simulator's own servo angle (k 0, 6 s):")
    for over in (dict(), dict(slide_mm_deg=0.0), dict(turn_deg=10)):
        p = dict(YAW, secs=6.0, **over)
        a = release(p)
        t, th, servo = a[:, 0], a[:, 1], a[:, 6]
        dt = t[1] - t[0]
        psi = -servo
        psidd = np.gradient(np.gradient(np.convolve(psi, np.ones(5) / 5, "same"), dt), dt)
        psid = np.gradient(np.convolve(psi, np.ones(5) / 5, "same"), dt)
        e = rock(constants(p), th[0], lambda i: (psi[i], psid[i], psidd[i]), t[-1], dt)
        A_s, f_s = stats(t, th, 3)
        A_e, f_e = stats(e[:, 0], e[:, 1], 3)
        print(f"   {str(over) or 'default':28s} sim +-{A_s:4.1f} deg {f_s:4.2f} Hz | equation +-{A_e:4.1f} deg {f_e:4.2f} Hz")


def job(over):
    p = dict(YAW, **over)
    s = summary(p, release(p))
    e = run(p)
    if e["fell"]:
        eq = f"FELL at {e['t']:.1f} s"
    else:
        o = e["out"]
        A, f = stats(o[:, 0], o[:, 1], 9)
        eq = f"+-{A:4.1f} deg {f:4.2f} Hz {o[-1, 3] * 1000:5.0f} mm"
    a = release(p)
    A_s, f_s = stats(a[:, 0], a[:, 1], 9)
    sim = "FELL" if s["fell"] else f"+-{s['rock']:4.1f} deg {f_s:4.2f} Hz {s['fwd']:5.0f} mm"
    return over, sim, eq


def part_b():
    print("B. equations alone vs the simulator (12 s; rocking at 9-12 s, rhythm, forward):")
    jobs = [dict(turn_deg=T) for T in (5, 10, 15, 20)]
    jobs += [dict(slide_mm_deg=g) for g in (0.0, 0.2, 0.45, 0.55, 0.67)]
    jobs += [dict(dir=1.0), dict(brain_y=0.0), dict(friction=0.4)]
    with mp.Pool() as pool:
        for over, sim, eq in pool.map(job, jobs):
            print(f"   {str(over):26s} sim {sim:30s} | equations {eq}")


if __name__ == "__main__":
    if len(sys.argv) < 2 or "a" in sys.argv[1:]:
        part_a()
    if len(sys.argv) < 2 or "b" in sys.argv[1:]:
        part_b()
