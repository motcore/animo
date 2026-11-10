# How the yaw duck turns (user, 04/11: a model of the slipping). Seen from
# above: nothing in the duck turns relative to the floor unless a foot
# slips, since the legs slide straight and the bar holds both feet. In the
# sim (step_diff 20) the planted foot slides fore and aft and spins while it
# slides; with equal steps it never slips.
#
# The model: the whole duck in the floor's plane, its pieces moved by the
# servos (prescribed: the turn beta(t), each leg's slide s(t), as
# yaw_duck.py), its bar free to move (X, Y) and turn (psi) on the floor.
# Each piece i (point mass m_i, own yaw inertia I_i) sits at
#     r_i = P + R(psi) rho_i(t)       (rho_i: in the bar's frame)
# so its velocity is linear in u = (X', Y', psi'):
#     v_i = V + psi' z x (r_i - P) + R(psi) rho_i'(t)
# The momentum (p, and L about the floor's origin) is then A(q, t) u + b(t),
# and only the floor changes it:
#     d/dt [p; L] = sum_j [F_j; (c_j x F_j)_z]
# The floor: each foot touches at its toe and heel (as the sim's sole mesh
# does: two end points), half the foot's load each, with Coulomb friction
# along the point's own sliding velocity (one cone for slide AND spin:
# a foot sliding fore and aft has little left against turning):
#     F_j = -mu N_j v_j / sqrt(|v_j|^2 + eps^2)
# Toe and heel do not share the load evenly (pitch=True, the piece that
# makes it match the sim): moments about the heel give the toe's share,
#     N_toe = N (y_G - y_heel) / L - (N / M g) M a_fwd h_G / L
# clamped to [0, N] (past an end the foot sits on that one point and spins
# about it nearly freely). A leg landing far forward puts the weight on its
# heel, and pushing the duck forward leans it back onto it too.
# The loads: the rocking hands the weight from foot to foot on the clock
# (the sim: the right foot from cycle -0.02 to 0.48, swapped in ~0.06).
#
#   python yaw_slip_model.py      turn rate against step_diff: model vs sim

import math
import multiprocessing as mp
import sys

import numpy as np

from yaw_duck import YAW, leg_profile, release
from yaw_clock_model import BEST
from yaw_rock_model import consts

DT = 1e-4
EPS = 2e-3             # m/s — the friction's smoothing (sticking ~ slower than this)


def pieces(p):
    """Masses (kg), own yaw inertias, and where each sits: torso pieces in
    the torso frame (turned by -beta from the bar), the rest in the bar's."""
    kg, mm = (lambda g: g / 1000), (lambda v: v / 1000)
    cube, brain = kg(p["cube_g"]), kg(p["brain_g"])
    a = 0.056
    xf = mm(p["sole_in"] + p["sole_out"]) / 2         # each foot's middle, sideways
    Lf = mm(p["foot_len"])
    return dict(
        torso=[(cube, cube * 2 * a * a / 12, (0.0, 0.0)),
               (brain, brain * (a * a + (a / 2) ** 2) / 12, (0.0, mm(p["brain_y"])))],
        bar=[(kg(p["bar_g"]), kg(p["bar_g"]) * (2 * mm(p["b"])) ** 2 / 12, (0.0, 0.0))],
        foot=(kg(p["foot_g"]), kg(p["foot_g"]) * Lf * Lf / 12, xf, mm(p["foot_y"]), Lf))


def run(p, secs=20.0, lag=-0.02, swap=0.06, mu=None, t_out=0.01, loads_of_t=None, pitch=True, beta_of_t=None):
    mu = p["friction"] if mu is None else mu
    pc = pieces(p)
    mf, If, xf, yf, Lf = pc["foot"]
    M = sum(m for m, _, _ in pc["torso"]) + sum(m for m, _, _ in pc["bar"]) + 2 * mf
    g = 9.81
    T, vmax = math.radians(p["turn_deg"]), p["servo_speed"]
    A = p["slide_amp"] / 1000
    f = p["clock_hz"]
    X = np.zeros(2)
    psi = 0.0
    beta, bd = 0.0, 0.0
    u = np.zeros(3)
    mom = None
    out = []

    def internal(t):
        """beta and the slides at t (prescribed), and their rates."""
        ph = t * f + p["clock_phase"]
        ramp = min(1.0, max(0.0, t - p["legs_start_s"]))
        tn = min(1.0, max(0.0, t - p["turn_s"])) if p["turn_s"] else 0.0
        dA = tn * p["step_diff"] / 2000
        base = ph - p["slide_phase"]
        sR = ramp * leg_profile(base % 1.0, A + dA)
        sL = ramp * leg_profile((base + 0.5) % 1.0, A - dA)
        return sR, sL

    def loads(t):
        if loads_of_t is not None:             # the sim's own loads
            return loads_of_t(t)
        ph = (t * f + p["clock_phase"] - lag) % 1.0
        # right share: 1 on [swap/2, 0.5 - swap/2], ramps either side
        def ramp_in(x):
            return min(1.0, max(0.0, x / swap + 0.5))
        wR = min(ramp_in(ph if ph < 0.75 else ph - 1.0), ramp_in(0.5 - ph))
        return M * g * wR, M * g * (1 - wR)

    def state(t, sR, sL, sRd, sLd, beta, bd, psi):
        """Each mass's offset from P (world axes), its velocity's part from
        the internal motions, and the u-columns."""
        c, s = math.cos(psi), math.sin(psi)
        R = np.array([[c, -s], [s, c]])
        cb, sb = math.cos(-beta), math.sin(-beta)
        Rb = np.array([[cb, -sb], [sb, cb]])
        items = []                              # (m, I, d (offset), v_int, w_int)
        for m, I, rho in pc["torso"]:
            r_b = Rb @ np.array(rho)            # in the bar frame
            v_b = -bd * np.array([-r_b[1], r_b[0]])
            items.append((m, I, R @ r_b, R @ v_b, -bd))
        for m, I, rho in pc["bar"]:
            items.append((m, I, R @ np.array(rho), np.zeros(2), 0.0))
        for side, sl, sld in ((1, sR, sRd), (-1, sL, sLd)):
            items.append((mf, If, R @ np.array([side * xf, sl + yf]), R @ np.array([0.0, sld]), 0.0))
        return R, items

    def momentum(Xp, items, u):
        V, w = u[:2], u[2]
        P = np.zeros(2); L = 0.0
        for m, I, d, vi, wi in items:
            v = V + w * np.array([-d[1], d[0]]) + vi
            r = Xp + d
            P += m * v
            L += m * (r[0] * v[1] - r[1] * v[0]) + I * (w + wi)
        return np.array([P[0], P[1], L])

    prev = None
    F_prev = np.zeros(3)
    hG = consts(p)["h"] if pitch else 0.0
    nsteps = int(secs / DT)
    cmd = 0.0
    for n in range(nsteps):
        t = n * DT
        on_right = (t * f + p["clock_phase"]) % 1.0 < 0.5
        want = p["dir"] * T if on_right else -p["dir"] * T
        cmd += max(-vmax * DT, min(vmax * DT, want - cmd))
        if beta_of_t is not None:      # the sim's own servo angle
            cmd = beta_of_t(t)
        bd, beta = (cmd - beta) / DT, cmd
        sR, sL = internal(t)
        if prev is None:
            sRd = sLd = 0.0
        else:
            sRd, sLd = (sR - prev[0]) / DT, (sL - prev[1]) / DT
        prev = (sR, sL)
        R, items = state(t, sR, sL, sRd, sLd, beta, bd, psi)
        b = momentum(X, items, np.zeros(3))
        Am = np.column_stack([momentum(X, items, e) - b for e in np.eye(3)])
        if mom is None:
            mom = b + Am @ u
        u = np.linalg.solve(Am, mom - b)
        # the floor: toe and heel of each foot
        NR, NL = loads(t)
        F = np.zeros(3)
        if pitch:                      # the centre of mass along the bar's y
            yG = sum(m * (R.T @ d)[1] for m, _, d, _, _ in items) / M
            a_fwd = (R.T @ F_prev[:2])[1] / M          # its fore-aft push
        for side, sl, sld, N in ((1, sR, sRd, NR), (-1, sL, sLd, NL)):
            if N <= 0:
                continue
            yt, yh = sl + yf + Lf / 2, sl + yf - Lf / 2
            Nt = N / 2
            if pitch:                  # toe's share: moments about the heel;
                # pushed forward, the duck leans back onto the heel. Past the
                # end it tips onto it (clamped): all the load on one point
                Nt = min(N, max(0.0, N * (yG - yh) / Lf - (N / (M * g)) * M * a_fwd * hG / Lf))
            for yy, Nj in ((yt, Nt), (yh, N - Nt)):
                d = R @ np.array([side * xf, yy])
                v = u[:2] + u[2] * np.array([-d[1], d[0]]) + R @ np.array([0.0, sld])
                Fj = -mu * Nj * v / math.sqrt(v @ v + EPS * EPS)
                r = X + d
                F += (Fj[0], Fj[1], r[0] * Fj[1] - r[1] * Fj[0])
        mom = mom + F * DT
        F_prev = F
        X = X + u[:2] * DT
        psi += u[2] * DT
        if n % int(t_out / DT) == 0:
            out.append((t, psi - beta, X[0], X[1], psi))
    return np.array(out)


def rate(o, t0=9.0):
    t = o[:, 0]
    i0 = np.searchsorted(t, t0)
    hd = o[:, 1]
    return math.degrees(hd[-1] - hd[i0]) / (t[-1] - t[i0])


def sim_loads(p):
    """The sim's run, with each sole's load over time."""
    import mujoco
    rec, ids = [], {}
    def hook(m, d):
        if not ids:
            ids["R"], ids["L"] = m.geom("Rs").id, m.geom("Ls").id
        N = {"R": 0.0, "L": 0.0}
        f6 = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            for s in "RL":
                if ids[s] in (c.geom1, c.geom2):
                    mujoco.mj_contactForce(m, d, i, f6)
                    N[s] += f6[0]
        rec.append((d.time, N["R"], N["L"]))
    a = release(p, 0.0, hook=hook)
    r = np.array(rec)
    return a, lambda t: (float(np.interp(t, r[:, 0], r[:, 1])), float(np.interp(t, r[:, 0], r[:, 2])))


def job(diff):
    p = dict(YAW, **dict(BEST, secs=20.0, turn_s=6.0, step_diff=diff))
    a, nl = sim_loads(p)
    o = run(p, secs=20.0, pitch=False)
    o2 = run(p, secs=20.0)
    t = a[:, 0]
    i0 = np.searchsorted(t, 9)
    hs = np.unwrap(a[:, 2])
    w_sim = math.degrees(hs[-1] - hs[i0]) / (t[-1] - t[i0])
    fell = np.degrees(np.abs(a[:, 1])).max() > 30
    return diff, rate(o), rate(o2), w_sim, fell


if __name__ == "__main__":
    print("turn rate (deg/s, + left) against the steps' difference: model: toe and heel even | toe and heel from the lean | sim")
    with mp.Pool() as pool:
        for diff, wm, wm2, ws, fell in pool.map(job, (0, 10, 15, 20, -20)):
            print(f"  step_diff {diff:+3d} mm: {wm:+6.1f} | {wm2:+6.1f} | {ws:+6.1f}{'  (sim FELL)' if fell else ''}")
