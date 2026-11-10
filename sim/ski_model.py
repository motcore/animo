# The ski duck as equations (2026-11-03): rocking + yaw, nothing else.
# A reduced model of ski.py's two-point feet, to see whether the walk that
# keeps itself going (ski length 100-110 mm) follows from a few terms.
#
# Each foot: a toe and a heel, L apart, on a line b out from the middle,
# sliding fore and aft (s, + forward) under the body. The body stands on
# one foot (theta > 0: the right one) and rocks over its outer line
# (Housner's block). The stance foot holds the floor at two points: the
# one nearer the centre of mass carries more load, so the body yaws (psi)
# about it, P, the other point sliding round it. The yaw is held back by
# friction up to
#     tau_max = mu N (L/2 - |e|)        e: the centre of mass's offset
#                                          from the foot's middle, = s
# Centre of mass relative to P: x_c = h theta - sgn(theta) b sideways,
# y_c = -y_P fore-aft. With the body yawing about P, its kinetic energy
#     T = 1/2 I0 theta'^2 + 1/2 J psi'^2 - M h theta' psi' y_c
#         + M x_c y_c' psi' + (the swing foot's)
# (Lagrange). Its momenta and equations:
#     p_theta = I0 theta' - M h psi' y_c
#     p_theta' = -M g R sin(alpha - |theta|) sgn(theta) + M h psi' (y_c' + psi' x_c)
#     p_psi = J psi' + G,  G = M (x_c y_c' - h theta' y_c) + m_s x_s y_s'
#     p_psi' = tau_f   (friction at P: |tau_f| <= tau_max; while it holds,
#                       psi' = 0 and tau_f = G')
# The foot tips (the user's "hop", 03/11): sliding the body along takes a
# floor force at the sole, h below the centre of mass, so the centre of
# pressure sits y_cop = h s''/g from it. P is the end nearer y_cop, and the
# friction lever is y_cop's distance from the other end:
#     tau_max = mu M g (L/2 - |y_cop - e|)
# Past an end the foot stands on that end alone and yaws almost freely.
# Landing on the other foot: theta' times r (Housner), psi' kept, P moves.
# The skis' slides: the same servo and rhythm as ski.py (the rocking sets it).
#
#   python ski_model.py                  the lengths, equations vs ski.py

import math
import multiprocessing as mp
import sys

import numpy as np

from ski import SKI, block

DT = 2e-5


SPIN = 0.0005          # m — a single point's own friction lever (its patch)
LIFT_FREE = 0.05e-3    # m — run_pitch: the yaw is free only once the lifted end
                       #    is this far up (0: as soon as the foot tips)


def run(p, theta0=math.radians(8), secs=12.0, log_every=0, tip=False):
    """tip=True: the tipping foot below (03/11: it tips 38 % of the first
    second, near the simulator's 52 %, but then the rocking dies at every
    length, so it is off by default)."""
    k = block(dict(p, line_r=1))
    M, h, I0, R, al, b, r = k["M"], k["h"], k["I0"], k["R"], k["alpha"], k["b"], k["r"]
    g, mu, L = 9.81, p["friction"], p["ski_len"] / 1000
    ms = p["ski_g"] / 1000
    Iz = p["body_g"] / 1000 * 2 * 0.056 ** 2 / 12        # the body's own yaw inertia
    A, v = p["step_mm"] / 1000, p["slide_v"] / 1000
    kp, c = p["slide_kp"], p["slide_c"]
    dead = math.radians(p["dead_deg"])
    th, pth, psi, wpsi = theta0, 0.0, 0.0, 0.0
    s = np.zeros(2)                    # slides, right / left
    sd = np.zeros(2)
    cmd = np.zeros(2)
    def Gof(yP, xc, ycd, thd, xs, ysd):
        return M * (xc * ycd + h * thd * yP) + ms * xs * ysd
    stick, relink = True, False
    G_prev, ppsi, P_prev, n_tip = None, 0.0, None, 0
    y = 0.0                            # body forward, world, ~
    out = []
    n = int(secs / DT)
    for i in range(n):
        side = 1 if th >= 0 else -1    # stance: right (0) for +
        st, sw = (0, 1) if side > 0 else (1, 0)
        # the rhythm: the stance ski back, the raised one forward
        if th > dead:
            want = np.array([-A, A])
        elif th < -dead:
            want = np.array([A, -A])
        else:
            want = cmd
        cmd += np.clip(want - cmd, -v * DT, v * DT)
        # slide servos: the stance one moves the body, the raised one itself
        meff = np.array([ms, ms])
        meff[st] = M - ms
        sdd = (kp * (cmd - s) - c * sd) / meff
        sd += sdd * DT
        s += sd * DT
        # geometry about P (body axes, centre of mass at the origin)
        e = s[st]
        if abs(e) >= L / 2:
            return dict(fell=True, t=i * DT, out=out)
        if tip:                         # the foot tips (the user's hop):
            # sliding the body along needs a floor force h below the centre
            # of mass, so the centre of pressure moves h a / g back from it
            ycop = h * sdd[st] / g
        else:
            ycop = 0.0
        dmid = ycop - e                 # from the foot's middle
        yP = e + math.copysign(L / 2, dmid if dmid else -e or 1.0)
        tipped = abs(dmid) >= L / 2     # past an end: on that end alone
        n_tip += tipped
        if P_prev is not None and yP != P_prev and not stick:
            relink = True               # P moved while yawing: psi' kept
        P_prev = yP
        yc, ycd = -yP, -sd[st]
        xc = h * th - side * b
        xs, ysd = -2 * side * b, sd[sw] - sd[st]
        thd_guess = (pth + M * h * wpsi * yc) / I0
        J = Iz + M * (xc ** 2 + yc ** 2) + ms * (xs ** 2 + (s[sw] - yP) ** 2)
        G = M * (xc * ycd - h * thd_guess * yc) + ms * xs * ysd
        # G' with the same P before and after (P may have just moved)
        Gd = 0.0 if G_prev is None else (G - Gof(yP, *G_prev)) / DT
        G_prev = (xc, ycd, thd_guess, xs, ysd)
        # friction holds the yaw by the light end, as far from P as the
        # centre of pressure is from it; tipped, only the point's own patch
        tmax = mu * M * g * max(L / 2 - abs(dmid), SPIN)
        if relink:                      # just landed: P moved, psi' kept
            ppsi, relink = J * wpsi + G, False
        if stick:
            if abs(Gd) <= tmax:         # friction gives what it takes
                wpsi, ppsi = 0.0, G
            else:                       # breaks loose: it gives tau_max
                stick = False
                ppsi = G - Gd * DT + math.copysign(tmax, Gd) * DT
                wpsi = (ppsi - G) / J
        else:
            ppsi += -math.copysign(tmax, wpsi) * DT
            w_new = (ppsi - G) / J
            if w_new * wpsi <= 0:       # yaw stops: holds again
                stick, w_new, ppsi = True, 0.0, G
            wpsi = w_new
        psi += wpsi * DT
        # rocking
        grav = -M * g * R * math.sin(al - abs(th)) * side
        pth += (grav + M * h * wpsi * (ycd + wpsi * xc)) * DT
        thd = (pth + M * h * wpsi * yc) / I0
        th_new = th + thd * DT
        if th_new * th < 0:            # lands on the other foot
            thd *= r
            pth = I0 * thd - M * h * wpsi * yc
            G_prev, relink = None, not stick
        th = th_new
        y += -sd[st] * DT              # the stance foot holds: body goes by -s'
        if log_every and i % log_every == 0:
            out.append((i * DT, th, wpsi, psi, y))
    return dict(fell=False, t=secs, th=th, out=out, y=y, psi=psi, tipped=n_tip / n)


def run_pitch(p, theta0=math.radians(8), secs=12.0, log_every=0, slip=True):
    """Version 2 (03/11, after ski_terms.py measured the simulator): with
    both points down the foot never slips and nothing yaws; the yaw happens
    only while the foot stands on its heel or its toe. Starting and stopping
    the slide tips it there: the slide is internal, so about the pivot the
    angular momentum changes only by gravity,
        I_p phi'' = -M g R_s sin(alpha_s - |phi|) sgn(phi) + M h u'
    u: the centre of mass's forward speed over the foot (= -s' of the
    stance slide), phi: + toe up (on the heel), - heel up (on the toe).
    Flat while |M h u'| <= M g d (d: the centre of mass from that end,
    L/2 -+ e); landing flat stops phi. On one point the yaw is free, so
    p_psi = J psi' + G is kept from the moment it tips (psi' = 0 then); flat
    again, psi' = 0. The rocking as before."""
    k = block(dict(p, line_r=1))
    M, h, I0, R, al, b, r = k["M"], k["h"], k["I0"], k["R"], k["alpha"], k["b"], k["r"]
    g, L = 9.81, p["ski_len"] / 1000
    ms = p["ski_g"] / 1000
    mb = p["body_g"] / 1000
    Iz = mb * 2 * 0.056 ** 2 / 12
    Ipc = mb * (0.056 ** 2 + 0.084 ** 2) / 12           # the body's own pitch inertia
    A, v = p["step_mm"] / 1000, p["slide_v"] / 1000
    kp, c = p["slide_kp"], p["slide_c"]
    dead = math.radians(p["dead_deg"])
    th, pth, psi, wpsi = theta0, 0.0, 0.0, 0.0
    phi, wphi = 0.0, 0.0
    s, sd, cmd = np.zeros(2), np.zeros(2), np.zeros(2)
    ppsi, y, n_tip, side_prev = None, 0.0, 0, None
    mu, uf, sliding = p["friction"], 0.0, False    # the stance point's slide
    out, tips = [], []
    n = int(secs / DT)
    for i in range(n):
        side = 1 if th >= 0 else -1
        st, sw = (0, 1) if side > 0 else (1, 0)
        if side != side_prev:          # landed on the other foot: flat
            phi = wphi = wpsi = 0.0
            if slip and side_prev is not None:   # it lands at its own speed
                uf = (uf - sd[sw]) + sd[st]      # the body's, plus its slide
                sliding = uf != 0.0
            side_prev = side
        one = p["one_step"]
        if one:                        # ski.py's single step, then hold
            want = np.array([-A if one in (1, 2) else 0.0, A if one in (1, 3) else 0.0])
        elif th > dead:
            want = np.array([-A, A])
        elif th < -dead:
            want = np.array([A, -A])
        else:
            want = cmd
        cmd += np.clip(want - cmd, -v * DT, v * DT)
        meff = np.array([ms, ms])
        meff[st] = M - ms
        sdd = (kp * (cmd - s) - c * sd) / meff
        sd += sdd * DT
        s += sd * DT
        e = s[st]                      # the stance foot's middle, from the CoM
        # the floor's forward force on the stance foot: what it takes to
        # move the skis' momentum, P = m_s u_f + (M - m_s)(u_f - s'_st) + m_s s'_sw,
        # with the foot held (u_f' = 0); past mu M g the foot slides
        Fneed = -(M - ms) * sdd[st] + ms * sdd[sw]
        if slip:
            if sliding:
                F = -math.copysign(mu * M * g, uf)
                uf_new = uf + (F - Fneed) / M * DT
                if uf_new * uf <= 0 and abs(Fneed) <= mu * M * g:
                    uf_new, sliding, F = 0.0, False, Fneed
                uf = uf_new
            elif abs(Fneed) > mu * M * g:          # breaks loose
                F = math.copysign(mu * M * g, Fneed)
                uf, sliding = (F - Fneed) / M * DT, True
            else:
                F = Fneed
        else:
            F = Fneed
        ud = F / M                     # u': the CoM's forward acceleration
        # pitch: which end, how far from the CoM
        if phi == 0.0:
            drive = M * h * ud         # + lifts the toe: pushed forward at the sole, the body tips back
            dh, dt_ = L / 2 - e, L / 2 + e
            if drive > M * g * dh:
                phi, wphi = 1e-9, 0.0      # onto the heel
            elif -drive > M * g * dt_:
                phi, wphi = -1e-9, 0.0     # onto the toe
            if phi:
                tips.append((i * DT, phi > 0))
        tipped = phi != 0.0
        if tipped:
            sgn = 1.0 if phi > 0 else -1.0
            d = L / 2 - sgn * e            # heel: L/2 - e; toe: L/2 + e
            if d <= 0:
                return dict(fell=True, t=i * DT, out=out)
            Rs, als = math.hypot(d, h), math.atan2(d, h)
            Ip = Ipc + M * (d * d + h * h)
            wphi += ((-M * g * Rs * math.sin(als - abs(phi)) * sgn + M * h * ud) / Ip) * DT
            phi_new = phi + wphi * DT
            if abs(phi_new) > als:
                return dict(fell=True, t=i * DT, out=out)
            if phi_new * phi <= 0:         # back flat: the foot stops
                phi_new, wphi = 0.0, 0.0
            yP = e - sgn * L / 2           # the heel (+) or the toe (-)
        else:
            yP = e
        # yaw: free on one point (p_psi kept), held on two. The centre of
        # mass goes fore and aft with the slide AND with the pitch: toe up
        # (phi +) throws it back by h phi' (ski_compare: missed at first)
        yc, ycd = -yP, uf - sd[st] - h * wphi
        xc = h * th - side * b
        xs, ysd = -2 * side * b, sd[sw] - sd[st]
        thd = (pth + M * h * wpsi * yc) / I0
        J = Iz + M * (xc ** 2 + yc ** 2) + ms * (xs ** 2 + (s[sw] - yP) ** 2)
        G = M * (xc * ycd - h * thd * yc) + ms * xs * ysd
        free = tipped and abs(phi) * L > LIFT_FREE    # the other end clear
        if free:
            if wpsi == 0.0 and ppsi is None:
                ppsi = G
            if ppsi is None:
                ppsi = J * wpsi + G
            ppsi += M * uf * h * thd * DT  # the point slides: -v_P x p
            wpsi = (ppsi - G) / J
        else:
            wpsi, ppsi = 0.0, None
        if tipped and phi_new == 0.0:      # lands flat at the end of this step
            ppsi = None
        if tipped:
            phi = phi_new
        n_tip += free
        psi += wpsi * DT
        grav = -M * g * R * math.sin(al - abs(th)) * side
        pth += (grav + M * h * wpsi * (ycd + wpsi * xc)) * DT
        thd = (pth + M * h * wpsi * yc) / I0
        th_new = th + thd * DT
        if th_new * th < 0:
            thd *= r
            pth = I0 * thd               # the new foot lands flat: psi' = 0
            wpsi, ppsi = 0.0, None
        th = th_new
        if abs(th) > al:               # past the outer edge: it tips over
            return dict(fell=True, t=i * DT, out=out)
        y += (uf - sd[st]) * DT
        if log_every and i % log_every == 0:
            out.append((i * DT, th, wpsi, psi, y, phi, uf))
    return dict(fell=False, t=secs, th=th, out=out, y=y, psi=psi, tipped=n_tip / n, tips=tips)


def job(L):
    p = dict(SKI, ski_len=L, step_mm=10, slide_v=150, line_r=1, two_pts=1)
    res = run_pitch(p, log_every=50)
    if res["fell"]:
        return (L, f"FELL at {res['t']:.2f} s")
    o = np.array(res["out"])
    t, th = o[:, 0], np.degrees(o[:, 1])
    late = t > 9
    return (L, "FELL" if res["fell"] else "", round(float(np.abs(th[late]).max()), 1) if late.any() else 0,
            round(res.get("y", 0) * 1000), round(float(np.degrees(np.ptp(o[late, 3]))) if late.any() else 0, 1),
            f"{res.get('tipped', 0) * 100:.0f} % on one point")


if __name__ == "__main__":
    Ls = [float(x) for x in sys.argv[1:]] or [60, 80, 90, 100, 110, 120, 130, 160]
    with mp.Pool() as pool:
        print("L mm | rocking at 9-12 s deg | forward mm | yaw p-p deg   (equations)")
        for row in pool.map(job, Ls):
            print(" ", row)
