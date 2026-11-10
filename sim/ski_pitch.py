# The stance foot's pitch, measured in the simulator (ski_terms.record: the
# toe and heel balls' heights), against model v2's pieces:
#   1. the yaw law with the pitch term: in the one-point spells,
#      psi' = (G0 - G) / J, G with y_c' = -s' - h phi'
#   2. the pitch equation: I_p phi'' = -M g R_s sin(alpha_s - |phi|) sgn(phi)
#      + M h u', u' = -s'' (stance slide), on the simulator's own spells
#   3. what starts a tip: M h u' against M g d at the moment it starts
#
#   python ski_pitch.py [L_mm]

import math
import sys

import numpy as np

from ski_terms import record


def main():
    L = float(sys.argv[1]) if len(sys.argv) > 1 else 100.0
    p, k, a = record(L, secs=8.0)
    t, th, wz = a[:, 0], a[:, 1], a[:, 2]
    s, acc, N = a[:, 3:5], a[:, 5:7], a[:, 7:11]
    Z = a[:, 19:23]                     # right toe, heel; left toe, heel
    dt = t[1] - t[0]
    M, h, b, g = k["M"], k["h"], k["b"], 9.81
    Lm = L / 1000
    ms, mb = p["ski_g"] / 1000, p["body_g"] / 1000
    Iz = mb * 2 * 0.056 ** 2 / 12
    Ipc = mb * (0.056 ** 2 + 0.084 ** 2) / 12
    side = np.where(th >= 0, 1, -1)
    i_ = np.arange(len(t))
    st = np.where(side > 0, 0, 1)
    sw = 1 - st
    zt = np.where(side > 0, Z[:, 0], Z[:, 2])
    zh = np.where(side > 0, Z[:, 1], Z[:, 3])
    phi = (zt - zh) / Lm                 # + toe up
    def smooth(x, n=15):
        return np.convolve(x, np.ones(n) / n, "same")
    phid = np.gradient(smooth(phi), dt)
    phidd = np.gradient(smooth(phid), dt)
    sd = np.gradient(s, dt, axis=0)
    thd = np.gradient(th, dt)
    e = s[i_, st]
    ud = -acc[i_, st]
    Nst = np.where(side[:, None] > 0, N[:, :2], N[:, 2:])
    one = (Nst > 0.02).sum(1) == 1
    heel = one & (Nst[:, 1] > 0.02)
    steady = t > 3
    print(f"L {L:g}: stance foot pitch, 5-95 %: {np.degrees(np.percentile(phi[steady], 5)):+.2f}.."
          f"{np.degrees(np.percentile(phi[steady], 95)):+.2f} deg; on the heel mean {np.degrees(phi[steady & heel]).mean():+.2f}, "
          f"on the toe {np.degrees(phi[steady & one & ~heel]).mean():+.2f}")
    print(f"   pitch rate on one point, |phi'| median {np.degrees(np.median(np.abs(phid[steady & one]))):.0f} deg/s -> "
          f"h phi' {np.median(np.abs(phid[steady & one])) * h * 1000:.0f} mm/s")
    # 1. the yaw law, with and without the pitch term
    yP = np.where(heel, e - Lm / 2, e + Lm / 2)
    yc = -yP
    xc = h * th - side * b
    xs, ysd = -2 * side * b, sd[i_, sw] - sd[i_, st]
    J = Iz + M * (xc ** 2 + yc ** 2) + ms * (xs ** 2 + (s[i_, sw] - yP) ** 2)
    G0 = M * (xc * (-sd[i_, st]) - h * thd * yc) + ms * xs * ysd
    G1 = M * (xc * (-sd[i_, st] - h * phid) - h * thd * yc) + ms * xs * ysd
    e_ = np.diff(np.r_[0, one.astype(int), 0])
    starts, ends = np.where(e_ == 1)[0], np.where(e_ == -1)[0]
    P0, P1, Mz = [], [], []
    for i0, i1 in zip(starts, ends):
        if t[i0] < 3 or i1 - i0 < 30:
            continue
        seg = slice(i0, i1)
        P0.append(((G0[i0] - G0[seg]) / J[seg]).mean() * side[i0])
        P1.append(((G1[i0] - G1[seg]) / J[seg]).mean() * side[i0])
        Mz.append(wz[seg].mean() * side[i0])
    P0, P1, Mz = (np.degrees(np.array(x)) for x in (P0, P1, Mz))
    for name, P in (("without h phi'", P0), ("with h phi'   ", P1)):
        c = np.polyfit(P, Mz, 1)
        print(f"1. yaw in a spell, {name}: measured = {c[0]:+.2f} x model {c[1]:+.0f} deg/s; "
              f"corr {np.corrcoef(P, Mz)[0, 1]:+.2f}; same sign {np.mean(np.sign(P) == np.sign(Mz)) * 100:.0f} %")
    # 2. the pitch equation on the spells
    sg = np.where(heel, 1.0, -1.0)
    d = Lm / 2 - sg * e
    Rs, als = np.hypot(d, h), np.arctan2(d, h)
    Ip = Ipc + M * (d * d + h * h)
    model_pdd = (-M * g * Rs * np.sin(als - np.abs(phi)) * sg + M * h * ud) / Ip
    sel = steady & one
    c = np.polyfit(model_pdd[sel], phidd[sel], 1)
    print(f"2. pitch accel on one point: measured = {c[0]:+.2f} x model {c[1]:+.1f}; corr "
          f"{np.corrcoef(model_pdd[sel], phidd[sel])[0, 1]:+.2f}  (gravity part alone: "
          f"{np.median(np.abs(M * g * Rs * np.sin(als) / Ip)[sel]):.0f}, slide part: "
          f"{np.median(np.abs(M * h * ud / Ip)[sel]):.0f} rad/s2)")
    # 3. what starts each tip
    ratio, lead = [], []
    for i0 in starts:
        if t[i0] < 3:
            continue
        heel0 = heel[i0]
        dd = Lm / 2 - e[i0] if heel0 else Lm / 2 + e[i0]
        drive = M * h * ud[i0] * (1 if heel0 else -1)
        ratio.append(drive / (M * g * dd))
        lead.append(side[i0] * np.sign(thd[i0]))
    ratio = np.array(ratio)
    print(f"3. at the start of {len(ratio)} spells, slide drive / gravity hold: median {np.median(ratio):+.2f}, "
          f"10-90 % {np.percentile(ratio, 10):+.2f}..{np.percentile(ratio, 90):+.2f} (model: tips past +1)")
    # where in the rocking: theta and theta' at the starts
    tt = np.array([th[i] * side[i] for i in starts if t[i] > 3])
    td = np.array([thd[i] * side[i] for i in starts if t[i] > 3])
    print(f"   at the starts: tilt onto the stance foot {np.degrees(np.median(tt)):+.1f} deg, rocking "
          f"{'away from' if np.median(td) < 0 else 'onto'} it ({np.mean(td < 0) * 100:.0f} % away); "
          f"landings are at tilt 0")
    # the vertical load: the rocking's own lift and fall
    Ntot = Nst.sum(1)
    print(f"   stance load at the starts / M g: median {np.median(Ntot[[i for i in starts if t[i] > 3]]) / (M * g):.2f}")


if __name__ == "__main__":
    main()
