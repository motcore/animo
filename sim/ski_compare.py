# Model v2 (ski_model.run_pitch) fed with the simulator's own motion
# (ski_terms.record): the rocking and the slides as measured, so each piece
# of the model is judged on its own, outside the loop.
#   A. pitch: from the measured slide acceleration, when does the model's
#      foot tip, and on which end, vs when the simulator's stands on one point
#   B. yaw: inside the simulator's own one-point spells, the model's free
#      yaw (p_psi kept from the start of the spell) vs the measured yaw rate
# Writes out/ski_compare.csv for ski_compare_plot.py.
#
#   python ski_compare.py [L_mm]

import csv
import math
import os
import sys

import numpy as np

from ski_terms import record

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    L = float(sys.argv[1]) if len(sys.argv) > 1 else 100.0
    p, k, a = record(L, secs=8.0)
    t, th, wz = a[:, 0], a[:, 1], a[:, 2]
    s = a[:, 3:5]
    acc = a[:, 5:7]
    N = a[:, 7:11]
    dt = t[1] - t[0]
    sd = np.gradient(s, dt, axis=0)
    thd = np.gradient(th, dt)
    M, h, b, g = k["M"], k["h"], k["b"], 9.81
    Lm = L / 1000
    ms, mb = p["ski_g"] / 1000, p["body_g"] / 1000
    Iz = mb * 2 * 0.056 ** 2 / 12
    Ipc = mb * (0.056 ** 2 + 0.084 ** 2) / 12
    side = np.where(th >= 0, 1, -1)
    st = np.where(side > 0, 0, 1)
    sw = 1 - st
    idx = np.arange(len(t))
    e = s[idx, st]
    ud = -acc[idx, st]
    Nst = np.where(side[:, None] > 0, N[:, :2], N[:, 2:])
    one = (Nst > 0.02).sum(1) == 1
    on_heel = one & (Nst[:, 1] > 0.02)
    # A. the model's pitch along the measured motion
    phi, wphi, tipped = 0.0, 0.0, np.zeros(len(t))
    prev_side = side[0]
    for i in range(len(t)):
        if side[i] != prev_side:
            phi = wphi = 0.0
            prev_side = side[i]
        if phi == 0.0:
            drive = M * h * ud[i]
            if drive > M * g * (Lm / 2 - e[i]):
                phi = 1e-9
            elif -drive > M * g * (Lm / 2 + e[i]):
                phi = -1e-9
        if phi:
            sg = 1.0 if phi > 0 else -1.0
            d = max(Lm / 2 - sg * e[i], 1e-4)
            Rs, als = math.hypot(d, h), math.atan2(d, h)
            Ip = Ipc + M * (d * d + h * h)
            wphi += ((-M * g * Rs * math.sin(als - abs(phi)) * sg + M * h * ud[i]) / Ip) * dt
            nphi = phi + wphi * dt
            if nphi * phi <= 0:
                nphi, wphi = 0.0, 0.0
            phi = nphi
        tipped[i] = np.sign(phi)
    steady = t > 3
    meas = np.where(one, np.where(on_heel, 1, -1), 0)
    agree = np.mean((tipped == meas)[steady])
    print(f"L {L:g}: on one point, simulator {np.mean(one[steady]) * 100:.0f} %, model fed the "
          f"measured slides {np.mean(tipped[steady] != 0) * 100:.0f} %; same state {agree * 100:.0f} % of the time")
    # B. the yaw inside the simulator's own spells
    yP = np.where(on_heel, e - Lm / 2, e + Lm / 2)
    yc, ycd = -yP, -sd[idx, st]
    xc = h * th - side * b
    xs, ysd = -2 * side * b, sd[idx, sw] - sd[idx, st]
    J = Iz + M * (xc ** 2 + yc ** 2) + ms * (xs ** 2 + (s[idx, sw] - yP) ** 2)
    G = M * (xc * ycd - h * thd * yc) + ms * xs * ysd
    Gs = M * xc * ycd                          # the parts of G, apart
    Gr = -M * h * thd * yc
    Gw = ms * xs * ysd
    pred = np.zeros(len(t))
    parts = np.zeros((len(t), 3))
    e_ = np.diff(np.r_[0, one.astype(int), 0])
    starts, ends = np.where(e_ == 1)[0], np.where(e_ == -1)[0]
    spells = []
    for i0, i1 in zip(starts, ends):
        seg = slice(i0, i1)
        pred[seg] = (G[i0] - G[seg]) / J[seg]
        for j, Gx in enumerate((Gs, Gr, Gw)):
            parts[seg, j] = (Gx[i0] - Gx[seg]) / J[seg]
        if t[i0] > 3 and i1 - i0 > 20:
            m = side[i0]                   # mirrored: as if on the right foot
            spells.append((t[i0], (i1 - i0) * dt, m * np.degrees(wz[seg]).mean(), m * np.degrees(pred[seg]).mean(),
                           *(m * np.degrees(parts[seg]).mean(0)), on_heel[i0]))
    sp = np.array(spells)
    print(f"  {len(sp)} one-point spells, {sp[:, 1].mean() * 1000:.0f} ms each; mean yaw rate in a spell:")
    for heel in (1, 0):
        q = sp[sp[:, 7] == heel]
        print(f"    on the {'heel' if heel else 'toe '}: simulator {q[:, 2].mean():+5.0f} deg/s, model {q[:, 3].mean():+5.0f}"
              f"  (slide {q[:, 4].mean():+.0f}, rocking {q[:, 5].mean():+.0f}, swing ski {q[:, 6].mean():+.0f})")
    with open(os.path.join(HERE, "out", "ski_compare.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "theta_deg", "meas_state", "model_state", "wz_meas", "wz_model", "slide_r", "slide_l"])
        for i in range(0, len(t), 5):
            w.writerow([t[i], math.degrees(th[i]), meas[i], tipped[i], math.degrees(wz[i]), math.degrees(pred[i]),
                        s[i, 0] * 1000, s[i, 1] * 1000])


if __name__ == "__main__":
    main()
