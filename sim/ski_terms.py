# The ski duck's equations (ski_model.py) checked piece by piece against
# the simulator (ski.py): two-point feet, the walk that keeps going. Each
# step it logs the rocking, the yaw, the slides, and the floor's force on
# each of the four points; then it tests the model's assumptions with the
# simulator's own numbers:
#   1. the stance foot carries the weight (N = M g)
#   2. the centre of pressure sits h a / g behind the centre of mass
#      (a: the centre of mass's forward acceleration)
#   3. the yaw turns about the more loaded point, held by friction up to
#      mu N_light L, and turns only past that
#   4. when, in the rocking cycle, the foot tips and the body yaws
# Writes out/ski_terms.csv (per step) and out/ski_terms_phase.csv (averaged
# over the cycle), and prints the checks.
#
#   python ski_terms.py [L_mm]
# Then: "C:/Program Files/FreeCAD 1.0/bin/python.exe" ski_terms_plot.py

import csv
import math
import os
import sys

import mujoco
import numpy as np

from ski import SKI, block, release

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
POINTS = ("skiR", "skiRb", "skiL", "skiLb")      # right toe, heel; left toe, heel


def record(L, secs=12.0):
    p = dict(SKI, ski_len=L, step_mm=10, slide_v=150, secs=secs, line_r=1, two_pts=1)
    k = block(dict(p))
    rows = []
    f6 = np.zeros(6)
    ids = {}

    def hook(m, d):
        if not ids:
            for n in POINTS:
                ids[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)] = POINTS.index(n)
        Rm = d.xmat[1].reshape(3, 3)
        th = math.atan2(Rm[0, 2], Rm[2, 2])
        yaw = math.atan2(-Rm[0, 1], Rm[1, 1])
        fwd = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
        com = d.subtree_com[1].copy()
        N = np.zeros(4)                 # normal force on each point
        Y = np.full(4, np.nan)          # its place fore-aft from the CoM
        T = np.zeros(4)                 # its force's yaw moment about the CoM
        for i in range(d.ncon):
            c = d.contact[i]
            j = ids.get(c.geom1, ids.get(c.geom2))
            if j is None:
                continue
            mujoco.mj_contactForce(m, d, i, f6)
            frame = c.frame.reshape(3, 3)        # rows: normal, tangent 1, tangent 2
            F = frame.T @ f6[:3]                 # on geom2, the normal from 1 to 2
            if c.geom1 in ids:                   # the foot is geom1: on it, -F
                F = -F
            r = c.pos - com
            N[j] += f6[0]
            Y[j] = float(r @ fwd)
            T[j] += r[0] * F[1] - r[1] * F[0]    # its yaw moment about the CoM
        Z = [d.geom_xpos[g][2] for g in sorted(ids, key=ids.get)]   # each ball's height
        rows.append((d.time, th, d.qvel[5], *d.qpos[7:9], *d.qacc[6:8], *N, *Y, *T, *Z))
    release(p, math.radians(8), hook=hook)
    return p, k, np.array(rows, dtype=float)


def main():
    L = float(sys.argv[1]) if len(sys.argv) > 1 else 100.0
    p, k, a = record(L)
    t, th, wz = a[:, 0], a[:, 1], a[:, 2]
    sR, sL, aR, aL = a[:, 3], a[:, 4], a[:, 5], a[:, 6]
    N, Y, T = a[:, 7:11], a[:, 11:15], a[:, 15:19]
    M, h, g, mu = k["M"], k["h"], 9.81, p["friction"]
    dt = t[1] - t[0]
    steady = t > 6
    right = th > 0
    st_N = np.where(right[:, None], N[:, :2], N[:, 2:])        # stance toe, heel
    st_Y = np.where(right[:, None], Y[:, :2], Y[:, 2:])
    st_a = np.where(right, aR, aL)                             # stance slide accel
    Ntot = st_N.sum(1)
    print(f"L {L:g} mm, steady part {steady.sum() * dt:.1f} s")
    # 1. load
    print(f"1. stance load / M g: mean {np.mean(Ntot[steady]) / (M * g):.2f}, "
          f"5-95 % {np.percentile(Ntot[steady], 5) / (M * g):.2f}..{np.percentile(Ntot[steady], 95) / (M * g):.2f}")
    # 2. centre of pressure
    ok = steady & (Ntot > 0.2 * M * g) & ~np.isnan(st_Y).all(1)
    yc = np.nansum(st_N * np.nan_to_num(st_Y), 1) / np.maximum(Ntot, 1e-9)
    pred = h * st_a / g                                        # the model's
    sel = ok & (st_N > 0.02).all(1)                            # both points down
    if sel.sum() > 20:
        c = np.polyfit(pred[sel], yc[sel], 1)
        r = np.corrcoef(pred[sel], yc[sel])[0, 1]
        print(f"2. centre of pressure (both points down, {sel.mean() / ok.mean() * 100:.0f} % of stance): "
              f"measured = {c[0]:+.2f} x (h s''/g) {c[1] * 1000:+.1f} mm; corr {r:+.2f}")
    # 3. one point or two
    one = ok & ((st_N > 0.02).sum(1) == 1)
    print(f"3. on a single point {one.sum() / ok.sum() * 100:.0f} % of stance; "
          f"|yaw rate| on one point {np.degrees(np.abs(wz[one])).mean():.0f} deg/s, "
          f"on two {np.degrees(np.abs(wz[ok & ~one])).mean():.0f} deg/s")
    # which end carries it when on one point, and where the CoM is
    toe_only = one & (st_N[:, 0] > 0.02)
    print(f"   on the toe alone {toe_only.sum() / max(one.sum(), 1) * 100:.0f} % of those, heel alone "
          f"{(one & (st_N[:, 1] > 0.02)).sum() / max(one.sum(), 1) * 100:.0f} %")
    # friction torque used vs available (two points down)
    Tz = np.where(right[:, None], T[:, :2], T[:, 2:]).sum(1)
    avail = mu * np.min(st_N, 1) * L / 1000
    two = ok & (st_N > 0.02).all(1)
    print(f"   two points down: yaw friction moment used / mu N_light L: median "
          f"{np.median(np.abs(Tz[two]) / np.maximum(avail[two], 1e-9)):.2f}")
    # 4. phase: each rocking cycle starts landing on the right foot
    up = np.where((th[:-1] < 0) & (th[1:] >= 0) & steady[1:])[0]
    bins = 50
    acc = {kk: np.zeros(bins) for kk in ("th", "wz", "Nt", "Nh", "one", "sR", "sL", "aS", "cop", "pred")}
    cnt = np.zeros(bins)
    for i0, i1 in zip(up, up[1:]):
        idx = np.arange(i0, i1)
        ph = ((idx - i0) / (i1 - i0) * bins).astype(int)
        for kk, v in (("th", np.degrees(th)), ("wz", np.degrees(wz)), ("Nt", st_N[:, 0] / (M * g)),
                      ("Nh", st_N[:, 1] / (M * g)), ("one", one.astype(float)), ("sR", sR * 1000),
                      ("sL", sL * 1000), ("aS", st_a), ("cop", np.where(ok, yc * 1000, 0)),
                      ("pred", pred * 1000)):
            np.add.at(acc[kk], ph, v[idx])
        np.add.at(cnt, ph, 1)
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "ski_terms_phase.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["phase"] + list(acc))
        for b_ in range(bins):
            w.writerow([(b_ + 0.5) / bins] + [acc[kk][b_] / max(cnt[b_], 1) for kk in acc])
    print(f"4. {len(up) - 1} cycles averaged: out/ski_terms_phase.csv")


if __name__ == "__main__":
    main()
