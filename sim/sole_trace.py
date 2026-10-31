# Where each sole touches the floor, over the gait cycle: the contact's path
# on the foot, as a function of the step's phase, every cycle laid on top of
# the others. Its mean, its spread from cycle to cycle (how deterministic the
# gait is) and a few Fourier harmonics (a compact signature to compare runs).
#
#   python sole_trace.py                         the starting sample (PRESET)
#   python sole_trace.py --set freq=1.2 ...      the same, other knobs
#   python sole_trace.py --name foo              out/sole_foo_*.csv / .json
#
# Then the picture (FreeCAD's Python has matplotlib):
#   "C:/Program Files/FreeCAD 1.0/bin/python.exe" sole_plot.py out/sole_start
#
# Coordinates on the sole (spherical cap feet only, ball_r > 0): angles from
# the sphere's centre, in the leg's frame, so they ride with the foot:
#   u  side to side, + outward (the left foot mirrored to match the right)
#   v  fore and aft, + toward the toe
# times ball_r they are millimetres along the sole. Phase phi: 0..360 deg of
# the leg's swing, 90 = the right leg at its most forward.

import argparse
import csv
import json
import math
import os

import mujoco
import numpy as np

from trex_sim import P, Run

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")

# The starting sample (2026-10-29): Mugatu-like feet, rubber, no tail or head
PRESET = dict(gait=0, tail=0, tail_deg=0, tail_only_s=0, fade_s=2, lean=0, head=0,
              foot_w_g=40, ball_r=105, ball_gap=28, ball_y=12, freq=1.0,
              leg_deg=15, foot_mu=1.2, foot_spin=5)

WARMUP_S = 10.0          # let the rocking settle before recording
SECS = 70.0
EVERY = 4                # record every 4th step (2 ms)
BINS = 36                # 10 deg of phase each
HARMONICS = 3


def record(p):
    """Samples (t, phi, foot, in_contact, u, v, force) for both feet."""
    r = Run(p)
    m, d = r.m, r.d
    R_ = p["ball_r"] / 1000
    centre = [np.array([s * p["ball_gap"] / 2000, p["ball_y"] / 1000,
                        R_ - p["hip"] / 1000]) for s in (1, -1)]
    bodies = [m.geom_bodyid[g] for g in r.feet]
    f6 = np.zeros(6)
    rows = []
    n = int(p["secs"] / m.opt.timestep)
    for i in range(n):
        r.step()
        if r.fell:
            break
        if i % EVERY or d.time < WARMUP_S:
            continue
        w = 2 * math.pi * p["freq"] * d.time - math.radians(p["phase"])
        phi = math.degrees(w) % 360.0
        for k, (g, b) in enumerate(zip(r.feet, bodies)):
            pts, fs = [], []
            for j in range(d.ncon):
                c = d.contact[j]
                if g in (c.geom1, c.geom2):
                    mujoco.mj_contactForce(m, d, j, f6)
                    pts.append(c.pos.copy())
                    fs.append(max(f6[0], 0.0))
            if not pts or sum(fs) <= 0:
                rows.append((d.time, phi, k, 0, math.nan, math.nan, 0.0))
                continue
            pos = np.average(pts, axis=0, weights=fs)
            R = d.xmat[b].reshape(3, 3)
            q = R.T @ (pos - d.xpos[b]) - centre[k]
            side = 1 if k == 0 else -1
            u = math.degrees(math.atan2(side * q[0], -q[2]))
            v = math.degrees(math.atan2(q[1], -q[2]))
            rows.append((d.time, phi, k, 1, u, v, sum(fs)))
    res = r.result()
    res["weight_N"] = float(sum(m.body_mass) * 9.81)
    return rows, res


def fourier(phi, x, n=HARMONICS):
    """Least squares: x ~ a0 + sum a_k cos(k phi) + b_k sin(k phi)."""
    t = np.radians(phi)
    cols = [np.ones_like(t)]
    for k in range(1, n + 1):
        cols += [np.cos(k * t), np.sin(k * t)]
    A = np.stack(cols, 1)
    coef, *_ = np.linalg.lstsq(A, x, rcond=None)
    fit = A @ coef
    return coef, float(np.sqrt(np.mean((x - fit) ** 2)))


def analyse(rows, p):
    a = np.array(rows, dtype=float)
    edges = np.linspace(0, 360, BINS + 1)
    mid = (edges[:-1] + edges[1:]) / 2
    feet = {}
    for k, name in ((0, "right"), (1, "left")):
        f = a[a[:, 2] == k]
        on = f[f[:, 3] == 1]
        b_all = np.digitize(f[:, 1], edges) - 1
        b_on = np.digitize(on[:, 1], edges) - 1
        bins = []
        for i in range(BINS):
            sel = on[b_on == i]
            frac = float(np.mean(f[b_all == i, 3])) if np.any(b_all == i) else 0.0
            load = float(np.mean(f[b_all == i, 6])) if np.any(b_all == i) else 0.0
            if len(sel) >= 5:
                bins.append(dict(phi=mid[i], contact=frac, load=load, n=len(sel),
                                 u=float(sel[:, 4].mean()), v=float(sel[:, 5].mean()),
                                 su=float(sel[:, 4].std()), sv=float(sel[:, 5].std()),
                                 force=float(sel[:, 6].mean())))
            else:
                bins.append(dict(phi=mid[i], contact=frac, load=load, n=len(sel)))
        cu, ru = fourier(on[:, 1], on[:, 4]) if len(on) > 20 else ([], math.nan)
        cv, rv = fourier(on[:, 1], on[:, 5]) if len(on) > 20 else ([], math.nan)
        full = [b for b in bins if "u" in b]
        spread = (float(np.mean([math.hypot(b["su"], b["sv"]) for b in full]))
                  if full else math.nan)
        feet[name] = dict(
            bins=bins,
            contact_share=float(np.mean(f[:, 3])) if len(f) else 0.0,
            u_range=[float(on[:, 4].min()), float(on[:, 4].max())] if len(on) else None,
            v_range=[float(on[:, 5].min()), float(on[:, 5].max())] if len(on) else None,
            spread_deg=spread,
            spread_mm=spread * math.pi / 180 * p["ball_r"],
            fourier_u=[float(c) for c in cu], fourier_v=[float(c) for c in cv],
            fit_rms_u=ru, fit_rms_v=rv)
    # Symmetry: the left leg runs half a cycle behind the right one
    R_, L_ = feet["right"]["bins"], feet["left"]["bins"]
    diff = []
    for i in range(BINS):
        r, l = R_[i], L_[(i + BINS // 2) % BINS]
        if "u" in r and "u" in l:
            diff.append(math.hypot(r["u"] - l["u"], r["v"] - l["v"]))
    sym = float(np.sqrt(np.mean(np.square(diff)))) if diff else math.nan
    return dict(feet=feet, symmetry_rms_deg=sym, bins=BINS, harmonics=HARMONICS)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--set", nargs="*", default=[], metavar="KEY=VALUE")
    ap.add_argument("--name", default="start")
    a = ap.parse_args()
    p = dict(P, **PRESET)
    p["secs"] = SECS
    for kv in a.set:
        k, v = kv.split("=")
        assert k in p, f"unknown: {k}"
        p[k] = float(v)
    assert p["ball_r"] > 0, "sole coordinates need the spherical cap feet"
    rows, res = record(p)
    out = analyse(rows, p)
    out["params"] = {k: p[k] for k in sorted(p)}
    out["result"] = {k: (float(v) if not isinstance(v, bool) else v) for k, v in res.items()}
    os.makedirs(OUT, exist_ok=True)
    base = os.path.join(OUT, "sole_" + a.name)
    with open(base + "_samples.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "phi", "foot", "contact", "u", "v", "force"])
        w.writerows(rows)
    with open(base + ".json", "w") as f:
        json.dump(out, f, indent=1)
    print(f"{base}_samples.csv / .json  ({len(rows)} samples)")
    print(f"walk: forward {res['fwd_mm']:+.0f} mm, turn {res['turn_deg']:+.0f} deg"
          f"{'  FELL' if res['fell'] else ''}")
    for name, ft in out["feet"].items():
        print(f"  {name:5s} on the floor {ft['contact_share'] * 100:4.0f} % of the time;"
              f" u {ft['u_range'][0]:+5.1f}..{ft['u_range'][1]:+5.1f} deg,"
              f" v {ft['v_range'][0]:+5.1f}..{ft['v_range'][1]:+5.1f} deg;"
              f" spread {ft['spread_deg']:.2f} deg ({ft['spread_mm']:.1f} mm)")
    print(f"  right vs left (half a cycle apart): {out['symmetry_rms_deg']:.2f} deg rms")


if __name__ == "__main__":
    main()
