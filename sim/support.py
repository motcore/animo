# How long both feet stand on the floor together (double support), and
# whether less of it means more progress. Runs a grid in the nominal case,
# every run kept in a CSV for later study.
#
#   python support.py                                the default grid
#   python support.py --grid ball_gap=15,28,40 freq=0.8,1.0,1.2 --name gaps
#
# Each run: 40 s; recorded after a 10 s warm-up, every 2 ms. Columns: the
# knobs, then double / right only / left only / none (shares of the time),
# forward and side mm, turn deg, roll amplitude deg, yaw wobble deg (the
# body's twist each step, the drift taken out), fell.

import argparse
import csv
import itertools
import math
import multiprocessing as mp
import os

import numpy as np

from sole_trace import PRESET
from trex_sim import P, Run

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
SECS, WARMUP = 40.0, 10.0

GRID = dict(ball_gap=[15, 28, 40, 50], freq=[0.8, 1.0, 1.2], leg_deg=[10, 15, 20],
            foot_w_g=[20, 40])


def one(over):
    p = dict(P, **PRESET)
    p.update(over)
    p["secs"] = SECS
    r = Run(p)
    n = int(SECS / r.m.opt.timestep)
    both = right = left = none = 0
    start = None
    yaw = []
    for i in range(n):
        r.step()
        if r.fell:
            break
        if r.d.time < WARMUP or i % 4:
            continue
        if start is None:
            start = r.d.qpos[:2].copy()
        R = r.d.xmat[1].reshape(3, 3)
        yaw.append(math.atan2(-R[0, 1], R[1, 1]))
        a, b = r.load > 0.02
        both += a and b
        right += a and not b
        left += b and not a
        none += not a and not b
    tot = max(both + right + left + none, 1)
    res = r.result()
    fwd = (r.d.qpos[1] - start[1]) * 1000 if start is not None else 0.0
    wob = 0.0
    k = max(1, int(round(1 / p["freq"] / (4 * r.m.opt.timestep))))
    if len(yaw) > 3 * k:                # minus its mean over one step cycle
        y = np.degrees(np.unwrap(yaw))
        res_ = y[k // 2:len(y) - k // 2] - np.convolve(y, np.ones(k) / k, "valid")[:len(y) - 2 * (k // 2)]
        wob = float(np.percentile(res_, 95) - np.percentile(res_, 5)) / 2
    return dict(over, double=both / tot, right_only=right / tot, left_only=left / tot,
                airborne=none / tot, fwd_mm=fwd, side_mm=res["side_mm"],
                turn_deg=res["turn_deg"], roll_amp=res["roll_amp"], yaw_wobble=wob, fell=int(res["fell"]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--grid", nargs="*", default=[])
    ap.add_argument("--name", default="support")
    a = ap.parse_args()
    grid = dict(GRID) if not a.grid else {}
    for kv in a.grid:
        k, v = kv.split("=")
        grid[k] = [float(x) for x in v.split(",")]
    jobs = [dict(zip(grid, vals)) for vals in itertools.product(*grid.values())]
    with mp.Pool() as pool:
        rows = pool.map(one, jobs)
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, a.name + ".csv")
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    ok = [r for r in rows if not r["fell"]]
    ds = np.array([r["double"] for r in ok])
    fw = np.array([r["fwd_mm"] for r in ok])
    print(f"{path}: {len(rows)} runs, {len(rows) - len(ok)} fell")
    print(f"double support {ds.min() * 100:.0f}..{ds.max() * 100:.0f} %;"
          f" correlation with forward progress: {np.corrcoef(ds, fw)[0, 1]:+.2f}")
    for k in grid:
        print(f"  {k}:", "  ".join(
            f"{v:g} -> double {np.mean([r['double'] for r in ok if r[k] == v]) * 100:3.0f} %,"
            f" fwd {np.mean([r['fwd_mm'] for r in ok if r[k] == v]):+5.0f}"
            for v in grid[k]))
    print("lowest double support:")
    for r in sorted(ok, key=lambda r: r["double"])[:8]:
        print("  " + " ".join(f"{k}={r[k]:g}" for k in grid) +
              f"  double {r['double'] * 100:3.0f} %  fwd {r['fwd_mm']:+5.0f}  turn {r['turn_deg']:+4.0f}"
              f"  roll +-{r['roll_amp']:.1f}")


if __name__ == "__main__":
    main()
