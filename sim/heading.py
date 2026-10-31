# How the heading evolves along the walk: a steady turn (a bias, trimmable)
# or a drift (wanders, either way)? Long runs, the heading every 50 ms, and
# the same gait started with a tiny nudge (1e-6 rad/s of yaw) several times:
# if the copies part ways, the turn is not in the gait but in its chaos.
#
#   python heading.py                       the default cases, 120 s
#   python heading.py --set leg_deg=15 freq=2.82
#
# Writes out/heading.csv (t, case, copy, heading_deg, x_mm, y_mm) and the
# knobs to out/heading.json; the picture with heading_plot.py.

import argparse
import csv
import itertools
import json
import math
import multiprocessing as mp
import os

import numpy as np

from sole_trace import PRESET
from trex_sim import P, Run

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
SECS = 120.0
EVERY_S = 0.05
COPIES = 4               # the same gait, nudged by k * 1e-6 rad/s of yaw

BASE = dict(cap_in=10, cap_out=50, cap_heel=45, cap_toe=40, leg_deg=10, freq=3.7)
CASES = {
    "nominal":     {},
    "rhythm -5%":  {"freq": 3.7 * 0.95},
    "rhythm +5%":  {"freq": 3.7 * 1.05},
    "heavy brain": {"brain_g": 75.0, "cube_g": 110.0},
}


def one(job):
    name, k, over = job
    p = dict(P, **PRESET)
    p.update(over)
    p["secs"] = SECS
    r = Run(p)
    m, d = r.m, r.d
    every = int(EVERY_S / m.opt.timestep)
    r.step()
    d.qvel[5] += k * 1e-6
    rows, last, turns = [], None, 0.0
    for i in range(int(SECS / m.opt.timestep)):
        r.step()
        if r.fell:
            break
        if i % every:
            continue
        R = d.xmat[1].reshape(3, 3)
        h = math.degrees(math.atan2(-R[0, 1], R[1, 1]))
        if last is not None:                      # unwrap
            turns += (h - last + 180) % 360 - 180 - (h - last)
        last = h
        rows.append((round(d.time, 3), name, k, h + turns,
                     (d.qpos[0] - r.start[0]) * 1000, (d.qpos[1] - r.start[1]) * 1000))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--set", nargs="*", default=[], metavar="KEY=VALUE")
    a = ap.parse_args()
    base = dict(BASE)
    for kv in a.set:
        key, v = kv.split("=")
        base[key] = float(v)
    if "freq" in base:
        f = base["freq"]
        CASES["rhythm -5%"]["freq"], CASES["rhythm +5%"]["freq"] = f * 0.95, f * 1.05
    jobs = [(n, k, dict(base, **c)) for (n, c), k in itertools.product(CASES.items(), range(COPIES))]
    with mp.Pool() as pool:
        out = pool.map(one, jobs)
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, "heading.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "case", "copy", "heading_deg", "x_mm", "y_mm"])
        for rows in out:
            w.writerows(rows)
    with open(os.path.join(OUT, "heading.json"), "w") as f:
        json.dump(base, f)
    print(path, base)
    for rows in out:
        h = np.array([r[3] for r in rows])
        t = np.array([r[0] for r in rows])
        q = len(h) // 4
        quarters = [h[min((i + 1) * q, len(h) - 1)] - h[i * q] for i in range(4)]
        rate = np.polyfit(t, h, 1)[0]
        print(f"  {rows[0][1]:12s} copy {rows[0][2]}  end {h[-1]:+6.0f} deg"
              f"  fit {rate * 60:+6.0f} deg/min  by quarter " +
              " ".join(f"{x:+5.0f}" for x in quarters) +
              f"  walked {math.hypot(rows[-1][4], rows[-1][5]) / 1000:4.1f} m")


if __name__ == "__main__":
    main()
