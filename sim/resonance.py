# The rocking's own frequency for each foot, and walking at it.
#
# 1. Free rocking: legs still, the body given a push sideways, left to rock;
#    the frequency from the roll's zero crossings, at a small and a large
#    swing (two non-concentric spheres: it depends on the amplitude).
# 2. Walking with the legs' rhythm at that frequency times 0.8..1.2, the
#    double support and the progress of each (support.one), to a CSV.
#
#   python resonance.py

import csv
import itertools
import math
import multiprocessing as mp
import os

import numpy as np

from sole_trace import PRESET
from support import one
from trex_sim import P, Run

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")

FEET = dict(ball_r=[80, 105], ball_gap=[15, 28, 40], foot_w_g=[20, 40])
RATIOS = [0.8, 0.9, 1.0, 1.1, 1.2]


def free_roll(feet, push):
    """Rocking frequency (Hz) and amplitude (deg) after a sideways push."""
    p = dict(P, **PRESET)
    p.update(feet)
    p.update(leg_deg=0.0, secs=8.0)
    r = Run(p)
    for _ in range(int(0.5 / r.m.opt.timestep)):     # settle
        r.step()
    r.d.qvel[4] += push                               # roll rate, rad/s
    r.roll.clear()
    for _ in range(int(6.0 / r.m.opt.timestep)):
        r.step()
        if r.fell:
            return math.nan, math.nan
    roll = np.array(r.roll)
    dt = r.m.opt.timestep
    x = roll - roll.mean()
    up = np.where((x[:-1] < 0) & (x[1:] >= 0))[0]
    if len(up) < 3:
        return math.nan, float(np.abs(x).max())
    up = up[:4]                                      # the first cycles
    periods = np.diff(up) * dt
    return 1.0 / periods.mean(), float(np.abs(x[:up[-1]]).max())


def measure(feet):
    lo = free_roll(feet, 0.6)
    hi = free_roll(feet, 1.5)
    return feet, lo, hi


def main():
    combos = [dict(zip(FEET, v)) for v in itertools.product(*FEET.values())]
    with mp.Pool() as pool:
        freqs = pool.map(measure, combos)
        print("free rocking (small push / big push):")
        jobs = []
        for feet, (f1, a1), (f2, a2) in freqs:
            print(f"  {feet}  {f1:.2f} Hz at +-{a1:.1f} deg,  {f2:.2f} Hz at +-{a2:.1f} deg")
            f0 = f1 if not math.isnan(f1) else f2
            if math.isnan(f0):
                continue
            for k in RATIOS:
                jobs.append(dict(feet, freq=round(f0 * k, 3), f_roll=round(f0, 3), ratio=k))
        rows = pool.map(_walk, jobs)
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, "resonance.csv")
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"\n{path}")
    print("walking, by rhythm / own rocking frequency (mean over the feet):")
    for k in RATIOS:
        sel = [r for r in rows if r["ratio"] == k]
        ok = [r for r in sel if not r["fell"]]
        print(f"  x{k:.1f}: double {np.mean([r['double'] for r in ok]) * 100:3.0f} %,"
              f" fwd {np.mean([r['fwd_mm'] for r in ok]):+5.0f} mm,"
              f" roll +-{np.mean([r['roll_amp'] for r in ok]):.1f},"
              f" |turn| {np.mean([abs(r['turn_deg']) for r in ok]):3.0f}, fell {len(sel) - len(ok)}")
    ok = [r for r in rows if not r["fell"]]
    print("best progress:")
    for r in sorted(ok, key=lambda r: -r["fwd_mm"])[:8]:
        print(f"  R {r['ball_r']:g} gap {r['ball_gap']:g} w {r['foot_w_g']:g}  f_roll {r['f_roll']:.2f}"
              f" x{r['ratio']:.1f}  double {r['double'] * 100:3.0f} %  fwd {r['fwd_mm']:+5.0f}"
              f"  turn {r['turn_deg']:+4.0f}  roll +-{r['roll_amp']:.1f}")


def _walk(job):
    over = {k: v for k, v in job.items() if k not in ("f_roll", "ratio")}
    out = one(over)
    out.update(f_roll=job["f_roll"], ratio=job["ratio"])
    return out


if __name__ == "__main__":
    main()
