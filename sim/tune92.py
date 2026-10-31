# The gait at the longer legs (hip 92) with the feet grown to match (R and
# the cap x 92/52), where the feet start alternating at ~2 Hz: the knobs that
# hip_freq.py left unscaled, swept for progress. Every run to a CSV.
#
#   python tune92.py [tune92 | tune92_y]

import csv
import itertools
import multiprocessing as mp
import os
import sys

from support import one

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")

K = 92 / 52
FIXED = dict(hip=92, ball_r=105 * K, cap_in=0.5, cap_out=50 * K, cap_heel=45 * K, cap_toe=40 * K)
GRIDS = {
    "tune92": dict(ball_gap=[28, 40, 50], ball_y=[12, 21], foot_w_g=[40, 80],
                   leg_deg=[10, 15, 20], freq=[1.8, 2.0, 2.2, 2.4]),
    # 2nd round: most of the 1st walked backward; the fore-aft asymmetry
    # (the sphere ahead of or behind the hip) picks the way
    "tune92_y": dict(ball_gap=[28, 40], ball_y=[-30, -15, 0, 12, 30, 45], foot_w_g=[40],
                     leg_deg=[10, 15], freq=[2.0, 2.2, 2.4]),
}


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "tune92"
    GRID = GRIDS[name]
    jobs = [dict(FIXED, **dict(zip(GRID, v))) for v in itertools.product(*GRID.values())]
    with mp.Pool() as pool:
        rows = pool.map(one, jobs)
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name + ".csv")
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    ok = [r for r in rows if not r["fell"]]
    print(f"{path}: {len(rows)} runs, {len(rows) - len(ok)} fell")
    for k in GRID:
        print(f"  {k}:", "  ".join(
            f"{v:g} -> fwd {sum(r['fwd_mm'] for r in ok if r[k] == v) / max(1, sum(1 for r in ok if r[k] == v)):+5.0f}"
            f" ({sum(1 for r in rows if r[k] == v and r['fell'])} fell)" for v in GRID[k]))
    print("most progress:")
    for r in sorted(ok, key=lambda r: -r["fwd_mm"])[:12]:
        print("  " + " ".join(f"{k}={r[k]:g}" for k in GRID) +
              f"  fwd {r['fwd_mm']:+5.0f}  turn {r['turn_deg']:+4.0f}  double {r['double'] * 100:2.0f} %"
              f"  air {r['airborne'] * 100:2.0f} %  roll +-{r['roll_amp']:.1f}")


if __name__ == "__main__":
    main()
