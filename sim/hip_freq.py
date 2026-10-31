# Longer legs against the legs' rhythm: does the alternation threshold drop
# like a pendulum's (f ~ 1 / sqrt(leg))? Two series:
#   fixed   the feet as they are (R 105)
#   scaled  the feet grown with the leg: R and the cap's extent x hip / 52
# The leg meshes come from the macro (ANIMO_HIP); every run to a CSV.
#
#   python hip_freq.py

import csv
import math
import multiprocessing as mp
import os

import numpy as np

from support import one

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")

HIPS = [52, 72, 92, 112]
FREQS = [round(1.2 + 0.2 * i, 1) for i in range(13)]       # 1.2 .. 3.6 Hz
FOOT = dict(ball_r=105, cap_in=0.5, cap_out=50, cap_heel=45, cap_toe=40)


def jobs():
    for series in ("fixed", "scaled"):
        for h in HIPS:
            k = h / 52 if series == "scaled" else 1.0
            foot = {key: v * k if key != "cap_in" else v for key, v in FOOT.items()}
            for f in FREQS:
                yield dict(foot, hip=h, freq=f, series=series)


def run(job):
    over = {k: v for k, v in job.items() if k != "series"}
    out = one(over)
    out["series"] = job["series"]
    return out


def main():
    with mp.Pool() as pool:
        rows = pool.map(run, list(jobs()))
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, "hip_freq.csv")
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(path)
    print("double % (fwd m in 30 s), F = fell")
    print("series hip  " + " ".join(f"{f:>9g}" for f in FREQS))
    for series in ("fixed", "scaled"):
        for h in HIPS:
            line, onset = [], None
            for f in FREQS:
                r = next(x for x in rows if x["series"] == series and x["hip"] == h and x["freq"] == f)
                if r["fell"]:
                    line.append("    F    ")
                    continue
                if onset is None and r["double"] < 0.05:
                    onset = f
                line.append(f"{r['double'] * 100:3.0f}({r['fwd_mm'] / 1000:+4.1f})")
            pend = 2.8 * math.sqrt(52 / h)
            print(f"{series:6s} {h:<4d} " + " ".join(line) +
                  f"  onset {onset} Hz (pendulum guess {pend:.1f})")


if __name__ == "__main__":
    main()
