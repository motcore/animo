# The ski duck released tilted (ski.py): the simulator against Housner's
# rocking-block equations. FreeCAD's Python has matplotlib:
#   "C:/Program Files/FreeCAD 1.0/bin/python.exe" ski_plot.py [out.png]

import csv
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
rows = list(csv.DictReader(open(os.path.join(OUT, "ski_release.csv"))))
info = json.load(open(os.path.join(OUT, "ski_release.json")))
k = info["block"]
data = {s: np.array([[float(r["t"]), float(r["theta_deg"])] for r in rows if r["src"] == s])
        for s in ("sim", "eq")}

fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 7.5))
for s, c, lw, lab in (("eq", "#2e6fb7", 2.2, "equations (rocking block)"),
                      ("sim", "#c0392b", 1.2, "simulator (MuJoCo)")):
    a1.plot(data[s][:, 0], data[s][:, 1], c=c, lw=lw, label=lab)
al = math.degrees(k["alpha"])
for y in (al, -al):
    a1.axhline(y, c="k", ls=":", lw=0.8)
a1.annotate(f"alpha = {al:.1f} deg: tips over past this", (0.55, al - 1.6), fontsize=8)
a1.set_xlim(0, 2.2)
a1.set_xlabel("time, s")
a1.set_ylabel("tilt, deg (+ = on the right ski's edge)")
a1.set_title(f"Ski duck released at {info['tilt_deg']:g} deg from rest: rocks faster and faster, "
             f"stops in finite time\nm {k['M'] * 1000:.0f} g, b {k['b'] * 1000:.1f} mm, "
             f"h {k['h'] * 1000:.1f} mm, r {k['r']:.3f} (energy kept per landing "
             f"{k['r'] ** 2 * 100:.0f} %)", fontsize=10)
a1.legend(fontsize=8, loc="upper right")
a1.grid(alpha=0.3)

for s, hits, c, mk in (("eq", info["hits_eq"], "#2e6fb7", "o"), ("sim", info["hits_sim"], "#c0392b", "x")):
    h = np.array(hits)
    half = np.diff(h)[:15]
    a2.plot(np.arange(1, len(half) + 1), 1 / (2 * half), mk, c=c, ms=6,
            label=("equations" if s == "eq" else "simulator"))
a2.set_xlabel("landing number")
a2.set_ylabel("rocking rate, Hz (1 / two half-rocks)")
a2.set_title("Each landing loses energy, the rock shrinks, the rhythm rises", fontsize=10)
a2.legend(fontsize=8)
a2.grid(alpha=0.3)
fig.tight_layout()
path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(OUT, "ski_release.png")
fig.savefig(path, dpi=100)
print(path)
