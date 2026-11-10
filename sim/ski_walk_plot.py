# The ski duck stepping in time with its rocking (ski.py --set step_mm=..):
# tilt, the skis' slides and how far the body went. FreeCAD's Python:
#   "C:/Program Files/FreeCAD 1.0/bin/python.exe" ski_walk_plot.py [out.png]

import csv
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
rows = [r for r in csv.DictReader(open(os.path.join(OUT, "ski_walk.csv"))) if r["src"] == "sim"]
info = json.load(open(os.path.join(OUT, "ski_walk.json")))
col = lambda k: np.array([float(r[k]) for r in rows])
t = col("t")
eq = [r for r in csv.DictReader(open(os.path.join(OUT, "ski_walk.csv"))) if r["src"] == "eq"]

fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
a1.plot([float(r["t"]) for r in eq], [float(r["theta_deg"]) for r in eq], c="#2e6fb7", lw=2,
        label="equations (skis held)")
a1.plot(t, col("theta_deg"), c="#c0392b", lw=1.2, label="simulator, stepping")
a1.set_ylabel("tilt, deg (+ = on right ski)")
a1.legend(fontsize=8)
a2.plot(t, col("skiR_mm"), c="#c0392b", label="right ski")
a2.plot(t, col("skiL_mm"), c="#2e6fb7", label="left ski")
a2.set_ylabel("ski slide, mm (+ = forward)")
a2.legend(fontsize=8)
a3.plot(t, col("y_mm") - col("y_mm")[0], c="k")
a3.set_ylabel("body forward, mm")
a3.set_xlabel("time, s")
for a in (a1, a2, a3):
    a.grid(alpha=0.3)
a1.set_xlim(0, 2.0)
p = info["params"]
a1.set_title(f"Ski duck released at {info['tilt_deg']:g} deg, skis stepping +-{p['step_mm']:g} mm at "
             f"{p['slide_v']:g} mm/s in time with the rocking\n(down ski slides back, raised ski "
             f"forward): the rocking still dies, and the walk with it", fontsize=10)
fig.tight_layout()
path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(OUT, "ski_walk.png")
fig.savefig(path, dpi=100)
print(path)
