# The ski duck, equations (ski_model.py) vs simulator (ski.py), two-point
# feet, steps +-10 mm at 150 mm/s. FreeCAD's Python has matplotlib:
#   "C:/Program Files/FreeCAD 1.0/bin/python.exe" ski_model_plot.py [out.png]
# The traces: out/ski_model_traces.csv (L 100 mm). The window: the runs of
# 2026-11-03 (notas O2.2-a a.11, a.13), rocking held at 9-12 s.

import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
rows = list(csv.DictReader(open(os.path.join(HERE, "out", "ski_model_traces.csv"))))
col = lambda src, k: np.array([float(r[k]) for r in rows if r["src"] == src])

WIN_L = [60, 80, 90, 100, 110, 120, 130]
WIN = {"simulator (MuJoCo)": ([0, 0, 0, 5.4, 4.9, 0, 0], "#c0392b", "s"),
       "equations, mu 0.8": ([4.0, 0.7, 0.5, 0, 0, 0, 0], "#2e6fb7", "o"),
       "equations, mu 0.45": ([8.1, 4.0, 3.7, 3.4, 0.8, 0.8, 0], "#7fa7d9", "^")}
STY = {"sim": ("#c0392b", 1.0, "simulator"), "eq0.8": ("#2e6fb7", 1.0, "equations, mu 0.8"),
       "eq0.45": ("#7fa7d9", 1.0, "equations, mu 0.45")}

fig = plt.figure(figsize=(11, 10))
gs = fig.add_gridspec(3, 2, height_ratios=[1, 1, 1.1])
a1 = fig.add_subplot(gs[0, :])
a2 = fig.add_subplot(gs[1, :], sharex=a1)
a3 = fig.add_subplot(gs[2, 0])
a4 = fig.add_subplot(gs[2, 1])
for src, (c, lw, lab) in STY.items():
    t = col(src, "t")
    a1.plot(t, col(src, "theta_deg"), c=c, lw=lw, label=lab)
    a2.plot(t, col(src, "yaw_deg") - col(src, "yaw_deg")[0], c=c, lw=lw, label=lab)
    a3.plot(t, col(src, "y_mm") - col(src, "y_mm")[0], c=c, lw=1.5, label=lab)
a1.set_ylabel("rocking, deg")
a1.set_title("Ski duck, two-point feet 100 mm apart, steps +-10 mm at 150 mm/s, released at 8 deg",
             fontsize=10)
a1.legend(fontsize=8, loc="upper right", ncol=3)
a2.set_ylabel("yaw (heading), deg")
a2.set_xlabel("time, s")
a1.set_xlim(0, 12)
a3.set_xlabel("time, s")
a3.set_ylabel("forward, mm")
a3.set_title("How far it goes", fontsize=10)
for lab, (vals, c, mk) in WIN.items():
    a4.plot(WIN_L, vals, "-" + mk, c=c, label=lab, ms=6)
a4.axvspan(97, 113, color="#c0392b", alpha=0.08)
a4.set_xlabel("foot length (toe to heel), mm")
a4.set_ylabel("rocking it keeps, deg")
a4.set_title("Which feet keep it going (0 = it dies)", fontsize=10)
a4.legend(fontsize=8)
for a in (a1, a2, a3, a4):
    a.grid(alpha=0.3)
fig.tight_layout()
path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "out", "ski_model.png")
fig.savefig(path, dpi=100)
print(path)
