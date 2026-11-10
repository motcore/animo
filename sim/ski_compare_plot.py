# ski_compare.py's output: the model's pieces fed with the simulator's own
# motion, over one second. FreeCAD's Python has matplotlib:
#   "C:/Program Files/FreeCAD 1.0/bin/python.exe" ski_compare_plot.py [out.png]

import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
rows = list(csv.DictReader(open(os.path.join(HERE, "out", "ski_compare.csv"))))
c = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
sel = (c["t"] > 5) & (c["t"] < 6)
t = c["t"][sel]

fig, ax = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
ax[0].plot(t, c["theta_deg"][sel], c="k")
ax[0].set_ylabel("rocking, deg")
ax[1].step(t, c["meas_state"][sel], c="#c0392b", where="post", label="simulator")
ax[1].step(t, c["model_state"][sel] * 0.85, c="#2e6fb7", where="post", label="model (x0.85 to show both)")
ax[1].set_yticks([-1, 0, 1], ["on toe", "both", "on heel"])
ax[1].set_ylabel("stance foot")
ax[1].legend(fontsize=8, loc="upper right")
ax[2].plot(t, c["wz_meas"][sel], c="#c0392b", label="simulator")
ax[2].plot(t, c["wz_model"][sel], c="#2e6fb7", label="model, inside the simulator's one-point spells")
ax[2].set_ylabel("yaw rate, deg/s")
ax[2].legend(fontsize=8, loc="upper right")
ax[3].plot(t, c["slide_r"][sel], c="#c0392b", label="right ski")
ax[3].plot(t, c["slide_l"][sel], c="#2e6fb7", label="left ski")
ax[3].set_ylabel("slide, mm")
ax[3].set_xlabel("time, s")
ax[3].legend(fontsize=8, loc="upper right")
for a in ax:
    a.grid(alpha=0.3)
ax[0].set_title("Model v2 fed the simulator's motion (two-point feet 100 mm, steps +-10 mm at 150 mm/s),\n"
                "one second of the steady walk", fontsize=10)
fig.tight_layout()
path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "out", "ski_compare.png")
fig.savefig(path, dpi=100)
print(path)
