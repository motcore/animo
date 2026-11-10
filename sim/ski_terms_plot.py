# One rocking cycle of the ski duck, averaged (ski_terms.py): what the
# stance foot does. FreeCAD's Python has matplotlib:
#   "C:/Program Files/FreeCAD 1.0/bin/python.exe" ski_terms_plot.py [out.png]

import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
rows = list(csv.DictReader(open(os.path.join(HERE, "out", "ski_terms_phase.csv"))))
c = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
ph = c["phase"] * 100

fig, ax = plt.subplots(5, 1, figsize=(9, 11), sharex=True)
ax[0].plot(ph, c["th"], c="k")
ax[0].set_ylabel("rocking, deg\n(+ on right foot)")
ax[1].plot(ph, c["wz"], c="#8e44ad")
ax[1].set_ylabel("yaw rate, deg/s")
ax[2].plot(ph, c["Nt"], c="#c0392b", label="toe")
ax[2].plot(ph, c["Nh"], c="#2e6fb7", label="heel")
ax[2].set_ylabel("stance foot load\n/ weight")
ax[2].legend(fontsize=8)
ax[3].fill_between(ph, 0, c["one"] * 100, color="#e67e22", alpha=0.5)
ax[3].set_ylabel("on a single\npoint, % of cycles")
ax[3].set_ylim(0, 100)
ax[4].plot(ph, c["sR"], c="#c0392b", label="right ski")
ax[4].plot(ph, c["sL"], c="#2e6fb7", label="left ski")
ax[4].set_ylabel("slide, mm\n(+ forward)")
ax[4].legend(fontsize=8)
ax[4].set_xlabel("rocking cycle, % (0 = lands on the right foot)")
for a in ax:
    a.grid(alpha=0.3)
    a.axvline(50, c="k", lw=0.6, ls=":")
ax[0].set_title("Ski duck, two-point feet 100 mm, steps +-10 mm at 150 mm/s: one cycle, averaged over 13\n"
                "(the stance foot: the right one from 0 to 50 %, the left one after)", fontsize=10)
fig.tight_layout()
path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "out", "ski_terms.png")
fig.savefig(path, dpi=100)
print(path)
