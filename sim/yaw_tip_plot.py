# The yaw duck's planted foot going onto its toe or heel (rigid legs, the
# default walk): one second of out/yaw_tip.csv. FreeCAD's Python:
#   "C:/Program Files/FreeCAD 1.0/bin/python.exe" yaw_tip_plot.py [out.png]

import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
a = np.genfromtxt(os.path.join(HERE, "out", "yaw_tip.csv"), delimiter=",", names=True)
sel = (a["t"] > 5) & (a["t"] < 6)
t = a["t"][sel]
W = 0.233 * 9.81
fig, ax = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
ax[0].plot(t, a["roll"][sel], c="k")
ax[0].set_ylabel("rocking, deg\n(+ on the right foot)")
for i, (side, c) in enumerate((("R", "#c0392b"), ("L", "#2e6fb7"))):
    a_ = ax[1 + i]
    a_.fill_between(t, 0, a[side + "t"][sel] / W, color=c, alpha=0.8, label="toe")
    a_.fill_between(t, 0, -a[side + "h"][sel] / W, color=c, alpha=0.35, label="heel (drawn downward)")
    a_.axhline(0, c="k", lw=0.6)
    a_.set_ylabel(f"{'right' if side == 'R' else 'left'} foot load\n/ weight")
    a_.legend(fontsize=8, loc="upper right")
ax[3].plot(t, a["slideR"][sel], c="#c0392b", label="right leg")
ax[3].plot(t, a["slideL"][sel], c="#2e6fb7", label="left leg")
ax[3].set_ylabel("leg slide, mm")
ax[3].set_xlabel("time, s")
ax[3].legend(fontsize=8, loc="upper right")
for a_ in ax:
    a_.grid(alpha=0.3)
ax[0].set_title("Yaw duck, rigid legs, default walk: when only the toe or only the heel carries the planted foot",
                fontsize=10)
fig.tight_layout()
path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "out", "yaw_tip.png")
fig.savefig(path, dpi=100)
print(path)
