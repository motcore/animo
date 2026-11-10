# One step from the release (ski.py one_step=..): the yaw of a single step,
# both skis / only the stance one / only the raised one / feet still.
# The runs: out/ski_onestep_{0..3}.csv. FreeCAD's Python has matplotlib:
#   "C:/Program Files/FreeCAD 1.0/bin/python.exe" ski_onestep_plot.py [out.png]

import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = "--model" in sys.argv          # overlay ski_model.run_pitch, dotted
sys.argv = [a for a in sys.argv if a != "--model"]
CASES = ((0, "feet still", "#7f8c8d"), (1, "both skis: right back, left forward", "k"),
         (2, "only the stance ski (right) back", "#c0392b"), (3, "only the raised ski (left) forward", "#2e6fb7"))
fig, ax = plt.subplots(4, 1, figsize=(9, 10), sharex=True)
for one, lab, c in CASES:
    a = np.genfromtxt(os.path.join(HERE, "out", f"ski_onestep_{one}.csv"), delimiter=",", names=True)
    t = a["t"] * 1000
    ax[0].plot(t, np.degrees(a["theta"]), c=c, lw=1.3, label=lab)
    ax[1].plot(t, np.degrees(a["yaw"]), c=c, lw=1.3)
    state = np.where((a["Nrh"] > 0.02) & (a["Nrt"] <= 0.02), 1, np.where((a["Nrt"] > 0.02) & (a["Nrh"] <= 0.02), -1, 0))
    state = np.where(a["theta"] > 0, state, np.nan)
    ax[2].step(t, state * (1 - 0.06 * one), c=c, where="post", lw=1.3)
    ax[3].plot(t, a["sR"] * 1000, c=c, lw=1.3)
    ax[3].plot(t, a["sL"] * 1000, c=c, lw=1.3, ls="--")
    mf = os.path.join(HERE, "out", f"ski_onestep_model_{one}.csv")
    if MODEL and os.path.exists(mf) and one in (1, 2):    # the equations, dotted
        b = np.genfromtxt(mf, delimiter=",", names=True)
        tb = b["t"] * 1000
        ax[0].plot(tb, np.degrees(b["theta"]), c=c, lw=1.6, ls=":")
        ax[1].plot(tb, np.degrees(b["yaw"]), c=c, lw=1.6, ls=":")
        st = np.where(b["theta"] > 0, np.sign(b["phi"]), np.nan)
        ax[2].step(tb, st * (0.9 - 0.06 * one), c=c, where="post", lw=1.6, ls=":")
ax[0].set_ylabel("rocking, deg")
ax[0].legend(fontsize=8)
ax[1].set_ylabel("heading (yaw), deg")
ax[2].set_yticks([-1, 0, 1], ["toe alone", "both", "heel alone"])
ax[2].set_ylabel("right foot\n(while on it)")
ax[3].set_ylabel("slide, mm\n(right solid, left dashed)")
ax[3].set_xlabel("time after the release, ms")
ax[3].set_xlim(0, 400)
for a_ in ax:
    a_.grid(alpha=0.3)
ax[0].set_title("Released at 8 deg onto the right foot, one step and hold (two-point feet 100 mm, "
                "10 mm at 150 mm/s)" + ("\nsolid: simulator; dotted: equations (model v2)" if MODEL else ""),
                fontsize=10)
fig.tight_layout()
path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "out", "ski_onestep.png")
fig.savefig(path, dpi=100)
print(path)
