# The picture of sagittal.py: progress per cycle against the stance phase
# (model points, the ideal 4 A H cos(phi - 180) line), the body's pitch,
# and what double support does. FreeCAD's Python has matplotlib:
#   "C:/Program Files/FreeCAD 1.0/bin/python.exe" sagittal_plot.py

import csv
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
rows = list(csv.DictReader(open(os.path.join(OUT, "sagittal.csv"))))
duty = list(csv.DictReader(open(os.path.join(OUT, "sagittal_duty.csv"))))
H = float(rows[0]["H"])
cols = {"92.5": "tab:blue", "140": "tab:green", "186": "tab:red", "400": "tab:orange"}

fig, axs = plt.subplots(1, 3, figsize=(17, 4.8))
ax = axs[0]
phi = np.linspace(0, 360, 200)
for A, ls in (("10.0", ":"), ("15.0", "-"), ("20.0", "--")):
    ax.plot(phi, 4 * math.radians(float(A)) * H * np.cos(np.radians(phi - 180)), "k", ls=ls, lw=1,
            label=f"4 A H cos(phi - 180), A {float(A):g} deg")
    for R, c in cols.items():
        sel = [r for r in rows if r["Rs"] == R and r["A"] == A and r["fell"] == "0"]
        ax.plot([float(r["phi"]) for r in sel], [float(r["fwd_cycle"]) for r in sel], "o", color=c, ms=4,
                label=f"model, sole R {R}" if A == "15.0" else None)
ax.axhline(0, color="k", lw=0.5)
ax.set_xticks(range(0, 361, 45))
ax.set_xlabel("stance phase phi, deg (180: the foot down while its leg sweeps back)")
ax.set_ylabel("progress per cycle, mm")
ax.set_title(f"Step = 4 x swing x hip height ({H:g} mm), whatever the sole")
ax.legend(fontsize=7)
ax.grid(alpha=0.3)

ax = axs[1]
for R, c in cols.items():
    sel = [r for r in rows if r["Rs"] == R and r["A"] == "15.0" and r["fell"] == "0"]
    ax.plot([float(r["phi"]) for r in sel], [float(r["pitch_amp"]) for r in sel], "o-", color=c, ms=4,
            label=f"sole R {R}")
ax.set_xticks(range(0, 361, 45))
ax.set_xlabel("stance phase phi, deg")
ax.set_ylabel("body pitch, +- deg")
ax.set_title("Pitch (legs +-15 deg; missing points: fell)")
ax.legend(fontsize=8)
ax.grid(alpha=0.3)

ax = axs[2]
for R, c in (("92.5", "tab:blue"), ("186", "tab:red")):
    for du, mk in (("0.5", "o"), ("0.6", "s"), ("0.7", "^"), ("0.8", "v")):
        sel = [r for r in duty if r["Rs"] == R and r["duty"] == du and r["fell"] == "0"]
        ax.plot([float(r["phi"]) for r in sel], [float(r["fwd_cycle"]) for r in sel], mk, color=c, ms=5,
                alpha=0.4 + 0.6 * (du == "0.5"),
                label=f"R {R}, both feet down {200 * (float(du) - 0.5):.0f} % of the time")
ax.axhline(0, color="k", lw=0.5)
ax.set_xticks(range(0, 361, 45))
ax.set_xlabel("stance phase phi, deg")
ax.set_ylabel("progress per cycle, mm")
ax.set_title("Double support: the step stays, but most phases fall")
ax.legend(fontsize=7)
ax.grid(alpha=0.3)
fig.suptitle("Sagittal plane alone: body + two legs (+-A at 2 Hz), each foot allowed down only in its share of the cycle",
             fontsize=11)
fig.tight_layout()
path = os.path.join(OUT, "sagittal.png")
fig.savefig(path, dpi=105)
print(path)
