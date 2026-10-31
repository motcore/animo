# The picture of heading.py: the heading against time and the path on the
# floor, every nudged copy of each case. FreeCAD's Python has matplotlib:
#   "C:/Program Files/FreeCAD 1.0/bin/python.exe" heading_plot.py out/heading

import csv
import json
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

base = sys.argv[1]
rows = list(csv.DictReader(open(base + ".csv")))
cases = list(dict.fromkeys(r["case"] for r in rows))
colours = ["tab:blue", "tab:orange", "tab:green", "tab:red"]

fig, axs = plt.subplots(2, len(cases), figsize=(4.2 * len(cases), 8))
for j, case in enumerate(cases):
    sel = [r for r in rows if r["case"] == case]
    for k in sorted({r["copy"] for r in sel}):
        c = [r for r in sel if r["copy"] == k]
        col = colours[int(k) % len(colours)]
        axs[0, j].plot([float(r["t"]) for r in c], [float(r["heading_deg"]) for r in c],
                       color=col, lw=1, label=f"nudge {k}e-6 rad/s")
        axs[1, j].plot([float(r["x_mm"]) / 1000 for r in c], [float(r["y_mm"]) / 1000 for r in c],
                       color=col, lw=1)
        axs[1, j].plot(float(c[-1]["x_mm"]) / 1000, float(c[-1]["y_mm"]) / 1000, "o", color=col, ms=4)
    axs[0, j].set_title(case)
    axs[0, j].set_xlabel("time, s")
    axs[0, j].set_ylabel("heading, deg (+ left)")
    axs[0, j].axhline(0, color="k", lw=0.5)
    axs[0, j].grid(alpha=0.3)
    axs[1, j].plot(0, 0, "k^", ms=7)
    axs[1, j].set_xlabel("x, m")
    axs[1, j].set_ylabel("y (start heading), m")
    axs[1, j].set_aspect("equal", adjustable="datalim")
    axs[1, j].grid(alpha=0.3)
axs[0, 0].legend(fontsize=7)
knobs = json.load(open(base + ".json"))
fig.suptitle("Heading along a 120 s walk — the same gait, started with a tiny yaw nudge\n" +
             ", ".join(f"{k} {v:g}" for k, v in knobs.items()), fontsize=10)
fig.tight_layout()
fig.savefig(base + ".png", dpi=100)
print(base + ".png")
