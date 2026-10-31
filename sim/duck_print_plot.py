# The duckling's whole footprint (out/duck_print.csv, from 54 nudged starts):
# where the right sole touches (left mirrored), walking vs the first 10 s,
# and the runs that fell. FreeCAD's Python has matplotlib:
#   "C:/Program Files/FreeCAD 1.0/bin/python.exe" duck_print_plot.py

import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
rows = list(csv.DictReader(open(os.path.join(OUT, "duck_print.csv"))))
t = np.array([float(r["t"]) for r in rows])
x = np.array([float(r["x_out"]) for r in rows])
y = np.array([float(r["y_toe"]) for r in rows])
fell = np.array([r["fwd_mm"] == "" for r in rows])
OUTER, HEEL, TOE = 60, 40, 80

fig, axs = plt.subplots(1, 3, figsize=(15, 6.2), sharey=True)
sets = (("walking (t > 10 s)", (t > 10) & ~fell), ("start (t < 10 s)", (t <= 10) & ~fell),
        ("runs that fell", fell))
for ax, (title, sel) in zip(axs, sets):
    h, xe, ye = np.histogram2d(x[sel], y[sel], bins=(np.arange(0, OUTER + 2, 2), np.arange(-HEEL, TOE + 2, 2)))
    ax.pcolormesh(xe, ye, np.log10(h.T + 1), cmap="magma_r")
    ax.add_patch(plt.Rectangle((0.5, -HEEL), OUTER - 0.5, HEEL + TOE, fill=False, lw=1.2, ec="k"))
    for q, ls in ((99.0, ":"), (99.9, "--")):           # how far each row reaches out
        ys = np.arange(-HEEL, TOE, 4)
        reach = [np.percentile(x[sel][(y[sel] >= a) & (y[sel] < a + 4)], q)
                 if ((y[sel] >= a) & (y[sel] < a + 4)).sum() > 50 else np.nan for a in ys]
        ax.plot(reach, ys + 2, "b", ls=ls, lw=1.2, label=f"reach out, {q:g} % of points")
    ax.set_title(f"{title}: {sel.sum()} points")
    ax.set_xlabel("x, mm outward from the middle")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
axs[0].set_ylabel("y, mm toward the toe (0 = under the hip)")
axs[0].legend(fontsize=8, loc="lower right")
fig.suptitle("Duckling sole footprint, right foot (left mirrored): hip 92, R 130 / 186, gap 7, 1.2 Hz, "
             "start phase 180; the box is the 60 x (40 + 80) sole; colour = log10 contact count", fontsize=10)
fig.tight_layout()
path = os.path.join(OUT, "duck_print.png")
fig.savefig(path, dpi=100)
print(path)
