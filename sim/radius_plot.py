# A knob against the legs' rhythm (support.py --grid <knob>=... freq=...):
# forward progress, double support and rocking as maps, and the best rhythm
# for each value. FreeCAD's Python has matplotlib:
#   "C:/Program Files/FreeCAD 1.0/bin/python.exe" radius_plot.py out/radius_freq
#   "C:/Program Files/FreeCAD 1.0/bin/python.exe" radius_plot.py out/amp_freq leg_deg

import csv
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

base = sys.argv[1]
key = sys.argv[2] if len(sys.argv) > 2 else "ball_r"
LABEL = dict(ball_r="foot sphere radius, mm", leg_deg="legs' swing, +- deg")
rows = list(csv.DictReader(open(base + ".csv")))
Rs = sorted({float(r[key]) for r in rows})
fs = sorted({float(r["freq"]) for r in rows})


def grid(col):
    g = np.full((len(Rs), len(fs)), np.nan)
    for r in rows:
        if int(r["fell"]):
            continue
        g[Rs.index(float(r[key])), fs.index(float(r["freq"]))] = float(r[col])
    return g


fwd, dbl, roll = grid("fwd_mm") / 1000, grid("double") * 100, grid("roll_amp")
fig, axs = plt.subplots(1, 3, figsize=(15, 4.8))
for ax, g, title, cmap in ((axs[0], fwd, "forward in 30 s, m", "viridis"),
                           (axs[1], dbl, "double support, % of the time", "magma_r"),
                           (axs[2], roll, "rocking amplitude, deg", "cividis")):
    im = ax.imshow(g, origin="lower", aspect="auto", cmap=cmap,
                   extent=(fs[0] - 0.1, fs[-1] + 0.1, -0.5, len(Rs) - 0.5))
    ax.set_yticks(range(len(Rs)), [f"{r:g}" for r in Rs])
    ax.set_xlabel("legs' rhythm, Hz")
    ax.set_ylabel(LABEL.get(key, key))
    ax.set_title(title)
    fig.colorbar(im, ax=ax, shrink=0.85)
    for i, r in enumerate(Rs):          # the best rhythm for each radius
        row = np.nan_to_num(fwd[i], nan=-1e9)
        ax.plot(fs[int(np.argmax(row))], i, "w*", ms=12, mec="k")
fig.suptitle(f"Does the best rhythm follow the {LABEL.get(key, key).split(',')[0]}? (star: most progress for that value)",
             fontsize=11)
fig.tight_layout()
fig.savefig(base + ".png", dpi=110)
print(base + ".png")
