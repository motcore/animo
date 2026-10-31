# The picture of a sole trace (sole_trace.py). Needs matplotlib, which the
# sim's venv lacks; FreeCAD's Python has it:
#   "C:/Program Files/FreeCAD 1.0/bin/python.exe" sole_plot.py out/sole_start

import csv
import json
import math
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

base = sys.argv[1]
info = json.load(open(base + ".json"))
rows = np.array([[float(x) for x in r] for r in list(csv.reader(open(base + "_samples.csv")))[1:]])
R = info["params"]["ball_r"]
mm = math.pi / 180 * R

fig = plt.figure(figsize=(13, 8))
gs = fig.add_gridspec(3, 2, width_ratios=[1, 1.4])
colours = {"right": "tab:red", "left": "tab:blue"}

# Left: the sole map, every contact coloured by phase, the mean curve on top
ax = fig.add_subplot(gs[:, 0])
for k, name in ((0, "right"), (1, "left")):
    f = rows[(rows[:, 2] == k) & (rows[:, 3] == 1)]
    sc = ax.scatter(f[:, 4] * mm, f[:, 5] * mm, c=f[:, 1], cmap="twilight", s=1,
                    alpha=0.15, vmin=0, vmax=360, rasterized=True)
    b = [x for x in info["feet"][name]["bins"] if "u" in x]
    u = [x["u"] * mm for x in b]
    v = [x["v"] * mm for x in b]
    ax.plot(u, v, "-o", ms=3, lw=1.5, color=colours[name], label=f"{name} (mean)")
    ax.annotate("", xy=(u[1], v[1]), xytext=(u[0], v[0]),
                arrowprops=dict(arrowstyle="->", color=colours[name]))
ax.set_xlabel("u: side to side on the sole, mm (+ outward)")
ax.set_ylabel("v: fore and aft on the sole, mm (+ toe)")
ax.set_aspect("equal")
ax.grid(alpha=0.3)
ax.legend(loc="lower right")
cb = fig.colorbar(sc, ax=ax, shrink=0.6)
cb.set_label("phase, deg (90 = right leg most forward)")
cb.solve_alpha = False
ax.set_title("Contact path on the sole (left foot mirrored)")

# Right: u, v and the load against the phase, mean and spread
for row, (key, label) in enumerate((("u", "u, mm"), ("v", "v, mm"))):
    ax = fig.add_subplot(gs[row, 1])
    for name in ("right", "left"):
        b = [x for x in info["feet"][name]["bins"] if "u" in x]
        phi = np.array([x["phi"] for x in b])
        m_ = np.array([x[key] for x in b]) * mm
        s_ = np.array([x["s" + key] for x in b]) * mm
        ax.plot(phi, m_, "-", color=colours[name], label=name)
        ax.fill_between(phi, m_ - s_, m_ + s_, color=colours[name], alpha=0.2)
    ax.set_ylabel(label)
    ax.set_xlim(0, 360)
    ax.set_xticks(range(0, 361, 45))
    ax.grid(alpha=0.3)
    if row == 0:
        ax.legend(loc="upper right")
        ax.set_title("Mean over the cycles, band = +-1 std (cycle to cycle)")
ax = fig.add_subplot(gs[2, 1])
for name in ("right", "left"):
    b = info["feet"][name]["bins"]
    phi = [x["phi"] for x in b]
    ax.plot(phi, [x["contact"] * 100 for x in b], color=colours[name], label=f"{name}: on the floor, %")
    ax.plot(phi, [x["load"] / info["result"]["weight_N"] * 100 for x in b], "--", color=colours[name], alpha=0.6,
            label=f"{name}: load (mean), % of weight")
ax.set_xlabel("phase, deg")
ax.set_xlim(0, 360)
ax.set_xticks(range(0, 361, 45))
ax.set_ylim(0, 110)
ax.grid(alpha=0.3)
ax.legend(fontsize=7, loc="lower right")

r = info["result"]
fr, fl = info["feet"]["right"], info["feet"]["left"]
fig.suptitle(
    f"Sole trace — R {R:g} mm, gap {info['params']['ball_gap']:g}, {info['params']['freq']:g} Hz, "
    f"legs ±{info['params']['leg_deg']:g}° — walk {r['fwd_mm']:+.0f} mm, turn {r['turn_deg']:+.0f}° "
    f"| spread {fr['spread_mm']:.1f}/{fl['spread_mm']:.1f} mm, R vs L {info['symmetry_rms_deg'] * mm:.1f} mm rms",
    fontsize=10)
fig.tight_layout()
fig.savefig(base + ".png", dpi=110)
print(base + ".png")
