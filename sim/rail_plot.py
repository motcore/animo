# The rail foot (trace_foot.py) seen from below on the printed sole's outline.
# FreeCAD's Python has matplotlib:
#   "C:/Program Files/FreeCAD 1.0/bin/python.exe" rail_plot.py [out.png [toes]]

import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
rail = json.load(open(os.path.join(HERE, "out", "rail_path.json")))
a = np.array(rail["points"])

# The printed outline (trace_foot.DUCK): out 60, heel 40, toe 80, the outer
# edge at 60 between y -10 and 25, curving in to x 50 at the heel, 15 at the toe
IN, OUT_, HEEL, TOE, W0, W1, HX, TX = 0.5, 60, 40, 80, -10, 25, 50, 15
ys = np.linspace(-HEEL, TOE, 241)
out = np.full_like(ys, OUT_)
for end, xe, sel in ((-HEEL, HX, ys < W0), (TOE, TX, ys > W1)):
    edge = W0 if end < 0 else W1
    u = (ys[sel] - edge) / (end - edge)
    out[sel] = xe + (OUT_ - xe) * np.sqrt(np.maximum(1 - u * u, 0))

fig, ax = plt.subplots(figsize=(5.4, 7.6))
ax.fill(np.r_[out, [IN, IN]], np.r_[ys, [TOE, -HEEL]], fc="#f3d9b5", ec="#b07a3a", lw=1.2,
        label="printed sole (outline)")
ax.plot(a[:, 0], a[:, 1], "-", c="#c0392b", lw=3, solid_capstyle="round",
        label=f"rail: the contact path, {rail['length_mm']:.0f} mm")
for k in (8, 30, 55):
    ax.annotate("", a[k + 2, :2], a[k, :2], arrowprops=dict(arrowstyle="-|>", color="#c0392b", lw=1.5))
fan = rail.get("toes", {}).get(sys.argv[2], []) if len(sys.argv) > 2 else []
for k, t in enumerate(fan):
    b = np.array(t["points"])
    ax.plot(b[:, 0], b[:, 1], "-", c="#2e6fb7", lw=3, solid_capstyle="round",
            label="toes" if k == 0 else None)
ax.plot(*a[0, :2], "o", c="#c0392b", ms=7)
ax.annotate("touch-down", a[0, :2], (a[0, 0] + 4, a[0, 1] + 6), fontsize=9)
ax.annotate("lift-off", a[-1, :2], (a[-1, 0] + 3, a[-1, 1] + 6), fontsize=9)
ax.plot(0, 0, "+", c="k", ms=12)
ax.annotate("under the hip", (0, 0), (2, -5), fontsize=8)
ax.set_xlim(-5, 65)
ax.set_ylim(-HEEL - 5, TOE + 5)
ax.set_aspect("equal")
ax.grid(alpha=0.3)
ax.set_xlabel("x, mm outward from the middle")
ax.set_ylabel("y, mm toward the toe")
ax.set_title("Right foot from below: rail foot = only the contact path"
             f"{f' + {len(fan)} toes' if fan else ''}\n"
             "(today's gait: slow start, then 1.2 Hz, +-15 deg)", fontsize=10)
ax.legend(loc="lower right", fontsize=8)
fig.tight_layout()
path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "out", "rail_foot.png")
fig.savefig(path, dpi=110)
print(path)
