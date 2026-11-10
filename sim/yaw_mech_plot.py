# The yaw duck's leg mechanism seen from above, through the cube (yaw_duck.py's
# default walk, k = 0): the bar on the bottom output turns, each whole leg
# rides a short rail at the bar's end, and a pin fixed under the body, on
# the other side of the axis, drives the leg along its rail through a slot
# in an arm. To scale (mm). FreeCAD's Python has matplotlib:
#   "C:/Program Files/FreeCAD 1.0/bin/python.exe" yaw_mech_plot.py [out.png]

import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

B = 23.5             # mm — the legs out from the axis (yaw_duck b)
G = 0.33 * 180 / math.pi   # mm per rad: 0.33 mm per deg of the bar
CUBE, BRAIN_Y = 28.0, 42.0


def rot(p, a):
    c, s = math.cos(a), math.sin(a)
    return np.array([c * p[0] - s * p[1], s * p[0] + c * p[1]])


def draw(ax, beta, faint=False):
    al = 0.25 if faint else 1.0
    lw = 1.0 if faint else 2.2
    for side, sgn in (("right", 1), ("left", -1)):
        end = rot((sgn * B, 0), beta)
        s = sgn * G * beta                      # the leg along its rail
        rail = [rot((sgn * B, y), beta) for y in (-9, 9)]
        ax.plot(*zip(*rail), c="#7f8c8d", lw=4 if not faint else 2, alpha=al, solid_capstyle="butt")
        leg = rot((sgn * B, s), beta)
        ax.add_patch(plt.Rectangle(leg - (3, 4), 6, 8, angle=0, color="#e67e22", alpha=al))
        # the arm: along the bar's x at the leg's y, across the axis to the
        # pin's slot on the other side (two heights: they cross)
        far = rot((-sgn * (G + 6), s), beta)
        ax.plot(*zip(leg, far), c="#e67e22", lw=lw, alpha=al, ls="-" if sgn > 0 else "--")
        slot = [rot((-sgn * (G + d), s), beta) for d in (-6, 6)]
        ax.plot(*zip(*slot), c="k", lw=5 if not faint else 2, alpha=0.35 * al, solid_capstyle="round")
        if not faint:
            ax.annotate(f"{side} leg on a carriage", leg, leg + (sgn * 6, 10), fontsize=8,
                        ha="left" if sgn > 0 else "right", arrowprops=dict(arrowstyle="-", lw=0.6))
    bar = [rot((x, 0), beta) for x in (-B, B)]
    ax.plot(*zip(*bar), c="#c0392b", lw=lw + 1, alpha=al)


fig, ax = plt.subplots(figsize=(8, 8.4))
ax.add_patch(plt.Rectangle((-CUBE, -CUBE), 2 * CUBE, 2 * CUBE, fc="#dfe6f1", ec="#7f8c8d"))
ax.add_patch(plt.Rectangle((-CUBE, CUBE), 2 * CUBE, 28, fc="#c8d3e8", ec="#7f8c8d"))
ax.text(0, CUBE + 14, "brain (front)", ha="center", fontsize=9)
ax.text(0, -CUBE + 3, "cube (see-through), seen from above", ha="center", fontsize=9)
draw(ax, 0.0, faint=True)
beta = math.radians(15)
draw(ax, beta)
for x in (-G, G):                       # pins fixed under the body
    ax.plot(x, 0, "o", c="k", ms=7)
ax.annotate(f"pins fixed to the body, {G:.1f} mm from the axis", (G, 0), (6, -20), fontsize=8,
            arrowprops=dict(arrowstyle="-", lw=0.6))
ax.plot(0, 0, "+", c="#c0392b", ms=14, mew=2)
ax.annotate("bottom output (vertical axis):\nthe bar turns +-15 deg", (0, 0), (-60, -45), fontsize=8,
            arrowprops=dict(arrowstyle="-", lw=0.6))
ax.annotate("rail at the bar's end", rot((B, 9), beta), (40, 30), fontsize=8,
            arrowprops=dict(arrowstyle="-", lw=0.6))
ax.annotate("arm with a slot: crosses under\nthe axis to the other side's pin", rot((-G - 2, G * beta), beta),
            (-75, 25), fontsize=8, arrowprops=dict(arrowstyle="-", lw=0.6))
ax.set_aspect("equal")
ax.set_xlim(-80, 80)
ax.set_ylim(-60, 65)
ax.grid(alpha=0.25)
ax.set_xlabel("x, mm (right +)")
ax.set_ylabel("y, mm (forward +)")
ax.set_title("Yaw duck legs, one servo: bar at +15 deg (bold) over the middle (faint).\n"
             f"Each leg slides {G * beta:.1f} mm on its rail ({0.33:g} mm per deg of the bar), "
             "the planted one back", fontsize=10)
fig.tight_layout()
path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "out", "yaw_mech.png")
fig.savefig(path, dpi=110)
print(path)
