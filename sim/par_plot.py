# The parallelogram leg as the simulator has it (yaw_duck.py par_L, par_s0),
# side view to scale: notas/patito_paralelogramo.png
#   "C:/Program Files/FreeCAD 1.0/bin/python.exe" par_plot.py
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

L, s0, A, d = 130.0, 3.0, 30.0, 18.0     # link length, lowest point, step, link spacing (drawn)
foot = 100.0
hip = 130.0                               # pivots this high above the floor (feet down at s0)


def pose(s):
    """Links tilted so the foot is s ahead; it rises z on the arc."""
    phi = math.asin((s - s0) / L)
    z = L * (1 - math.cos(phi))
    return phi, z


fig, (ax, ax2) = plt.subplots(1, 2, figsize=(12, 5.6), gridspec_kw=dict(width_ratios=[1.5, 1]))
cols = {-A: "#2a7ab9", s0: "#333333", A: "#c0392b"}
for s in (-A, s0, A):
    phi, z = pose(s)
    c = cols[s]
    for px in (-d / 2, d / 2):            # two parallel links from the body to the foot block
        x0, y0 = px + s0, hip
        x1, y1 = x0 + L * math.sin(phi), hip - L * math.cos(phi)
        ax.plot([x0, x1], [y0, y1], color=c, lw=2.2 if s == s0 else 1.4, alpha=1 if s == s0 else 0.8)
    cx, cy = s0 + L * math.sin(phi), hip - L * math.cos(phi)
    ax.plot([cx - d / 2, cx + d / 2], [cy, cy], color=c, lw=3)         # the block the links share
    ax.add_patch(Rectangle((cx - foot / 2, cy), foot, 3, color=c, alpha=0.35 if s != s0 else 0.6))
    lab = {-A: f"leg back ({-A:.0f} mm): foot {z:.1f} mm up",
           s0: f"lowest point (s0 = {s0:.0f} mm): on the floor",
           A: f"leg forward (+{A:.0f} mm): foot {z:.1f} mm up"}[s]
    ax.plot([], [], color=c, lw=2, label=lab)
ax.plot([-d / 2 + s0, d / 2 + s0], [hip, hip], "ks", ms=6)
ax.text(s0 + d / 2 + 4, hip - 2, "pivots on the body (cube)", fontsize=9)
ax.axhline(0, color="0.5", lw=1)
ax.text(-95, -9, "floor", color="0.4", fontsize=9)
ax.annotate("", xy=(s0 - d / 2 - 22, hip), xytext=(s0 - d / 2 - 22, 0), arrowprops=dict(arrowstyle="<->", color="0.5"))
ax.text(s0 - d / 2 - 52, hip / 2, f"L = {L:.0f} mm", color="0.3", fontsize=9)
ax.set_aspect("equal")
ax.set_xlim(-110, 110)
ax.set_ylim(-20, 185)
ax.set_xlabel("fore-aft (mm, + forward)")
ax.set_ylabel("height (mm)")
ax.set_title("Parallelogram leg, side view (to scale)")
ax.legend(fontsize=8, loc="upper left")

# the foot's rise along the step, magnified
ss = [x / 2 for x in range(-80, 81)]
for s0_, c in ((3.0, "#c0392b"), (0.0, "0.5"), (15.0, "#8e44ad")):
    zs = [L * (1 - math.cos(math.asin((s - s0_) / L))) - L * (1 - math.cos(math.asin((0 - s0_) / L))) for s in ss]
    ax2.plot(ss, zs, color=c, lw=2 if s0_ == 3 else 1.3, label=f"s0 = {s0_:.0f} mm")
ax2.axvspan(-A, A, color="0.9")
ax2.text(-A + 1, 9.3, f"step +-{A:.0f}", fontsize=8, color="0.4")
ax2.axhline(0, color="0.7", lw=0.8)
ax2.set_xlim(-40, 40)
ax2.set_ylim(-2, 10)
ax2.set_xlabel("leg place s (mm)")
ax2.set_ylabel("foot height vs. the middle (mm)")
ax2.set_title("How much the foot rises (magnified)")
ax2.legend(fontsize=8)
ax2.text(-38, -1.6, "one leg forward + the other back: different heights when s0 != 0", fontsize=7.5, color="0.35")
fig.tight_layout()
out = r"C:\Users\jjavi\dev\motcore\notas\patito_paralelogramo.png"
fig.savefig(out, dpi=130)
print(out)
