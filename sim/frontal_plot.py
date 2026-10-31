# The picture of frontal.py: the rocking's frequency against its amplitude
# for the two cubes (model points, theory lines), and the frequency at a
# +-15 deg swing over radius, gap and centre-of-mass height. FreeCAD's
# Python has matplotlib:
#   "C:/Program Files/FreeCAD 1.0/bin/python.exe" frontal_plot.py

import csv
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
G = 9.81


def f_A(R, h, gap, k, A):
    """frontal.theory's swing frequency (Hz) at amplitude A (deg)."""
    a, b = gap / 2, R - h
    I = (k * k + a * a + h * h) / 1000
    A = math.radians(A)
    th = np.linspace(0, A, 4001)[:-1]
    U = lambda t: a * np.sin(t) - b * np.cos(t)
    v = np.sqrt(np.maximum(2 * G * (U(A) - U(th)) / I, 1e-12))
    trap = getattr(np, "trapezoid", None) or np.trapz      # numpy 1 or 2
    return 1 / (4 * trap(1 / v, th))


cubes = list(csv.DictReader(open(os.path.join(OUT, "frontal_cubes.csv"))))
sweep = [r for r in csv.DictReader(open(os.path.join(OUT, "frontal_sweep.csv"))) if r["lean"] == "15"]

fig, axs = plt.subplots(1, 4, figsize=(18, 4.6), gridspec_kw=dict(width_ratios=[1.3, 1, 1, 1]))
ax = axs[0]
for name, col in (("hip52", "tab:blue"), ("hip92", "tab:red")):
    rs = [r for r in cubes if r["name"] == name]
    c = rs[0]
    A = np.linspace(2, 28, 60)
    ax.plot(A, [f_A(float(c["R"]), float(c["h"]), float(c["gap"]), float(c["k"]), x) for x in A],
            color=col, label=f"{name}: R {float(c['R']):.0f}, h {float(c['h']):.0f}, gap {float(c['gap']):.0f} (theory)")
    pts = [(float(r["amp"]), float(r["f_sim"])) for r in rs if r["f_sim"] not in ("nan", "")]
    ax.plot(*zip(*pts), "o", color=col, ms=5, label=f"{name}: model, released from a lean")
ax.axhspan(1.9, 2.1, color="tab:red", alpha=0.1)
ax.text(27.5, 2.0, "hip-92 gait, 2 Hz", ha="right", va="center", fontsize=8, color="tab:red")
ax.set_xlabel("rocking amplitude, +- deg")
ax.set_ylabel("rocking frequency, Hz")
ax.set_title("Smaller swings rock faster (V-shaped well)")
ax.grid(alpha=0.3)
ax.legend(fontsize=7)

for ax, h in zip(axs[1:], ("55", "73", "90")):
    for R, col in zip(("120", "150", "186", "240"), ("tab:purple", "tab:green", "tab:red", "tab:orange")):
        rs = sorted((r for r in sweep if r["R"].split(".")[0] == R and r["h"].split(".")[0] == h),
                    key=lambda r: float(r["gap"]))
        g = [float(r["gap"]) for r in rs]
        ax.plot(g, [float(r["f_sim"]) for r in rs], "o-", color=col, ms=4, label=f"R {R}")
    ax.set_title(f"centre of mass {h} mm up")
    ax.set_xlabel("gap between the feet's centres, mm")
    ax.set_ylabel("frequency at +-15 deg, Hz")
    ax.set_ylim(0.8, 4.1)
    ax.grid(alpha=0.3)
axs[1].legend(fontsize=8)
fig.suptitle("Frontal plane alone: a rigid body rocking on two arcs cut at the middle (model points; theory within ~5-10 %)",
             fontsize=11)
fig.tight_layout()
path = os.path.join(OUT, "frontal.png")
fig.savefig(path, dpi=105)
print(path)
