# The frontal plane alone, idealised: a rigid body that can only rock side
# to side (slide x, slide z, turn about the fore-aft axis) on two arcs, one
# per foot, cut at the middle like the cap feet. No legs, no stepping.
#
#   radius R   each foot's arc (the sphere's radius, seen from the front)
#   gap g      the arcs' centres g apart side to side; cut at the middle
#   height h   the centre of mass above the floor, standing upright
#   k          radius of gyration about the fore-aft axis through the COM
#
# One well, V-shaped: upright the body stands on both feet (on each arc's
# lowest point, g apart) like a stool; leaning onto one foot lifts the
# centre of mass, so it rocks back, and at every crossing of the middle the
# support jumps from one foot to the other: a knock (Housner's rocking block,
# with rounded feet). From the theory (a = g/2, b = R - h, I = k^2 + a^2 + h^2):
#   f0     sqrt(g b / (k^2 + h^2)) / 2 pi   the smooth rocker (no gap)
#   f(A)   the period of a swing of amplitude A in the V-shaped well
#          U = M g (a sin|th| - b cos th), with I fixed (a fair guess)
#   keep   (k^2 + h^2 - a^2) / I            the swing's speed kept per knock
# From the model, released from a lean: the swing's frequency, and how much
# of the amplitude each half swing keeps.
#
#   python frontal.py                 the two cubes (hip 52, hip 92)
#   python frontal.py --sweep         R, h, gap around the hip-92 cube
#
# Writes out/frontal_*.csv; the picture with frontal_plot.py.

import argparse
import csv
import itertools
import math
import multiprocessing as mp
import os

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
G = 9.81

# From trex_sim (2026-10-30): mass, COM height, radius of gyration, feet
CUBES = {
    "hip52": dict(M=0.285, h=45.2, k=38.4, R=105.0, gap=28.0, out=50.0),
    "hip92": dict(M=0.225, h=72.9, k=36.7, R=185.8, gap=14.0, out=71.0),
}
LEANS = [2, 4, 6, 8, 10, 12, 15, 18, 22, 26]     # deg, released from rest


def theory(c, A=None):
    a, b, h, k = c["gap"] / 2, c["R"] - c["h"], c["h"], c["k"]
    out = dict(f0=math.sqrt(G * 1000 * b / (k * k + h * h)) / (2 * math.pi),
               keep=(k * k + h * h - a * a) / (k * k + h * h + a * a))
    if A:
        I = (k * k + a * a + h * h) / 1000            # mm^2 -> m*mm: U in mm
        A = math.radians(A)
        U = lambda th: a * np.sin(th) - b * np.cos(th)
        th = np.linspace(0, A, 4001)[:-1]
        v = np.sqrt(np.maximum(2 * G * (U(A) - U(th)) / I, 1e-12))
        out["f_A"] = 1 / (4 * np.trapezoid(1 / v, th))
    return out


def arc(c, s, n=60):
    """One foot's hull points (s = +1 right, -1 left), mm: the arc from the
    middle to `out`, extruded 40 mm fore and aft, closed by a flat top."""
    xs = np.linspace(0.0, c["out"], n) * s
    z = c["R"] - np.sqrt(np.maximum(c["R"] ** 2 - (xs - s * c["gap"] / 2) ** 2, 0))
    top = z.max() + 5
    pts = [(x, y, zz) for x, zz in zip(xs, z) for y in (-20, 20)]
    pts += [(x, y, top) for x in (xs[0], xs[-1]) for y in (-20, 20)]
    return " ".join(f"{v / 1000:.6f}" for p in pts for v in p)


def build(c):
    k2 = (c["k"] / 1000) ** 2 * c["M"]
    return f"""<mujoco>
  <option timestep="0.0005" gravity="0 0 -9.81"/>
  <asset>
    <mesh name="footR" vertex="{arc(c, 1)}"/>
    <mesh name="footL" vertex="{arc(c, -1)}"/>
  </asset>
  <worldbody>
    <geom type="plane" size="1 1 .01" friction="1.2 0.005 0.0001"/>
    <body name="body">
      <joint name="x" type="slide" axis="1 0 0"/>
      <joint name="z" type="slide" axis="0 0 1"/>
      <joint name="roll" type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 {c['h'] / 1000}" mass="{c['M']}" diaginertia="{k2} {k2} {k2}"/>
      <geom name="footR" type="mesh" mesh="footR" mass="0" friction="1.2 0.005 0.0001"/>
      <geom name="footL" type="mesh" mesh="footL" mass="0" friction="1.2 0.005 0.0001"/>
    </body>
  </worldbody>
</mujoco>"""


def release(c, lean, secs=6.0):
    """Lean the body (deg, + to the right foot), let go: the roll and which
    feet touch, every ms."""
    m = mujoco.MjModel.from_xml_string(build(c))
    d = mujoco.MjData(m)
    d.qpos[2] = math.radians(lean)
    d.qpos[1] = 0.02                                  # drop from 2 cm up
    gR, gL = (mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n) for n in ("footR", "footL"))
    for _ in range(int(0.3 / m.opt.timestep)):        # fall onto the floor,
        d.qvel[:] = 0                                 # held at the lean
        d.qpos[2] = math.radians(lean)
        mujoco.mj_step(m, d)
    rows = []
    for i in range(int(secs / m.opt.timestep)):
        mujoco.mj_step(m, d)
        if i % 2:
            continue
        on = {d.contact[j].geom1 for j in range(d.ncon)} | {d.contact[j].geom2 for j in range(d.ncon)}
        rows.append((d.time, math.degrees(d.qpos[2]), gR in on, gL in on))
    return rows


def analyse(rows):
    t = np.array([r[0] for r in rows])
    a = np.array([r[1] for r in rows])
    pk = [i for i in range(1, len(a) - 1) if abs(a[i]) >= abs(a[i - 1]) and abs(a[i]) > abs(a[i + 1])
          and abs(a[i]) > 0.3]
    peaks = [(t[i], a[i]) for i in pk]
    out = dict(f_sim=math.nan, keep_sim=math.nan, amp=math.nan)
    if len(peaks) >= 3:                 # the first full swing
        out["f_sim"] = 1 / (peaks[2][0] - peaks[0][0])
        out["keep_sim"] = abs(peaks[1][1]) / abs(peaks[0][1])
        out["amp"] = abs(peaks[0][1])
    out["settled_s"] = next((tt for tt, aa in zip(t[::-1], a[::-1]) if abs(aa) > 0.3), t[0]) - t[0]
    return out


def one(job):
    name, c, lean = job
    res = analyse(release(c, lean))
    th = theory(c, res["amp"] if res["amp"] == res["amp"] else lean)
    return dict(name=name, lean=lean, **{k: c[k] for k in ("R", "h", "gap", "k", "M")}, **th, **res)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep", action="store_true")
    a = ap.parse_args()
    if a.sweep:
        base = CUBES["hip92"]
        jobs = []
        for R, h, g in itertools.product((120, 150, 186, 240), (55, 73, 90), (0, 7, 14, 28, 40)):
            c = dict(base, R=R, h=h, gap=g)
            for lean in (5, 15):
                jobs.append((f"R{R}_h{h}_g{g}", c, lean))
        name = "frontal_sweep"
    else:
        jobs = [(n, c, lean) for n, c in CUBES.items() for lean in LEANS]
        name = "frontal_cubes"
    with mp.Pool() as pool:
        rows = pool.map(one, jobs)
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name + ".csv")
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(path)
    for r in rows:
        print(f"  {r['name']:14s} lean {r['lean']:4.0f}: swing +-{r['amp']:4.1f} deg  f sim {r['f_sim']:.2f} Hz,"
              f" theory {r['f_A']:.2f} (smooth rocker {r['f0']:.2f}) | keeps sim {r['keep_sim']:.2f},"
              f" theory {r['keep']:.2f} | still after {r['settled_s']:.1f} s")


if __name__ == "__main__":
    main()
