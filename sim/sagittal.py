# The sagittal plane alone, idealised: the body can only go forward, up and
# pitch (slide y, slide z, turn about the side-to-side axis); two legs on
# servos swing in antiphase, each with a sole curved fore and aft (an arc of
# radius Rs). The frontal plane's job -- lifting one foot while the other
# carries the body -- is imposed instead of simulated: a foot can touch the
# floor only in its own share of the cycle (duty 0.5: strict alternation;
# more: both feet down for a while).
#
# The knob that ties the two planes: the stance phase phi, where in the legs'
# swing a foot's stance is centred. The right leg's angle is A sin(w t),
# + = foot forward; phi = 180 deg puts the stance on the backward sweep.
#
#   python sagittal.py                progress per cycle over Rs, A and phi
#   python sagittal.py --duty         ... and over the double support
#
# Masses from trex_sim (hip 92, 2026-10-30). Writes out/sagittal*.csv; the
# picture with sagittal_plot.py.

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

CUBE = dict(H=92.5,                      # mm, hip above the floor
            M=0.185, com=-6.8, k=23.6,   # body: kg, COM below the hip, pitch k
            m_leg=0.0201, leg_com=-78.7, leg_I=10033e-9,  # each leg (kg m^2)
            heel=60.0, toe=60.0, yc=0.0,  # sole: mm behind / ahead of the hip,
            Rs=186.0,                     #   its arc's centre ahead, radius
            A=15.0, f=2.0, phi=180.0, duty=0.5)
WARM, SECS = 3.0, 10.0


def sole(c, n=60):
    """The sole's hull points in the leg's frame (hip at 0), mm -> m."""
    lim = 0.95 * c["Rs"]
    ys = np.linspace(max(-c["heel"], c["yc"] - lim), min(c["toe"], c["yc"] + lim), n)
    z = -c["H"] + c["Rs"] - np.sqrt(c["Rs"] ** 2 - (ys - c["yc"]) ** 2)
    top = z.max() + 5
    pts = [(x, y, zz) for y, zz in zip(ys, z) for x in (-10, 10)]
    pts += [(x, y, top) for y in (ys[0], ys[-1]) for x in (-10, 10)]
    return " ".join(f"{v / 1000:.6f}" for p in pts for v in p)


def build(c):
    kb = c["M"] * (c["k"] / 1000) ** 2
    legs = ""
    for side in ("R", "L"):
        legs += f"""
      <body name="leg{side}">
        <joint name="hip{side}" type="hinge" axis="1 0 0"/>
        <inertial pos="0 0 {c['leg_com'] / 1000}" mass="{c['m_leg']}"
                  diaginertia="{c['leg_I']} {c['leg_I']} {c['leg_I'] / 4}"/>
        <geom name="foot{side}" type="mesh" mesh="sole" mass="0" friction="1.2 0.005 0.0001"/>
      </body>"""
    return f"""<mujoco>
  <option timestep="0.0005" gravity="0 0 -9.81"/>
  <asset><mesh name="sole" vertex="{sole(c)}"/></asset>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 .01" friction="1.2 0.005 0.0001"/>
    <body name="body" pos="0 0 {c['H'] / 1000 + 0.001}">
      <joint name="y" type="slide" axis="0 1 0"/>
      <joint name="z" type="slide" axis="0 0 1"/>
      <joint name="pitch" type="hinge" axis="1 0 0"/>
      <inertial pos="0 0 {c['com'] / 1000}" mass="{c['M']}" diaginertia="{kb} {kb} {kb}"/>{legs}
    </body>
  </worldbody>
  <actuator>
    <position joint="hipR" kp="3" forcerange="-0.2 0.2" forcelimited="true"/>
    <position joint="hipL" kp="3" forcerange="-0.2 0.2" forcelimited="true"/>
  </actuator>
</mujoco>"""


def run(c):
    m = mujoco.MjModel.from_xml_string(build(c))
    d = mujoco.MjData(m)
    feet = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"foot{s}") for s in "RL"]
    w, A = 2 * math.pi * c["f"], math.radians(c["A"])
    cut = math.cos(math.pi * c["duty"])        # a foot is down while cos(.) > cut
    prev = [0.0, 0.0]
    y0, pitch, n = None, [], int((WARM + SECS) / m.opt.timestep)
    for i in range(n):
        t = d.time
        ramp = min(1.0, t / 1.0)
        target = A * ramp * math.sin(w * t)
        d.ctrl[:] = (target, -target)
        for k, g in enumerate(feet):           # the imposed alternation
            ph = w * t - math.radians(c["phi"]) - k * math.pi
            on = 1 if math.cos(ph) > cut else 0
            m.geom_contype[g] = m.geom_conaffinity[g] = on
        mujoco.mj_step(m, d)
        if d.qpos[1] < -0.06 or abs(d.qpos[2]) > 1.2:
            return dict(c, fwd_cycle=math.nan, pitch_amp=math.nan, fell=1)
        if t >= WARM:
            if y0 is None:
                y0 = d.qpos[0]
            if i % 10 == 0:
                pitch.append(math.degrees(d.qpos[2]))
    cycles = SECS * c["f"]
    p = np.array(pitch)
    return dict(c, fwd_cycle=(d.qpos[0] - y0) * 1000 / cycles,
                pitch_amp=float(np.percentile(p, 95) - np.percentile(p, 5)) / 2, fell=0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duty", action="store_true")
    a = ap.parse_args()
    phis = list(range(0, 360, 30))
    if a.duty:
        jobs = [dict(CUBE, Rs=R, A=15.0, phi=float(ph), duty=du)
                for R, ph, du in itertools.product((92.5, 186, 400), phis, (0.5, 0.6, 0.7, 0.8))]
        name = "sagittal_duty"
    else:
        jobs = [dict(CUBE, Rs=R, A=A_, phi=float(ph))
                for R, A_, ph in itertools.product((60, 92.5, 140, 186, 400), (10.0, 15.0, 20.0), phis)]
        name = "sagittal"
    with mp.Pool() as pool:
        rows = pool.map(run, jobs)
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name + ".csv")
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(path)
    keys = ("Rs", "duty") if a.duty else ("Rs", "A")
    for combo in sorted({tuple(r[k] for k in keys) for r in rows}):
        sel = sorted((r for r in rows if tuple(r[k] for k in keys) == combo), key=lambda r: r["phi"])
        print("  " + " ".join(f"{k} {v:g}" for k, v in zip(keys, combo)).ljust(18) + " mm/cycle by phi: " +
              " ".join("  fell" if r["fell"] else f"{r['fwd_cycle']:+6.1f}" for r in sel))
    print("  phi:" + " " * 30 + " ".join(f"{p:6d}" for p in phis))


if __name__ == "__main__":
    main()
