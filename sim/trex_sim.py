# T-rex walker v1 in MuJoCo: the cube rocking on its arched feet, the tail
# pumping it, the legs stepping — the same open-loop gait as
# firmware/trex_walk.py, so the knobs found here carry over.
#
#   python trex_sim.py                      one run, prints what happened
#   python trex_sim.py --view               the same, in a window, real time
#                                           (or .er.ps1 freq=0.9 ...)
#   python trex_sim.py --set freq=1.6 phase=180 lean=-5
#   python trex_sim.py --sweep freq         tail only: which rhythm rocks most
#   python trex_sim.py --sweep phase        walking: which phase goes forward
#   python trex_sim.py --sweep lean | weight
#   python trex_sim.py --sweep weight_g=6,12,18 --set leg_deg=0    any knob
#
# The parts' shapes come from the macro (cad/animo_trex.py, ANIMO_VIEW=
# sim/meshes); the masses are guesses to replace with the scale's (P below).
# The cube and the brain are plain boxes. Contacts: only with the floor.
# What it can't know: the real servos' speed and backlash, the floor's grip.
# Trust the trends (which phase, roughly what rhythm), not the millimetres.

import argparse
import math
import os
import struct

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MESH = os.path.join(HERE, "meshes")

P = dict(
    cube_g=125.0,      # g — the cube without the brain (guess: 185 in all)
    brain_g=60.0,      # g — battery, boards, box and lid, up top (guess)
    leg_g=10.0,        # g — each leg, printed (15 cm3 solid, ~half full)
    tail_g=2.0,        # g — the tail, printed
    weight_g=6.0,      # g — M5x16 + 2 nuts at the tail's tip
    head=2.0,          # 2: a head that NODS (fore-aft, about X) on the front
                       #    wall: each nod forward is a step, like a pigeon
                       #    (user, 2026-10-28). The pin-in-slot that turns the
                       #    front output's swing into a nod is not drawn yet:
                       #    here the nod is driven directly.
                       # 1: a second tail on the front wall, swinging with it
                       # 0: none
    head_g=-1.0,       # g — the head's tip weight; -1 = as the tail's
    head_len=40.0,     # mm — nod pivot to the head's weight
    head_down=15.0,    # deg — the neck's slope at rest
    gait=1.0,          # 1: three phases, repeated (user, 2026-10-28):
                       #    START the tail starts the rocking; STEPS the head
                       #    nods once per step, the legs step; STOP all to
                       #    rest, the tail against the rocking to calm it
                       # 0: continuous (tail, legs, the head as a tail)
    start_n=3.0,       # rocking cycles of each phase
    steps_n=4.0,
    stop_n=2.0,
    rest_s=1.0,        # s — still, between rounds
    head_deg=15.0,     # the nod's depth (down from rest)
    head_phase=180.0,  # deg — where in the step the nod falls
    tail_keep=0.5,     # the tail during STEPS, as a share of tail_deg
    mirror=1.0,        # 1: every other round is the mirror image (the tail
                       #    starts to the other side): each round's start
                       #    turns it one way, the next one back
    freq=1.0,          # Hz
    tail_deg=15.0,     # each way
    leg_deg=6.0,       # each way
    phase=225.0,       # deg, legs behind the tail (robust.py: in both gaits)
    lean=0.0,          # deg, both legs together
    tail_only_s=6.0,
    fade_s=4.0,
    secs=20.0,
    friction=0.8,      # PETG on a table, roughly
    servo_speed=10.0,  # rad/s — MG90D: 0.1 s / 60 deg
    servo_torque=0.2,  # N m — MG90D: ~2 kg cm
)

HIP_H = 0.052          # m — from the macro: hip above the ground


def binary_stl(name, mirror_y=False):
    """FreeCAD writes ASCII STL; MuJoCo reads binary. Convert once.
    mirror_y: the mirror image front to back (the tail -> a head)."""
    src = os.path.join(MESH, name + ".stl")
    dst = os.path.join(MESH, name + ("_my" if mirror_y else "") + "_bin.stl")
    if os.path.exists(dst) and os.path.getmtime(dst) >= os.path.getmtime(src):
        return dst
    tris, cur = [], []
    with open(src) as f:
        for line in f:
            w = line.split()
            if w and w[0] == "vertex":
                cur.append([float(v) / 1000.0 for v in w[1:4]])   # mm -> m
                if len(cur) == 3:
                    tris.append(cur)
                    cur = []
    with open(dst, "wb") as f:
        f.write(b"\0" * 80 + struct.pack("<I", len(tris)))
        for t in tris:
            a, b, c = (np.array(v) for v in t)
            if mirror_y:
                a, b, c = (v * np.array([1, -1, 1]) for v in (a, c, b))
            n = np.cross(b - a, c - a)
            n = n / (np.linalg.norm(n) or 1.0)
            f.write(struct.pack("<12fH", *n, *a, *b, *c, 0))
    return dst


def build(p):
    kg = lambda g: g / 1000.0
    hg = p["weight_g"] if p["head_g"] < 0 else p["head_g"]
    head = f"""<body name="head">
        <joint name="head" type="hinge" axis="0 1 0"/>
        <geom type="mesh" mesh="head" mass="{kg(p['tail_g'])}" rgba=".3 .6 .8 1"/>
        <geom type="cylinder" size=".0046 .0115" pos="0 .052 -.030" euler="0 90 0"
              mass="{kg(max(hg, 0.0001))}" rgba=".4 .4 .45 1"/>
      </body>""" if p["head"] else ""
    act_head = (f'<position name="head" joint="head" kp="3" forcerange="-{p["servo_torque"]} '
                f'{p["servo_torque"]}"/>' if p["head"] else "")
    if p["head"] == 2:
        a = math.radians(p["head_down"])
        hy, hz = p["head_len"] / 1000 * math.cos(a), -p["head_len"] / 1000 * math.sin(a)
        head = f"""<body name="head" pos="0 .031 0">
        <joint name="head" type="hinge" axis="1 0 0"/>
        <geom type="capsule" fromto="0 0 0 0 {hy} {hz}" size=".004"
              mass="{kg(p['tail_g'] + 1.0)}" rgba=".3 .6 .8 1"/>
        <geom type="sphere" size=".006" pos="0 {hy} {hz}"
              mass="{kg(max(hg, 0.0001))}" rgba=".4 .4 .45 1"/>
      </body>"""
    xml = f"""
<mujoco model="animo_trex">
  <option timestep="0.0005" gravity="0 0 -9.81"/>
  <asset>
    <mesh name="legR" file="{binary_stl('LegRight')}"/>
    <mesh name="legL" file="{binary_stl('LegLeft')}"/>
    <mesh name="tail" file="{binary_stl('Tail')}"/>
    <mesh name="head" file="{binary_stl('Tail', mirror_y=True)}"/>
    <texture name="grid" type="2d" builtin="checker" width="256" height="256"
             rgb1=".85 .85 .85" rgb2=".75 .75 .75"/>
    <material name="grid" texture="grid" texrepeat="20 20"/>
  </asset>
  <default>
    <geom contype="1" conaffinity="0" friction="{p['friction']} 0.005 0.0001"
          solref="0.004 1"/>
    <joint damping="0.002" armature="0.0001"/>
  </default>
  <worldbody>
    <light pos="0 -0.3 0.6" dir="0 0.5 -1"/>
    <geom name="floor" type="plane" size="2 2 0.01" material="grid"
          contype="0" conaffinity="1"/>
    <body name="torso" pos="0 0 {HIP_H + 0.0005}">
      <freejoint/>
      <geom name="cube" type="box" size=".028 .028 .028" mass="{kg(p['cube_g'])}"
            rgba=".75 .8 .9 1"/>
      <geom name="brain" type="box" size=".028 .028 .014" pos="0 0 .042"
            mass="{kg(p['brain_g'])}" rgba=".6 .7 .85 1"/>
      <geom name="nose" type="box" size=".006 .003 .006" pos="0 .031 .042"
            mass="0.0001" rgba=".9 .2 .2 1"/>
      <body name="legR">
        <joint name="legR" type="hinge" axis="1 0 0"/>
        <geom type="mesh" mesh="legR" mass="{kg(p['leg_g'])}" rgba=".95 .6 .2 1"/>
      </body>
      <body name="legL">
        <joint name="legL" type="hinge" axis="1 0 0"/>
        <geom type="mesh" mesh="legL" mass="{kg(p['leg_g'])}" rgba=".95 .6 .2 1"/>
      </body>
      <body name="tail">
        <joint name="tail" type="hinge" axis="0 1 0"/>
        <geom type="mesh" mesh="tail" mass="{kg(p['tail_g'])}" rgba=".3 .7 .3 1"/>
        <geom type="cylinder" size=".0046 .0115" pos="0 -.052 -.030" euler="0 90 0"
              mass="{kg(p['weight_g'])}" rgba=".4 .4 .45 1"/>
      </body>
      {head}
    </body>
  </worldbody>
  <actuator>
    <position name="legR" joint="legR" kp="3" forcerange="-{p['servo_torque']} {p['servo_torque']}"/>
    <position name="legL" joint="legL" kp="3" forcerange="-{p['servo_torque']} {p['servo_torque']}"/>
    <position name="tail" joint="tail" kp="3" forcerange="-{p['servo_torque']} {p['servo_torque']}"/>
    {act_head}
  </actuator>
</mujoco>"""
    m = mujoco.MjModel.from_xml_string(xml)
    return m, mujoco.MjData(m)


def phases(p, t):
    """The three-phase gait, repeated: (legR, legL, tail, head) in degrees,
    and the phase's name. The phases last whole rocking cycles, so the
    rhythm runs on unbroken from one to the next."""
    T = 1.0 / p["freq"]
    n1, n2, n3 = p["start_n"] * T, p["steps_n"] * T, p["stop_n"] * T
    period = n1 + n2 + n3 + p["rest_s"]
    k = t % period
    flip = p["mirror"] and int(t // period) % 2 == 1
    w = 2 * math.pi * p["freq"] * t
    tail, step, head = 0.0, 0.0, 0.0
    if k < n1:                                          # START
        name = "start"
        tail = p["tail_deg"] * min(1.0, k / T) * math.sin(w)
    elif k < n1 + n2:                                   # STEPS
        name = "steps"
        u = k - n1
        fade = min(1.0, u / (0.5 * T))
        tail = p["tail_deg"] * p["tail_keep"] * math.sin(w)
        step = fade * p["leg_deg"] * math.sin(w - math.radians(p["phase"]))
        nod = (1 - math.cos(2 * w - math.radians(p["head_phase"]))) / 2
        head = -fade * p["head_deg"] * nod              # two nods a cycle
    elif k < n1 + n2 + n3:                              # STOP
        name = "stop"
        u = k - n1 - n2
        fade = max(0.0, 1 - u / (0.5 * T))
        step = fade * p["leg_deg"] * math.sin(w - math.radians(p["phase"]))
        tail = -p["tail_deg"] * (1 - u / n3) * math.sin(w)   # against it
    else:
        name = "rest"
    if flip:                    # mirror image: the legs swap, the tail flips
        return (-step, step, -tail, head), name
    return (step, -step, tail, head), name


def targets(p, t):
    """The firmware's gait: (legR, legL, tail) in radians at time t."""
    w = 2 * math.pi * p["freq"] * t
    tail = p["tail_deg"] * min(1.0, t / 2)
    legs = p["leg_deg"] * min(1.0, max(0.0, (t - p["tail_only_s"]) / p["fade_s"]))
    lean = p["lean"] * min(1.0, t / 2)
    step = legs * math.sin(w - math.radians(p["phase"]))
    r = math.radians
    return r(lean + step), r(lean - step), r(tail * math.sin(w))


class Run:
    def __init__(self, p):
        self.p = p
        self.m, self.d = build(p)
        self.cmd = np.zeros(self.m.nu)
        self.start = None
        self.roll, self.pitch, self.fell = [], [], False
        self.phase = ""

    def step(self):
        p, m, d = self.p, self.m, self.d
        if p["gait"] == 1:
            deg, self.phase = phases(p, d.time)
            want = np.radians(deg[:m.nu])
        else:
            want = np.array(targets(p, d.time))
            if m.nu == 4:
                want = np.append(want, want[2])     # the head swings with the tail
        dmax = p["servo_speed"] * m.opt.timestep        # the servo's top speed
        self.cmd += np.clip(want - self.cmd, -dmax, dmax)
        d.ctrl[:] = self.cmd
        mujoco.mj_step(m, d)
        if self.start is None:
            self.start = d.qpos[:2].copy()
        R = d.xmat[1].reshape(3, 3)          # torso's axes in the world
        up = R[:, 2]
        if up[2] < 0.5:
            self.fell = True
        self.roll.append(math.degrees(math.atan2(up[0], up[2])))
        self.pitch.append(math.degrees(math.atan2(-up[1], up[2])))

    def go(self):
        n = int(self.p["secs"] / self.m.opt.timestep)
        for _ in range(n):
            self.step()
            if self.fell:
                break
        return self.result()

    def result(self):
        d = self.d
        tail = int(5.0 / self.m.opt.timestep)
        roll = np.array(self.roll[-tail:])
        pitch = np.array(self.pitch[-tail:])
        fwd = d.qpos[:2] - self.start
        R = d.xmat[1].reshape(3, 3)
        heading = math.degrees(math.atan2(-R[0, 1], R[1, 1]))
        return dict(fell=self.fell, t=d.time,
                    fwd_mm=fwd[1] * 1000, side_mm=fwd[0] * 1000,
                    turn_deg=heading,
                    roll_amp=(roll.max() - roll.min()) / 2 if len(roll) else 0,
                    pitch_mean=pitch.mean() if len(pitch) else 0)


def show(p):
    import mujoco.viewer
    import time
    r = Run(p)
    with mujoco.viewer.launch_passive(r.m, r.d) as v:
        v.cam.distance, v.cam.elevation, v.cam.azimuth = 0.45, -15, 135
        while v.is_running() and r.d.time < p["secs"]:
            t0 = time.perf_counter()
            last = r.phase
            for _ in range(int(0.02 / r.m.opt.timestep)):
                r.step()
            if r.phase != last:
                print(f"{r.d.time:5.1f} s  {r.phase.upper()}", flush=True)
            v.cam.lookat[:] = r.d.qpos[:3]
            v.sync()
            time.sleep(max(0.0, 0.02 - (time.perf_counter() - t0)))
    print(fmt(r.result()))


def fmt(res):
    s = "FELL at %.1f s  " % res["t"] if res["fell"] else ""
    return (s + "forward %+6.1f mm  side %+6.1f  turn %+5.1f deg  "
            "rocking +-%4.1f deg  pitch %+5.1f deg" %
            (res["fwd_mm"], res["side_mm"], res["turn_deg"],
             res["roll_amp"], res["pitch_mean"]))


SWEEPS = {
    "freq":   ("freq", [1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4], dict(leg_deg=0.0)),
    "phase":  ("phase", [0, 45, 90, 135, 180, 225, 270, 315], {}),
    "lean":   ("lean", [-10, -5, 0, 5, 10], {}),
    "weight": ("weight_g", [0, 3, 6, 9, 12], dict(leg_deg=0.0)),
    "tail":   ("tail_deg", [15, 20, 25, 30, 35], dict(leg_deg=0.0)),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--sweep", help="a name in SWEEPS, or key=v1,v2,...")
    ap.add_argument("--set", nargs="*", default=[], metavar="KEY=VALUE")
    a = ap.parse_args()
    p = dict(P)
    for kv in a.set:
        k, v = kv.split("=")
        assert k in p, f"unknown: {k}"
        p[k] = float(v)
    if a.sweep:
        if "=" in a.sweep:
            key, vals = a.sweep.split("=")
            assert key in p, f"unknown: {key}"
            key, values, fixed = key, [float(v) for v in vals.split(",")], {}
        else:
            key, values, fixed = SWEEPS[a.sweep]
        print(f"sweep {key}  (fixed: {fixed or 'none'})")
        for val in values:
            q = dict(p, **fixed)
            q[key] = val
            print(f"  {key} = {val:>5}: " + fmt(Run(q).go()))
    elif a.view:
        show(p)
    else:
        print(fmt(Run(p).go()))


if __name__ == "__main__":
    main()
