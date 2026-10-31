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
    brain_down=0.0,    # 1: the cube upside down, the brain under the hips
                       #    (the centre of mass low, like Mugatu's); in
                       #    between, the brain that far down: 0.5 = at the hips
    sag=0.0,           # mm — the sole's fore-aft arc, its centre above the hip
                       #    (cad ANIMO_SAG_UP): 20 the drawn one, 60, 150;
                       #    0 = flat fore and aft (a cylinder, curved only
                       #    side to side). 20 is nearly a ball: it pivots.
                       #    Step by step wants 0 (2026-10-29): on one foot an
                       #    arc lets the body pitch instead of going forward
    front=6.0,         # deg — 0: the sole is an arc side to side (R 75, a
                       #    rocker). >0: flat, rising outward this much: the
                       #    body rests still on one sole (cad ANIMO_FRONT)
    gap=3.0,           # mm — the soles' inner edges from the middle (cad
                       #    ANIMO_XIN); flat soles want them apart
    ball_r=0.0,        # mm — >0: spherical feet (Mugatu, arXiv 2308.08401):
                       #    the floor touches a sphere of this radius
    ball_gap=0.0,      # mm — between the two spheres' centres, side to side
    ball_rs=0.0,       # mm — >0: the sole's radius fore and aft, ball_r then
                       #    only side to side (a doubly curved sole, 17/10)
    ball_y=0.0,        # mm — the spheres' centres ahead of the hip
    cap_heel=30.0,     # mm — the cap foot reaches this far behind the hip
    cap_toe=30.0,      #      and this far ahead
    heel_round=0.0,    # mm — the heel's edge rounded off, this radius: the
                       #    sole curls up over the last heel_round mm
    cap_in=0.5,        # mm — its inner edge out from the middle; <0 crosses
                       #    under the other foot (the feet don't collide in
                       #    the sim, a printed pair would)
    cap_out=35.0,      # mm — its outer edge out from the middle
    cap_wide_from=-999.0,  # mm — the outer edge (seen from below) is at cap_out
    cap_wide_to=999.0,     #   only between these two y; toward the heel and
    cap_heel_x=0.0,        #   the toe it curves in (a quarter ellipse) to
    cap_toe_x=0.0,         #   these x at the very ends. Default: a rectangle
    foot_w_g=0.0,      # g — a weight low in each cap foot (Mugatu's wheel
                       #    weights): lowers the centre of mass
    foot_mu=0.0,       # >0: the cap feet in rubber, this friction, and they
    foot_spin=0.0,     #    resist spinning on the floor: mm, the contact
                       #    patch's lever (MuJoCo's torsional friction; the
                       #    default contact has none, a point spins freely)
    hip=52.0,          # mm — hip to sole (cad ANIMO_HIP): longer legs
    leg_g=10.0,        # g — each leg, printed (15 cm3 solid, ~half full)
    tail_g=2.0,        # g — the tail, printed
    tail=1.0,          # 0: no tail (drawn faint, no mass, never touches);
                       # 2: a tail that never touches the floor (raised)
    weight_g=40.0,     # g — at the tail's tip: 40 shifts the weight from one
                       #    flat sole to the other (28 does not). Was 6: M5 + nuts
    head=2.0,          # 2: a head that POKES forward (a neck pivoting about X
                       #    on the front wall, pointing up): its weight tips
                       #    the whole body forward to fall onto the foot that
                       #    steps — each poke a step (user, 2026-10-28). A
                       #    small nod down barely moved the centre of mass. The pin-in-slot that turns the
                       #    front output's swing into a nod is not drawn yet:
                       #    here the nod is driven directly.
                       # 1: a second tail on the front wall, swinging with it
                       # 0: none
    head_g=20.0,       # g — the head's weight (step by step: 20 g on a 50 mm
                       #    neck, upright, poking 90 deg to level). Gait 1 used
                       #    12 g on 40 mm at -70. Its fore-aft pull is touchy:
                       #    6 g tips the body 4 deg poking, 12 g 6 deg, 18 g
                       #    falls on its face standing still
    head_len=50.0,     # mm — pivot to the head's weight
    head_down=-90.0,   # deg — the neck's slope at rest (minus = up)
    gait=2.0,          # 2: step by step, no passive dynamics (user,
                       #    2026-10-29): slow keyframes, each held until the
                       #    body is still. The tail lifts the first foot
                       #    (start), then each step: the head pokes and the
                       #    body tips onto the front foot, the legs swap, the
                       #    head comes back. kf_* below
                       # 1: three phases, repeated (user, 2026-10-28):
                       #    START the tail starts the rocking; STEPS the head
                       #    nods once per step, the legs step; STOP all to
                       #    rest, the tail against the rocking to calm it
                       # 0: continuous (tail, legs, the head as a tail)
    start_n=3.0,       # rocking cycles of each phase
    steps_n=4.0,
    stop_n=2.0,
    rest_s=1.0,        # s — still, between rounds
    head_deg=40.0,     # how far the head pokes forward
    head_phase=135.0,  # deg — where in the step the poke falls
    tail_keep=0.5,     # the tail during STEPS, as a share of tail_deg
    mirror=1.0,        # 1: every other round is the mirror image (the tail
                       #    starts to the other side): each round's start
                       #    turns it one way, the next one back
    freq=1.0,          # Hz
    tail_deg=15.0,     # each way
    leg_deg=6.0,       # each way
    phase=315.0,       # deg, legs behind the tail (robust.py, sag 150)
    lean=4.0,          # deg, both legs together (+ = feet forward). Step by
                       #    step: 4 (order 3), -4 (order 2). Gait 1 wanted -5
    kf_ramp=0.4,       # s — each keyframe's move, eased (no jerk to bounce on).
                       #    0.4 with no hold: steps run on (robust.py); 0.35
                       #    falls apart, 0.45 turns more
    kf_hold=0.0,       # s — then still, for the body to settle (was 0.6)
    kf_tail=35.0,      # deg — the tail, each way
    kf_leg=8.0,        # deg — each leg, forward or back of lean
    kf_head=0.0,       # deg — the poke: upright to level (90). Order 3: off,
                       #    it only helped on a grippy floor and stalled slipping
    kf_steps=6.0,      # steps a round
    kf_overlap=0.0,    # order 3: 0 the tail, then the legs; 1 both at once
    kf_side=1.0,       # which way the tail goes first (+1 / -1)
    kf_order=3.0,      # 0: poke, step (head out), back; 1: poke, back, step;
                       #    2: poke, back, push (stance foot), reach (free foot)
                       #    3: the tail shifts the weight each step (flat soles)
    tail_only_s=6.0,
    fade_s=4.0,
    secs=20.0,
    friction=0.8,      # PETG on a table, roughly
    servo_speed=10.0,  # rad/s — MG90D: 0.1 s / 60 deg
    servo_torque=0.2,  # N m — MG90D: ~2 kg cm
    freq0=0.0,         # Hz, the start's rhythm, climbing to freq over chirp_s
    chirp_s=0.0,       #    seconds (0: freq from the start)
    trail_s=3.0,      # s of faint contact path the viewer keeps (0: none)
    servo_curve=0.0,   # 1: the torque falls with speed, a DC motor's line from
                       #    servo_torque (stalled) to servo_speed (no load);
                       #    MG90D at 4.8 V: 0.206 N m, 10.5 rad/s (sellers)
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
    tmp = f"{dst}.{os.getpid()}"            # parallel runs: write, then swap
    with open(tmp, "wb") as f:
        f.write(b"\0" * 80 + struct.pack("<I", len(tris)))
        for t in tris:
            a, b, c = (np.array(v) for v in t)
            if mirror_y:
                a, b, c = (v * np.array([1, -1, 1]) for v in (a, c, b))
            n = np.cross(b - a, c - a)
            n = n / (np.linalg.norm(n) or 1.0)
            f.write(struct.pack("<12fH", *n, *a, *b, *c, 0))
    try:
        os.replace(tmp, dst)
    except PermissionError:             # Windows: another process wrote it
        os.remove(tmp)                  # first and has it open; same mesh
    return dst


CAP_STEP = 2.0         # mm — the cap mesh's grid
CAP_T = 15.0           # mm — a cap foot this thick at the bottom. Its extent:
                       #      cap_in / cap_out, cap_heel / cap_toe (P)


def cap_stl(p, s):
    """A spherical-cap foot (s = +1 right, -1 left), in the hip's frame: the
    piece of the ball_r sphere a real foot would be, cut at the middle so
    the two feet never overlap. Closed grid mesh, binary STL."""
    R, h = p["ball_r"], p["hip"]
    cx, cy = s * p["ball_gap"] / 2, p["ball_y"]
    nx = int(np.ceil((p["cap_out"] - p["cap_in"]) / CAP_STEP)) + 1   # the same
    ny = int(np.ceil((p["cap_heel"] + p["cap_toe"]) / CAP_STEP)) + 1 # facets
    xs = np.linspace(s * p["cap_in"], s * p["cap_out"], nx)    # whatever the
    ys = np.linspace(-p["cap_heel"], p["cap_toe"], ny)         # foot's size
    rf = min(p["heel_round"], p["cap_heel"] + p["cap_toe"])
    if rf:                          # finer near the heel, for the curl
        ys = np.union1d(ys, np.linspace(-p["cap_heel"], -p["cap_heel"] + rf, 21))
        ny = len(ys)
    X, Y = np.meshgrid(xs, ys)
    w0, w1 = max(p["cap_wide_from"], -p["cap_heel"]), min(p["cap_wide_to"], p["cap_toe"])
    if w0 > -p["cap_heel"] or w1 < p["cap_toe"]:     # the outline: each row of
        out = np.full(ny, p["cap_out"])            # the grid squeezed in to it
        for end, x_end, sel in ((-p["cap_heel"], p["cap_heel_x"], ys < w0),
                                (p["cap_toe"], p["cap_toe_x"], ys > w1)):
            edge = w0 if end < 0 else w1
            u = (ys[sel] - edge) / (end - edge)
            out[sel] = x_end + (p["cap_out"] - x_end) * np.sqrt(np.maximum(1 - u * u, 0))
        out = np.maximum(out, p["cap_in"] + 1)
        X = s * (p["cap_in"] + (out[:, None] - p["cap_in"]) * np.linspace(0, 1, nx)[None, :])
    if p["ball_rs"]:               # two radii: side to side, fore and aft
        Rs = p["ball_rs"]
        Zb = (-h + R - np.sqrt(np.maximum(R * R - (X - cx) ** 2, 0))
              + Rs - np.sqrt(np.maximum(Rs * Rs - (Y - cy) ** 2, 0)))
    else:
        Zb = (R - h) - np.sqrt(np.maximum(R * R - (X - cx) ** 2 - (Y - cy) ** 2, 0))
    if rf:                          # a quarter circle: flush with the sole
        d = np.clip(-p["cap_heel"] + rf - Y, 0, rf)    # rf in, upright at
        Zb = Zb + rf - np.sqrt(rf * rf - d * d)        # the very edge
    Zt = np.full_like(Zb, max(-h + CAP_T, Zb.max() + 3))   # a long cap's
                                    # ends rise past CAP_T: keep the top above
    bot = np.stack([X, Y, Zb], -1) / 1000
    top = np.stack([X, Y, Zt], -1) / 1000
    tris = []
    def quad(a, b, c, d):
        tris.extend([(a, b, c), (a, c, d)])
    for i in range(ny - 1):
        for j in range(nx - 1):
            quad(bot[i, j], bot[i + 1, j], bot[i + 1, j + 1], bot[i, j + 1])
            quad(top[i, j], top[i, j + 1], top[i + 1, j + 1], top[i + 1, j])
    for a, b in ((bot[0, :], top[0, :]), (bot[-1, :], top[-1, :]),   # the
                 (bot[:, 0], top[:, 0]), (bot[:, -1], top[:, -1])):     # sides
        for k in range(len(a) - 1):
            quad(a[k], a[k + 1], b[k + 1], b[k])
    rs = f"_rs{p['ball_rs']:g}" if p["ball_rs"] else ""
    shape = (f"_w{w0:g},{w1:g},{p['cap_heel_x']:g},{p['cap_toe_x']:g}"
             if w0 > -p["cap_heel"] or w1 < p["cap_toe"] else "")
    tag = (f"cap_r{R:g}{rs}_g{p['ball_gap']:g}_y{p['ball_y']:g}_h{h:g}"
           f"_b{p['cap_heel']:g}_f{p['cap_toe']:g}_i{p['cap_in']:g}_o{p['cap_out']:g}{f"_hr{p['heel_round']:g}" if p['heel_round'] else ""}{shape}_v3_{'R' if s > 0 else 'L'}")
    dst = os.path.join(MESH, tag + "_bin.stl")
    if not os.path.exists(dst):
        tmp = f"{dst}.{os.getpid()}"
        with open(tmp, "wb") as f:
            f.write(b"\0" * 80 + struct.pack("<I", len(tris)))
            for a, b, c in tris:
                if s < 0:                   # mirrored grid: turn the faces
                    b, c = c, b             # back out, or they show inside
                f.write(struct.pack("<12fH", 0, 0, 0, *a, *b, *c, 0))
        try:
            os.replace(tmp, dst)
        except PermissionError:         # Windows: another process wrote it
            os.remove(tmp)              # first and has it open; same mesh
    return dst


def build(p):
    kg = lambda g: g / 1000.0

    def leg_geoms(side, s):
        """The printed leg; or, with ball_r, the leg drawn but the floor
        touching a sphere (Mugatu's feet): centre ball_r up from the
        ground, ball_gap/2 out from the middle, ball_y ahead of the hip."""
        mesh = (f'<geom name="foot{side}" type="mesh" mesh="leg{side}" '
                f'mass="{kg(p["leg_g"])}" rgba=".95 .6 .2 1"')
        grip = ""
        if p["foot_mu"]:                # rubber soles: slide and spin both
            grip = (f'condim="4" friction="{p["foot_mu"]} '    # resisted
                    f'{p["foot_spin"] / 1000} 0.0001"')
        if not p["ball_r"]:
            return mesh + "/>"
        return (mesh.replace('name="foot', 'name="shin').replace(' 1"', ' 0"') +
                ' contype="0" conaffinity="0"/>'
                f'<geom name="foot{side}" type="mesh" mesh="cap{side}" mass="0.0001"'
                f' rgba=".95 .6 .2 1" {grip}/>' + (
                f'<geom type="cylinder" size=".010 .003" mass="{kg(p["foot_w_g"])}"'
                f' pos="{s * max(p["ball_gap"] / 2, 10) / 1000} {p["ball_y"] / 1000}'
                f' {(4 - p["hip"]) / 1000}" contype="0" conaffinity="0"'
                ' rgba=".4 .4 .45 1"/>' if p["foot_w_g"] else ""))
    hg = p["weight_g"] if p["head_g"] < 0 else p["head_g"]
    head = f"""<body name="head">
        <joint name="head" type="hinge" axis="0 1 0"/>
        <geom type="mesh" mesh="head" mass="{kg(p['tail_g'])}" rgba=".3 .6 .8 1"/>
        <geom type="cylinder" size=".0046 .0115" pos="0 .052 -.030" euler="0 90 0"
              mass="{kg(max(hg, 0.0001))}" rgba=".4 .4 .45 1"/>
      </body>""" if p["head"] else ""
    act_head = (f'<position name="head" joint="head" kp="3" forcerange="-{p["servo_torque"]} '
                f'{p["servo_torque"]}"/>' if p["head"] else "")
    tag = "_flat" if p["sag"] == 0 else f"_sag{p['sag']:.0f}"
    tag += f"_v{p['front']:g}g{p['gap']:g}" if p["front"] else ""
    tag += f"_h{p['hip']:g}" if p["hip"] != 52 else ""
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
    caps = ""
    if p["ball_r"]:
        caps = "".join(f'<mesh name="cap{n}" file="{cap_stl(p, s)}"/>'
                       for n, s in (("R", 1), ("L", -1)))
    xml = f"""
<mujoco model="animo_trex">
  <option timestep="0.0005" gravity="0 0 -9.81"/>
  <asset>
    <mesh name="legR" file="{binary_stl('LegRight' + tag)}"/>
    <mesh name="legL" file="{binary_stl('LegLeft' + tag)}"/>
    {caps}
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
    <body name="torso" pos="0 0 {p['hip'] / 1000 + 0.0005}">
      <freejoint/>
      <geom name="cube" type="box" size=".028 .028 .028" mass="{kg(p['cube_g'])}"
            rgba=".75 .8 .9 1"/>
      <geom name="brain" type="box" size=".028 .028 .014" pos="0 0 {.042 * (1 - 2 * p['brain_down']):.4f}"
            mass="{kg(p['brain_g'])}" rgba=".6 .7 .85 1"/>
      <geom name="nose" type="box" size=".006 .003 .006" pos="0 .031 .042"
            mass="0.0001" rgba=".9 .2 .2 1"/>
      <body name="legR">
        <joint name="legR" type="hinge" axis="1 0 0"/>
        {leg_geoms("R", 1)}
      </body>
      <body name="legL">
        <joint name="legL" type="hinge" axis="1 0 0"/>
        {leg_geoms("L", -1)}
      </body>
      <body name="tail">
        <joint name="tail" type="hinge" axis="0 1 0"/>
        <geom type="mesh" mesh="tail" mass="{kg(p['tail_g']) if p['tail'] else 1e-7}"
              rgba=".3 .7 .3 {1 if p['tail'] else .1}" {"" if p['tail'] == 1 else 'contype="0"'}/>
        <geom type="cylinder" size=".0046 .0115" pos="0 -.052 -.030" euler="0 90 0"
              mass="{kg(p['weight_g']) if p['tail'] else 1e-7}"
              rgba=".4 .4 .45 {1 if p['tail'] else .1}" {"" if p['tail'] == 1 else 'contype="0"'}/>
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
    lean = p["lean"] * min(1.0, t / 2)
    if flip:                    # mirror image: the legs swap, the tail flips
        return (lean - step, lean + step, -tail, head), name
    return (lean + step, lean - step, tail, head), name


def keyframes(p):
    """The step-by-step gait as a list of (name, ramp_s, hold_s, (R, L, tail,
    head)) in degrees, before lean — one round: start, steps, stop, rest.
    Every other round is the mirror image (the tail lifts the other foot)."""
    A, H, T = p["kf_leg"], -p["kf_head"], p["kf_tail"]
    r, h = p["kf_ramp"], p["kf_hold"]
    out = []
    for side in (p["kf_side"], -p["kf_side"]):
        lifted = 0 if side > 0 else 1          # tail + lifts the... (index)
        legs = [0.0, 0.0]
        def f(name, tail, head, ramp=r, hold=h, win=None):
            out.append((name, ramp, hold, (legs[0], legs[1], tail, head), win))
        f("lift", side * T, 0.0)                     # START: tail lifts a foot
        legs[lifted], legs[1 - lifted] = A, -A
        f("swing", side * T, 0.0)                    # the lifted foot forward
        front = lifted
        if p["kf_order"] == 3:              # the tail shifts the weight every
            tail = side * T                 # step (flat soles, 2026-10-29):
            for i in range(int(p["kf_steps"])):   # onto the front foot, then
                tail = -tail                # it pushes back and the free one
                o = p["kf_overlap"]         # reaches forward, the head
                if o:                       # poking meanwhile. Overlapped:
                    legs[front], legs[1 - front] = -A, A   # one move, the
                    a, b = (1 + o) / 2, (1 - o) / 2        # legs start
                    f("stride", tail, H, ramp=2 * r,      # before the tail
                      win=[(b, 1), (b, 1), (0, a), (b, 1)])  # is over
                else:
                    f("shift", tail, 0.0)
                    legs[front], legs[1 - front] = -A, A
                    f("step", tail, H)
                front = 1 - front
            f("head", tail, 0.0)
            tail = 0.0
        for i in range(int(p["kf_steps"]) * (p["kf_order"] != 3)):
            if p["kf_order"] == 2:                   # one leg at a time, so
                f("poke", 0.0, H)                    # only the head moves
                f("back", 0.0, 0.0)                  # the weight: poke onto
                legs[front] = -A                     # the front foot, head
                f("push", 0.0, 0.0)                  # in, that foot pushes
                legs[1 - front] = A                  # back, the free one
                f("reach", 0.0, 0.0)                 # reaches forward
                front = 1 - front
                continue
            f("poke", 0.0, H)                        # onto the front foot
            if p["kf_order"]:                        # head back first: the
                f("back", 0.0, 0.0)                  # legs swap with it in
            legs[front], legs[1 - front] = -A, A
            f("step", 0.0, 0.0 if p["kf_order"] else H)  # rear foot forward
            if not p["kf_order"]:
                f("back", 0.0, 0.0)                  # head back
            front = 1 - front
        legs[:] = [0.0, 0.0]
        f("stop", 0.0, 0.0)                          # STOP
        f("rest", 0.0, 0.0, ramp=0.0, hold=p["rest_s"])
    return out


def frame_at(frames, t):
    """Targets at time t (degrees), eased between keyframes, and the name."""
    period = sum(f[1] + f[2] for f in frames)
    k = t % period
    prev = frames[-1][3]
    for name, r, h, tg, win in frames:
        if k < r:
            out = []
            for i, (a, b) in enumerate(zip(prev, tg)):
                w0, w1 = win[i] if win else (0.0, 1.0)  # this channel's share
                u = min(1.0, max(0.0, (k / r - w0) / (w1 - w0)))
                u = u * u * (3 - 2 * u)              # smoothstep
                out.append(a + (b - a) * u)
            return tuple(out), name
        k -= r
        if k < h:
            return tg, name
        k -= h
        prev = tg
    return prev, "rest"


def targets(p, t):
    """The firmware's gait: (legR, legL, tail) in radians at time t."""
    f1, f0, T = p["freq"], p["freq0"] or p["freq"], p["chirp_s"]
    if T and t < T:                 # the rhythm climbs from freq0 to freq
        w = 2 * math.pi * (f0 * t + (f1 - f0) * t * t / (2 * T))
    else:
        w = 2 * math.pi * (f1 * t + ((f0 - f1) * T / 2 if T else 0))
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
        self.frames = keyframes(p) if p["gait"] == 2 else None
        self.feet = [mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, n)
                     for n in ("footR", "footL")]
        self.load = np.zeros(2)      # N on each foot, this step

    def step(self):
        p, m, d = self.p, self.m, self.d
        if p["gait"] == 2:
            (R, L, tl, hd), self.phase = frame_at(self.frames, d.time)
            lean = p["lean"]
            deg = (lean + R, lean + L, tl, hd)
            want = np.radians(deg[:m.nu])
        elif p["gait"] == 1:
            deg, self.phase = phases(p, d.time)
            want = np.radians(deg[:m.nu])
        else:
            want = np.array(targets(p, d.time))
            if m.nu == 4:
                want = np.append(want, want[2])     # the head swings with the tail
        dmax = p["servo_speed"] * m.opt.timestep        # the servo's top speed
        self.cmd += np.clip(want - self.cmd, -dmax, dmax)
        d.ctrl[:] = self.cmd
        if p["servo_curve"]:
            # Pushing along the motion the motor gives less the faster it
            # turns; against it (braking) the full stall torque.
            ts, ws = p["servo_torque"], p["servo_speed"]
            w = d.qvel[m.jnt_dofadr[m.actuator_trnid[:, 0]]]
            m.actuator_forcerange[:, 1] = np.clip(ts * (1 - w / ws), 0, ts)
            m.actuator_forcerange[:, 0] = -np.clip(ts * (1 + w / ws), 0, ts)
        mujoco.mj_step(m, d)
        self.load[:] = 0
        f6 = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            for j, g in enumerate(self.feet):
                if g in (c.geom1, c.geom2):
                    mujoco.mj_contactForce(m, d, i, f6)
                    self.load[j] += f6[0]
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
    """Real time, in a window. Each sole carries a line through the points
    where it has touched the floor in the last trail_s seconds (red right,
    blue left): the path of the contact over the foot, faint; it breaks where
    the foot lifts. Over it a comet: a bright head where the sole touches now,
    a tail that thins and fades over the last FADE seconds. The physics keeps
    to the wall clock; only the frame rate drops if drawing is slow."""
    import mujoco.viewer
    import time
    r = Run(p)
    m, d = r.m, r.d
    feet = [(g, m.geom_bodyid[g], np.array(rgba, dtype=np.float32)) for g, rgba in
            zip(r.feet, ([.9, .1, .1, 1], [.1, .3, .9, 1]))]
    trace = [[], []]                      # per foot: (time, local point or None = lifted)
    fresh = [[], []]                      # per foot: (time, local point), fading
    FADE = 0.6                            # s, the comet's tail
    def mark():
        for k, (g, b, _) in enumerate(feet):
            pts = [d.contact[i].pos for i in range(d.ncon)
                   if g in (d.contact[i].geom1, d.contact[i].geom2)]
            t = trace[k]
            while t and d.time - t[0][0] > p["trail_s"]:
                t.pop(0)
            if not pts:
                if t and t[-1][1] is not None:
                    t.append((d.time, None))
                continue
            R = d.xmat[b].reshape(3, 3)
            loc = R.T @ (np.mean(pts, axis=0) - d.xpos[b])
            fresh[k].append((d.time, loc))
            if p["trail_s"] and (not t or t[-1][1] is None or np.linalg.norm(loc - t[-1][1]) > 0.0007):
                t.append((d.time, loc))
    def draw(scn):
        scn.ngeom = 0
        for k, (g, b, rgba) in enumerate(feet):     # the comet: a bright
            R = d.xmat[b].reshape(3, 3)             # head where the sole
            fresh[k][:] = [(t, q) for t, q in fresh[k] if d.time - t < FADE]
            pts = [(t, d.xpos[b] + R @ q) for t, q in fresh[k]]  # touches now,
            for (t0, a), (t1, c) in zip(pts, pts[1:]):        # a tail that
                if t1 - t0 > 0.03 or scn.ngeom >= scn.maxgeom:  # thins and
                    continue                                  # fades behind
                f = 1 - (d.time - t1) / FADE
                col = rgba.copy()
                col[3] = f ** 1.5
                geom = scn.geoms[scn.ngeom]
                mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3),
                                    np.zeros(3), np.eye(3).flatten(), col)
                mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE,
                                     0.0003 + 0.0015 * f, a, c)
                scn.ngeom += 1
            if pts and d.time - pts[-1][0] < 0.03 and scn.ngeom < scn.maxgeom:
                head = np.minimum(rgba + 0.6, 1.0)
                mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                                    [0.003, 0, 0], pts[-1][1], np.eye(3).flatten(), head)
                scn.ngeom += 1
        for k, (g, b, rgba) in enumerate(feet):
            R = d.xmat[b].reshape(3, 3)
            t = [q for _, q in trace[k]]
            for a, c in zip(t, t[1:]):
                if a is None or c is None or scn.ngeom >= scn.maxgeom:
                    continue
                geom = scn.geoms[scn.ngeom]
                faint = rgba.copy()
                faint[3] = 0.3                  # the whole path, faint
                mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_LINE, np.zeros(3),
                                    np.zeros(3), np.eye(3).flatten(), faint)
                mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, 3,
                                     d.xpos[b] + R @ a, d.xpos[b] + R @ c)
                scn.ngeom += 1
    with mujoco.viewer.launch_passive(m, d) as v:
        v.cam.distance, v.cam.elevation, v.cam.azimuth = 0.45, -15, 135
        wall0, frames = time.perf_counter(), 0
        while v.is_running() and d.time < p["secs"]:
            t0 = time.perf_counter()
            last = r.phase
            goal = min(t0 - wall0, d.time + 0.1)   # catch up with the clock, 0.1 s at most
            i = 0
            while d.time < goal:
                r.step()
                if i % 8 == 0:
                    mark()
                i += 1
            if goal < t0 - wall0:                   # too far behind: let the clock wait
                wall0 = t0 - d.time
            if r.phase != last:
                print(f"{d.time:5.1f} s  {r.phase.upper()}", flush=True)
            v.cam.lookat[:] = d.qpos[:3]
            with v.lock():
                draw(v.user_scn)
            v.sync()
            frames += 1
            time.sleep(max(0.0, 0.02 - (time.perf_counter() - t0)))
    print(fmt(r.result()))
    print(f"{frames / max(d.time, 1e-9):.0f} frames per simulated second")


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
