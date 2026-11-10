# The ski duck (2026-11-03, user: start over with a physical model): the
# cube on two long FLAT feet like skis. The skis don't turn with the legs:
# each slides fore and aft under the body (a slide joint along Y; here held
# still). Seen from the front it is a rigid block standing on the skis'
# outer edges, so its rocking is Housner's rocking block (1963):
#
#   tilted by theta onto one outer edge (half-width b, centre of mass at
#   height h; R = sqrt(b^2 + h^2), alpha = atan(b / h)), about that edge:
#       I0 theta'' = -m g R sin(alpha - |theta|) sgn(theta)
#   with I0 the moment of inertia about the edge. Each time it lands on
#   the other edge the angular momentum about the new edge is kept:
#       theta'_after = r theta'_before,  r = 1 - 2 m b^2 / I0
#   Small alpha: p = sqrt(m g R / I0), and from rest at theta0 a full
#   rock lasts  T = (4 / p) acosh(1 / (1 - theta0 / alpha)):  the smaller
#   the rock, the shorter. Each landing keeps r^2 of the energy, so it rocks
#   faster and faster and stops in a finite time.
#
#   python ski.py                   release at 8 deg: sim vs the equations
#   python ski.py --tilt 12 --view  the same, in a window
#   python ski.py --set ski_w=30    other skis
# Then the picture (FreeCAD's Python has matplotlib):
#   "C:/Program Files/FreeCAD 1.0/bin/python.exe" ski_plot.py

import argparse
import csv
import json
import math
import os

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")

SKI = dict(
    body_g=185.0,      # g — cube + brain (the duck's: 185 g at 106 mm)
    body_z=106.0,      # mm — its centre: the cube 64..120 and the brain on top
    hip=92.0,          # mm — hip (cube centre) above the floor
    ski_g=24.0,        # g — each leg and its ski, low (the duck's leg: 24 g)
    ski_len=160.0,     # mm — each ski, fore and aft
    ski_w=20.0,        # mm — each ski's width
    ski_t=4.0,         # mm — its thickness
    gap=7.0,           # mm — between the two skis' inner edges
    line_r=0.0,        # mm — >0: each ski a single line (a rod this thick)
                       #    where the flat ski's outer edge was: same b
    pitch0_deg=0.0,    # deg — at the release, also pitched (+: nose down)
    switch_at_peak=0.0,  # 1: the skis switch at the rocking's ends (by the
                       #    sign of its rate), not at the middle: they are
                       #    moving as each foot lands (user, 04/11: rockers)
    rate_dead=5.0,     # deg/s — near the ends: hold
    sag_r=0.0,         # mm — >0: with line_r, each foot a rocker: an arc fore
                       #    and aft this radius, ski_len long, a thin blade
                       #    (user, 04/11: a curve instead of two points)
    two_pts=0.0,       # 1: with line_r, each foot only two points (balls of
                       #    radius line_r) ski_len apart: toe and heel
    vis_r=6.0,         # mm — the toe and heel balls as drawn (not touching)
    friction=0.8,
    contact_tc=0.002,  # s — the contact's softness (MuJoCo solref): smaller,
                       #    harder floor and feet
    contact_damp=1.0,  # its damping ratio: 1 no bounce (critical), < 1
                       #    bouncier, > 1 duller
    secs=6.0,
    step_mm=0.0,       # mm — >0: the skis step, +-this about the middle.
                       #    The rocking sets the rhythm (user, 2026-11-03):
                       #    tilted onto the right ski (theta > dead_deg) the
                       #    right one is down and slides back, the left one
                       #    is up and slides forward; and the mirror. 0: held
    slide_v=150.0,     # mm/s — the slide's top speed (MG90D 10 rad/s on a
                       #    15 mm crank)
    dead_deg=0.3,      # deg — near upright both are down: hold
    slide_kp=3000.0,   # N/m — the slide's servo stiffness, and its damping
    slide_c=10.0,      #    N s/m: how hard it turns the ski round
    one_step=0.0,      # with step_mm: one step only, from the release onto the
                       #    right foot, then hold. 1: both skis (right back,
                       #    left forward); 2: only the right (stance) one;
                       #    3: only the left (raised) one
    hold_r=0.0,        # mm — where each ski starts (and stays, step_mm 0),
    hold_l=0.0,        #    + = forward of the middle
)


LIFT_MM = 0.5            # a raised ski must clear the floor this much to step


def ski_place(p):
    """Each ski's centre (mm): out from the middle, up from the floor. A
    line sits where the flat ski's outer edge was, so b stays the same."""
    if p["line_r"]:
        return p["gap"] / 2 + p["ski_w"], p["line_r"]
    return p["gap"] / 2 + p["ski_w"] / 2, p["ski_t"] / 2


def build(p):
    kg = lambda g: g / 1000.0
    mm = lambda v: v / 1000.0
    xc, zs = ski_place(p)                         # each ski's centre: out, up
    L2 = mm(p["ski_len"] / 2)
    legs, meshes = "", ""
    for side, s in (("R", 1), ("L", -1)):
        if p["line_r"] and p["sag_r"]:            # a rocker: an arc fore and
            R_, r_ = mm(p["sag_r"]), mm(p["line_r"])   # aft, a thin blade
            v = []
            ys = np.linspace(-L2, L2, 25)
            zb = [-r_ + R_ - math.sqrt(R_ * R_ - y * y) for y in ys]
            for y, z in zip(ys, zb):
                for x in (-r_, r_):
                    v += [x, y, z, x, y, max(zb) + 0.003]
            meshes += f'<mesh name="rock{side}" vertex="{" ".join(f"{q:.6f}" for q in v)}"/>'
            ski = (f'<geom name="ski{side}" type="mesh" mesh="rock{side}" mass="{kg(p["ski_g"])}"'
                   ' rgba=".95 .6 .2 1"/>')
        elif p["line_r"] and p["two_pts"]:        # two points: toe and heel
            ski = "".join(f'<geom name="ski{side}{"" if k == 0 else "b"}" type="sphere"'
                          f' pos="0 {y} 0" size="{mm(p["line_r"])}" mass="{kg(p["ski_g"] / 2)}"'
                          f' rgba=".95 .6 .2 1"/>' for k, y in enumerate((L2, -L2)))
            vr = mm(max(p["vis_r"], p["line_r"]))    # drawn bigger, resting on
            ski += "".join(f'<geom type="sphere" pos="0 {y} {vr - mm(p["line_r"])}" size="{vr}"'
                           ' mass="0.00001" contype="0" conaffinity="0" rgba=".95 .6 .2 .6"/>'
                           for y in (L2, -L2))         # the floor: only a picture
            ski += (f'<geom type="capsule" fromto="0 {-L2} 0 0 {L2} 0" size=".001" mass="0.00001"'
                    ' contype="0" conaffinity="0" rgba=".95 .6 .2 .3"/>')
        elif p["line_r"]:                         # a line: a thin rod along Y
            ski = (f'<geom name="ski{side}" type="capsule" fromto="0 {-L2} 0 0 {L2} 0"'
                   f' size="{mm(p["line_r"])}" mass="{kg(p["ski_g"])}" rgba=".95 .6 .2 1"/>')
        else:
            ski = (f'<geom name="ski{side}" type="box" size="{L2} {mm(p["ski_w"] / 2)} {mm(zs)}"'
                   f' euler="0 0 90" mass="{kg(p["ski_g"])}" rgba=".95 .6 .2 1"/>')
        legs += f"""
      <body name="ski{side}" pos="{mm(s * xc)} 0 {mm(zs - p['body_z'])}">
        <joint name="slide{side}" type="slide" axis="0 1 0" range="-.05 .05" damping="{p['slide_c']}"/>
        {ski}
        <geom type="capsule" fromto="0 0 {mm(zs)} 0 0 {mm(p['hip'] - zs)}" size=".003"
              mass="0.00001" contype="0" conaffinity="0" rgba=".95 .6 .2 1"/>
      </body>"""
    xml = f"""
<mujoco model="ski_duck">
  <option timestep="0.0002" gravity="0 0 -9.81"/>
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="256" height="256"
             rgb1=".85 .85 .85" rgb2=".75 .75 .75"/>
    <material name="grid" texture="grid" texrepeat="20 20"/>
    {meshes}
  </asset>
  <default>
    <geom contype="1" conaffinity="0" friction="{p['friction']} 0.005 0.0001" solref="{p['contact_tc']} {p['contact_damp']}"/>
  </default>
  <worldbody>
    <light pos="0 -0.3 0.6" dir="0 0.5 -1"/>
    <geom name="floor" type="plane" size="2 2 0.01" material="grid" contype="0" conaffinity="1"/>
    <body name="torso" pos="0 0 {mm(p['body_z'])}">
      <freejoint/>
      <geom name="body" type="box" size=".028 .028 .042" mass="{kg(p['body_g'])}"
            rgba=".75 .8 .9 1" contype="0"/>
      <geom name="nose" type="box" size=".006 .003 .006" pos="0 .031 .03"
            mass="0.0001" rgba=".9 .2 .2 1" contype="0"/>
      {legs}
    </body>
  </worldbody>
  <actuator>
    <position name="slideR" joint="slideR" kp="{p['slide_kp']}"/>
    <position name="slideL" joint="slideL" kp="{p['slide_kp']}"/>
  </actuator>
</mujoco>"""
    m = mujoco.MjModel.from_xml_string(xml)
    return m, mujoco.MjData(m)


def block(p):
    """The block's numbers, from the same masses: m, b, h, I0, alpha, p, r."""
    m_b, m_s = p["body_g"] / 1000, p["ski_g"] / 1000
    M = m_b + 2 * m_s
    b = (p["gap"] / 2 + p["ski_w"]) / 1000            # outer edge
    xc, zs = (v / 1000 for v in ski_place(p))
    h = (m_b * p["body_z"] / 1000 + 2 * m_s * zs) / M
    w, t = (0.0, 0.0) if p["line_r"] else (p["ski_w"] / 1000, p["ski_t"] / 1000)
    # about the right ski's outer edge (b, 0): each piece's own inertia
    # about X (box: m (w^2 + t^2) / 12; a line: none) plus m d^2
    I0 = m_b * (0.056 ** 2 + 0.084 ** 2) / 12 + m_b * (b ** 2 + (p["body_z"] / 1000) ** 2)
    for x in (xc, -xc):
        I0 += m_s * (w ** 2 + t ** 2) / 12
        I0 += m_s * ((b - x) ** 2 + zs ** 2)
    R = math.hypot(b, h)
    return dict(M=M, b=b, h=h, I0=I0, R=R, alpha=math.atan2(b, h),
                p=math.sqrt(M * 9.81 * R / I0), r=1 - 2 * M * b * b / I0)


def housner(k, theta0, secs, dt=1e-5):
    """The equations above, integrated: (t, theta) in rad, and the landings."""
    th, w, t = theta0, 0.0, 0.0
    out, hits = [], []
    while t < secs:
        s = 1.0 if th > 0 else -1.0
        acc = -k["p"] ** 2 * math.sin(k["alpha"] - abs(th)) * s
        w2 = w + acc * dt
        th2 = th + w2 * dt
        if th2 * th < 0:
            w2 *= k["r"]                             # lands on the other edge
            hits.append(t)
        th, w, t = th2, w2, t + dt
        if abs(th) < 1e-5 and abs(w) < 1e-4:
            th, w = 0.0, 0.0
        out.append((t, th))
    return np.array(out[::20]), hits


def release(p, theta0, view=False, hook=None):
    """Tilt the whole duck theta0 about the right ski's outer edge, at rest,
    and let go. Returns rows (t, roll, y, x, ski R, ski L, yaw), m, rad.
    hook(m, d), if given, runs before every step."""
    m, d = build(p)
    b = (p["gap"] / 2 + p["ski_w"]) / 1000
    z0 = p["body_z"] / 1000
    c, s = math.cos(theta0), math.sin(theta0)
    hold = np.array([p["hold_r"], p["hold_l"]]) / 1000
    def place():
        # the torso's origin (0, 0, z0) turned about the edge (b, 0, 0),
        # about Y (+ lifts the left ski): x' = x c + z s, z' = -x s + z c
        mujoco.mj_resetData(m, d)
        d.qpos[0] = b - b * c + z0 * s
        d.qpos[2] = b * s + z0 * c + 0.0001
        d.qpos[3:7] = [math.cos(theta0 / 2), 0, math.sin(theta0 / 2), 0]
        ph = math.radians(p["pitch0_deg"])
        if ph:                         # pitched too (+: nose down), about the
            if p["sag_r"]:             # rocker's centre, or the toe / heel
                cy, cz = 0.0, p["sag_r"] / 1000
            else:
                cy, cz = math.copysign(p["ski_len"] / 2000, ph), 0.0
            y0, z1 = d.qpos[1] - cy, d.qpos[2] - cz
            cph, sph = math.cos(-ph), math.sin(-ph)        # about X
            d.qpos[1] = cy + y0 * cph - z1 * sph
            d.qpos[2] = cz + y0 * sph + z1 * cph + 0.0005
            qx = np.array([math.cos(-ph / 2), math.sin(-ph / 2), 0, 0])
            qr = np.array(d.qpos[3:7])
            w1, x1, y1, z1_ = qx
            w2, x2, y2, z2 = qr
            d.qpos[3:7] = [w1 * w2 - x1 * x2 - y1 * y2 - z1_ * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1_ * y2,
                           w1 * y2 - x1 * z2 + y1 * w2 + z1_ * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1_ * w2]
        d.qpos[7:9] = d.ctrl[:] = hold          # the skis where they're held
        mujoco.mj_forward(m, d)
    place()
    rows = []
    def roll():
        R = d.xmat[1].reshape(3, 3)
        return math.atan2(R[0, 2], R[2, 2])
    A, dv = p["step_mm"] / 1000, p["slide_v"] / 1000 * m.opt.timestep
    dead = math.radians(p["dead_deg"])
    cmd = hold.copy()                      # right, left
    last = [theta0]                        # the roll a step ago (its rate)
    one = p["one_step"]
    def step():
        if A:                              # the rocking sets the rhythm
            th = roll()
            if one:                        # a single step from the release
                want = np.array([-A if one in (1, 2) else 0.0,  # (on the right
                                 A if one in (1, 3) else 0.0])  # foot), then hold
            elif p["switch_at_peak"]:     # switch at the rocking's ends: the
                rate = (th - last[0]) / m.opt.timestep   # feet cross the
                last[0] = th                             # landing mid-move
                if rate < -math.radians(p["rate_dead"]):   # falling back
                    want = np.array([-A, A])   # from the right: as on it
                elif rate > math.radians(p["rate_dead"]):
                    want = np.array([A, -A])
                else:
                    want = cmd
            elif th > dead:                # on the right ski: right back,
                want = np.array([-A, A])   # left (up) forward
            elif th < -dead:
                want = np.array([A, -A])
            else:
                want = cmd
            cmd[:] += np.clip(want - cmd, -dv, dv)
            d.ctrl[:] = cmd
        if hook:                           # studies: extra forces, logging
            hook(m, d)
        mujoco.mj_step(m, d)
    if view:
        from mujoco import viewer
        import time
        with viewer.launch_passive(m, d) as v:
            v.cam.distance, v.cam.elevation, v.cam.azimuth = 0.45, -10, 90
            v.cam.lookat[:] = [0, 0, 0.07]
            for rep in range(1000):
                if not v.is_running():
                    break
                place()
                cmd[:] = hold
                v.sync()
                time.sleep(1.0)                  # still, tilted
                wall0 = time.perf_counter()
                while v.is_running() and d.time < p["secs"]:
                    slow = 4.0                   # slow motion: x4
                    while d.time < (time.perf_counter() - wall0) / slow:
                        step()
                    v.cam.lookat[:] = [d.qpos[0], d.qpos[1], 0.07]
                    v.sync()
                    time.sleep(0.01)
        return None
    while d.time < p["secs"]:
        step()
        R = d.xmat[1].reshape(3, 3)
        rows.append((d.time, roll(), d.qpos[1], d.qpos[0], d.qpos[7], d.qpos[8],
                     math.atan2(-R[0, 1], R[1, 1])))
    return np.array(rows[::10])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tilt", type=float, default=8.0, help="deg, released from rest")
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--set", nargs="*", default=[], metavar="KEY=VALUE")
    a = ap.parse_args()
    p = dict(SKI)
    for kv in a.set:
        k, v = kv.split("=")
        assert k in p, f"unknown: {k}"
        p[k] = float(v)
    th0 = math.radians(a.tilt)
    if a.view:
        return release(p, th0, view=True)
    k = block(p)
    print(f"block: m {k['M'] * 1000:.0f} g, b {k['b'] * 1000:.1f} mm, h {k['h'] * 1000:.1f} mm, "
          f"alpha {math.degrees(k['alpha']):.1f} deg, p {k['p']:.2f} 1/s, r {k['r']:.3f} "
          f"(energy kept per landing {k['r'] ** 2 * 100:.0f} %)")
    sim = release(p, th0)
    eq, hits = housner(k, th0, p["secs"])
    # the sim's landings: roll changes sign
    sh = sim[np.where(np.sign(sim[:-1, 1]) * np.sign(sim[1:, 1]) < 0)[0], 0]
    print(f"landings  equations: {len(hits)}, last at {hits[-1] if hits else 0:.2f} s; "
          f"sim: {len(sh)}, last at {sh[-1] if len(sh) else 0:.2f} s")
    print("  half-rock times (s)  equations:", np.round(np.diff([0] + hits), 3)[:10])
    print("                       sim:      ", np.round(np.diff(np.r_[0, sh]), 3)[:10])
    if p["step_mm"]:
        # The equations' walk: the slides along Y don't touch the rocking, so
        # it dies as above; each half-rock the down ski pushes back as far as
        # it can in that time, at most 2 step_mm: the body goes that forward
        # A half-rock counts while the raised ski clears the floor by
        # LIFT_MM (2 b sin theta at its peak); the first starts from 0
        A, v = p["step_mm"], p["slide_v"]
        edges = [0] + hits
        half = np.diff(edges)
        peak = [np.abs(eq[(eq[:, 0] >= t0) & (eq[:, 0] < t1), 1]).max()
                for t0, t1 in zip(edges, edges[1:])]
        lift = [2 * k["b"] * 1000 * math.sin(x) for x in peak]
        push = [min(A if i == 0 else 2 * A, v * tau)
                for i, (tau, l) in enumerate(zip(half, lift)) if l >= LIFT_MM]
        print(f"walk  sim: forward {sim[-1, 2] * 1000 - sim[0, 2] * 1000:+.1f} mm, side "
              f"{(sim[-1, 3] - sim[0, 3]) * 1000:+.1f} mm, turn {math.degrees(sim[-1, 6]):+.1f} deg; "
              f"equations: forward {sum(push):+.1f} mm in {len(push)} half-rocks "
              f"({sum(1 for x in push if x >= 2 * A - 1e-9)} full steps)")
    os.makedirs(OUT, exist_ok=True)
    name = "ski_walk" if p["step_mm"] else "ski_release"
    with open(os.path.join(OUT, name + ".csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["src", "t", "theta_deg", "y_mm", "x_mm", "skiR_mm", "skiL_mm", "yaw_deg"])
        w.writerows(("sim", r[0], math.degrees(r[1]), r[2] * 1000, r[3] * 1000, r[4] * 1000,
                     r[5] * 1000, math.degrees(r[6])) for r in sim)
        w.writerows(("eq", t, math.degrees(x), "", "", "", "", "") for t, x in eq)
    json.dump(dict(params=p, tilt_deg=a.tilt, block={kk: float(v) for kk, v in k.items()},
                   hits_eq=hits, hits_sim=sh.tolist()),
              open(os.path.join(OUT, name + ".json"), "w"), indent=1)


if __name__ == "__main__":
    main()
