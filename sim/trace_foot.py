# The rail foot (2026-11-03, notas O2.2-a a.1): a foot that is ONLY the path
# the contact follows over the sole in today's gait. Walk the printed duck
# (rounded doubly curved feet, the firmware's slow start), record where each
# sole touches, average it by the step's phase into one path, and write it as
# points on the sole with their normals (out/rail_path.json). trex_sim.py
# with rail_r > 0 then builds each foot as capsules lying along that path:
# nothing else of the foot touches the floor. It is expected to fall.
#
#   python trace_foot.py                  record the path, then walk the rail
#   python trace_foot.py --walk           only walk the rail (path kept)
#   python trace_foot.py --view           the rail walking, in a window
#   python trace_foot.py --toes 4 [--view]   a fan of 4 toes added to the rail

import argparse
import json
import math
import os

import numpy as np

from trex_sim import P, Run, gait_angle, fmt, show

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")

# The printed duckling (cad/animo_duck.py) and its firmware (duck_walk.py,
# 2026-11-02): 5 s at 1.00 Hz, 12 s climbing to 1.2 Hz, +-15 deg
DUCK = dict(gait=0, tail=0, tail_deg=0, tail_only_s=0, fade_s=2, lean=0, head=0, foot_w_g=0,
            hip=92, ball_r=130, ball_rs=186, ball_gap=7, ball_y=0, cap_in=0.5, cap_out=60,
            cap_heel=40, cap_toe=80, cap_wide_from=-10, cap_wide_to=25, cap_heel_x=50,
            cap_toe_x=15, brain_down=0, leg_deg=15, freq=1.2, phase=180, foot_mu=0.5,
            foot_spin=5, cap_g=18, leg_g=6, freq0=1.0, start_s=5, chirp_s=12, secs=40)
DUCK_ARGS = " ".join(f"{k}={v:g}" for k, v in DUCK.items())

FROM_S = 20.0            # record once the rhythm is 1.2 Hz and settled
BINS = 72                # 5 deg of phase each
SPACING = 2.0            # mm between the rail's points


def sole(p, x, y):
    """The doubly curved sole's height (mm, hip frame) and outward normal."""
    R, Rs, h = p["ball_r"], p["ball_rs"], p["hip"]
    cx = p["ball_gap"] / 2
    a, b = math.sqrt(R * R - (x - cx) ** 2), math.sqrt(Rs * Rs - y * y)
    z = -h + R - a + Rs - b
    n = np.array([(x - cx) / a, y / b, -1.0])      # down and out: z = f(x, y)
    return z, n / np.linalg.norm(n)


def record(p):
    """The contact's mean place on the right sole (left mirrored, half a
    cycle later) in each phase bin, x y in mm, and the share of time down."""
    r = Run(p)
    m, d = r.m, r.d
    rows = []
    for i in range(int(p["secs"] / m.opt.timestep)):
        r.step()
        if r.fell:
            break
        if i % 4 or d.time < FROM_S:
            continue
        w = gait_angle(p, d.time) - math.radians(p["phase"])
        for k, b in enumerate(r.foot_body):
            pts = [d.contact[j].pos for j in range(d.ncon)
                   if b in (m.geom_bodyid[d.contact[j].geom1], m.geom_bodyid[d.contact[j].geom2])]
            phi = math.degrees(w + math.pi * k) % 360
            if not pts:
                rows.append((phi, 0, math.nan, math.nan))
                continue
            R = d.xmat[b].reshape(3, 3)
            q = R.T @ (np.mean(pts, axis=0) - d.xpos[b]) * 1000
            rows.append((phi, 1, q[0] * (1 if k == 0 else -1), q[1]))
    a = np.array(rows)
    edges = np.linspace(0, 360, BINS + 1)
    path = []
    for i in range(BINS):
        sel = a[(a[:, 0] >= edges[i]) & (a[:, 0] < edges[i + 1])]
        on = sel[sel[:, 1] == 1]
        share = len(on) / max(len(sel), 1)
        if share > 0.5:
            path.append((edges[i] + 2.5, share, on[:, 2].mean(), on[:, 3].mean()))
    return path, r.result()


def rail(p, path):
    """The path as one run of points SPACING apart (the stance, in phase
    order from touch-down to lift-off), lying on the sole."""
    ph = np.array([q[0] for q in path])
    gap = np.argmax(np.diff(np.r_[ph, ph[0] + 360]))     # the swing: no contact
    order = np.r_[np.arange(gap + 1, len(path)), np.arange(0, gap + 1)]
    xy = np.array([[path[i][2], path[i][3]] for i in order])
    xy = np.array([np.convolve(np.r_[[c[0]] * 2, c, [c[-1]] * 2], np.ones(5) / 5, "valid")
                   for c in xy.T]).T                     # a little smoothing
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    t = np.arange(0, s[-1] + 1e-9, SPACING)
    xy = np.stack([np.interp(t, s, xy[:, 0]), np.interp(t, s, xy[:, 1])], 1)
    pts, nrm = [], []
    for x, y in xy:
        z, n = sole(p, x, y)
        pts.append([x, y, z])
        nrm.append(n.tolist())
    return dict(points=pts, normals=nrm, length_mm=float(s[-1]),
                phase_from=float(ph[order[0]]), phase_to=float(ph[order[-1]]))


TOE_FROM = 0.5           # the toes leave the path this far along it
TOE_SPAN = (8, 52)       # mm: the fan's ends, x on the printed outline's toe
                         # end (inner at the tip, outer down the curve)
TOE_IN = 2.0             # mm inside the outline


def outline_toe(x):
    """The printed outline's toe end (trace_foot.DUCK): y at this x."""
    out, toe, w1, tx = DUCK["cap_out"], DUCK["cap_toe"], DUCK["cap_wide_to"], DUCK["cap_toe_x"]
    if x <= tx:
        return toe
    u = (x - tx) / (out - tx)                   # the quarter ellipse
    return w1 + (toe - w1) * math.sqrt(max(1 - u * u, 0))


def toes(p, out, n):
    """n straight toes (seen from below) fanning from the middle of the path
    to the toe end of the sole, evenly along it, lying on the sole
    (2026-11-03, user: two toes, then more)."""
    a = np.array(out["points"])
    x0, y0 = a[int(TOE_FROM * (len(a) - 1)), :2]
    xs = np.linspace(*TOE_SPAN, 400)            # the outline, by arc length
    e = np.array([[x, outline_toe(x)] for x in xs])
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(e, axis=0), axis=1))]
    res = []
    for t in np.linspace(0, s[-1], n):
        x1, y1 = np.interp(t, s, e[:, 0]), np.interp(t, s, e[:, 1])
        d = math.hypot(x1 - x0, y1 - y0)
        x1, y1 = x1 - (x1 - x0) * TOE_IN / d, y1 - (y1 - y0) * TOE_IN / d
        m = max(2, int(math.hypot(x1 - x0, y1 - y0) / SPACING) + 1)
        pts, nrm = [], []
        for u in np.linspace(0, 1, m):
            x, y = x0 + (x1 - x0) * u, y0 + (y1 - y0) * u
            z, nv = sole(p, x, y)
            pts.append([x, y, z])
            nrm.append(nv.tolist())
        res.append(dict(points=pts, normals=nrm))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--walk", action="store_true", help="only walk the rail")
    ap.add_argument("--r", type=float, default=1.5, help="rail radius, mm")
    ap.add_argument("--view", action="store_true", help="walk the rail in a window")
    ap.add_argument("--toes", type=int, default=0, help="with this many toes (2 drawn)")
    a = ap.parse_args()
    p = dict(P, **DUCK, rail_toes=a.toes)
    if a.view:
        return show(dict(p, rail_r=a.r))
    if a.toes:                          # the toes onto the recorded path
        f = os.path.join(OUT, "rail_path.json")
        out = json.load(open(f))
        if not isinstance(out.get("toes"), dict):
            out["toes"] = {}
        out["toes"][str(a.toes)] = toes(p, out, a.toes)
        json.dump(out, open(f, "w"), indent=1)
    elif not a.walk:
        path, res = record(p)
        print("printed duck, today's gait: " + fmt(res))
        out = rail(p, path)
        os.makedirs(OUT, exist_ok=True)
        json.dump(out, open(os.path.join(OUT, "rail_path.json"), "w"), indent=1)
        xs = [q[0] for q in out["points"]]
        ys = [q[1] for q in out["points"]]
        print(f"rail: {len(xs)} points, {out['length_mm']:.0f} mm long, phase "
              f"{out['phase_from']:.0f} -> {out['phase_to']:.0f} deg; x {min(xs):.0f}..{max(xs):.0f}, "
              f"y {min(ys):.0f}..{max(ys):.0f} mm")
    print(f"rail r {a.r} mm: " + fmt(Run(dict(p, rail_r=a.r)).go()))


if __name__ == "__main__":
    main()
