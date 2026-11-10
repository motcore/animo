# The slip model against the simulator, as a picture (notas/patito_giro_resbalar.png):
# heading over time while turning, and turn rate against the steps' difference.
#   .venv/Scripts/python.exe slip_plot.py data        runs both, writes slip_plot.json
#   "C:/Program Files/FreeCAD 1.0/bin/python.exe" slip_plot.py      draws it
import json
import os
import sys
import math
import multiprocessing as mp
import numpy as np
DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "slip_plot.json")

DIFFS = (0, 10, 15, 20, -20)


def job(diff):
    from yaw_duck import YAW, release
    from yaw_clock_model import BEST
    import yaw_slip_model as sm
    p = dict(YAW, **dict(BEST, secs=20.0, turn_s=6.0, step_diff=diff))
    a = release(p, 0.0)
    even = sm.run(p, secs=20.0, pitch=False)
    lean = sm.run(p, secs=20.0)
    k = slice(None, None, 25)
    return dict(diff=diff, t=a[k, 0].tolist(), h=np.degrees(np.unwrap(a[:, 2]))[k].tolist(),
                te=even[:, 0].tolist(), he=np.degrees(even[:, 1]).tolist(),
                tl=lean[:, 0].tolist(), hl=np.degrees(lean[:, 1]).tolist())


def rate(t, h):
    t, h = np.asarray(t), np.asarray(h)
    i0 = np.searchsorted(t, 9)
    return (h[-1] - h[i0]) / (t[-1] - t[i0])


if __name__ == "__main__" and sys.argv[1:] == ["data"]:
    with mp.Pool() as pool:
        json.dump(pool.map(job, DIFFS), open(DATA, "w"))
    print(DATA)
elif __name__ == "__main__":
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    res = {r["diff"]: r for r in json.load(open(DATA))}
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.6))
    cols = {10: "#2a7ab9", 20: "#c0392b"}
    for d in (10, 20):
        r = res[d]
        ax1.plot(r["t"], r["h"], color=cols[d], lw=2, label=f"sim, step diff {d} mm")
        ax1.plot(r["tl"], r["hl"], color=cols[d], lw=1.5, ls="--",
                 label=f"model, toe/heel from the lean, {d} mm")
        ax1.plot(r["te"], r["he"], color=cols[d], lw=1, ls=":",
                 label=f"model, toe/heel half and half, {d} mm")
    ax1.axvline(6, color="0.6", lw=1)
    ax1.text(6.2, -12, "turning starts", color="0.4", fontsize=9)
    ax1.set_xlabel("time (s)")
    ax1.set_ylabel("heading (deg, + left)")
    ax1.set_title("Heading while turning")
    ax1.legend(fontsize=7.5, loc="upper left")
    ax1.grid(alpha=0.3)

    xs = sorted(DIFFS)
    sim = [rate(res[d]["t"], res[d]["h"]) for d in xs]
    ev = [rate(res[d]["te"], res[d]["he"]) for d in xs]
    le = [rate(res[d]["tl"], res[d]["hl"]) for d in xs]
    ax2.plot(xs, sim, "o-", color="k", lw=2, label="sim")
    ax2.plot(xs, le, "s--", color="#c0392b", label="model, toe/heel from the lean")
    ax2.plot(xs, ev, "^:", color="0.5", label="model, toe/heel half and half")
    ax2.axhline(0, color="0.7", lw=0.8)
    ax2.set_xlabel("right step longer than left by (mm)")
    ax2.set_ylabel("turn rate (deg/s, + left)")
    ax2.set_title("Turn rate: the toe/heel split is what was missing")
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    out = r"C:\Users\jjavi\dev\motcore\notas\patito_giro_resbalar.png"
    fig.savefig(out, dpi=130)
    print(out)
