# Paths over 120 s in every scenario, for two gaits (python paths.py out.json)
import sys, json, multiprocessing as mp
sys.path.insert(0, r"C:\Users\jjavi\dev\motcore\animo\sim")
from trex_sim import P, Run
from robust import SCENARIOS
GAITS = {"ball (sag 20)": dict(sag=20, head_phase=0, phase=135, head_deg=40, leg_deg=6),  # best for each sole
         "flatter (sag 150)": dict(sag=150, head_phase=135, phase=315, head_deg=40, leg_deg=6)}
def one(a):
    gname, sname = a
    p = dict(P, **GAITS[gname]); sc = SCENARIOS[sname]
    p.update({k: v for k, v in sc.items() if k != "freq_k"}); p["freq"] *= sc.get("freq_k", 1)
    p["secs"] = 120
    r = Run(p); n = int(0.1 / r.m.opt.timestep); xy = []
    for i in range(int(p["secs"] / 0.1)):
        for _ in range(n): r.step()
        xy.append([float(r.d.qpos[0]), float(r.d.qpos[1])])
        if r.fell: break
    return gname, sname, xy, r.fell
if __name__ == "__main__":
    with mp.Pool() as pool:
        out = pool.map(one, [(g, s) for g in GAITS for s in SCENARIOS])
    json.dump(out, open(sys.argv[1], "w"))
