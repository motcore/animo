# Robust gait search for the T-rex: a gait is only worth printing for if it
# still walks when the real cube differs from the model. Each candidate runs
# in SCENARIOS (the rhythm off by 5 %, other floors, other masses, a lighter
# head) and is scored on its WORST case.
#
#   python robust.py                     grid search, best 12
#   python robust.py --check freq=1.0 phase=180 ...   one gait, every scenario

import argparse
import itertools
import multiprocessing as mp

from trex_sim import P, Run

SECS = 40.0             # four rounds of the three-phase gait
SCENARIOS = {
    "nominal":      {},
    "rhythm -5%":   {"freq_k": 0.95},
    "rhythm +5%":   {"freq_k": 1.05},
    "slippery":     {"friction": 0.5},
    "grippy":       {"friction": 1.1},
    "heavy brain":  {"brain_g": 75.0, "cube_g": 110.0},
    "light head":   {"head_g": 4.5},
}

GRID = dict(
    tail_deg=[6, 9, 12, 15],
    leg_deg=[6, 10, 15, 20],
    phase=[0, 45, 90, 135, 180, 225, 270, 315],
)


def run_one(args):
    gait, scen = args
    p = dict(P, **gait)
    p["secs"] = SECS
    k = scen.get("freq_k", 1.0)
    p.update({a: b for a, b in scen.items() if a != "freq_k"})
    p["freq"] = gait.get("freq", P["freq"]) * k
    return Run(p).go()


def score(results):
    """Worst forward progress, minus turning; a fall sinks it."""
    if any(r["fell"] for r in results):
        return -1000.0 + sum(not r["fell"] for r in results)
    worst = min(r["fwd_mm"] for r in results)
    turn = max(abs(r["turn_deg"]) for r in results)
    return worst - 2.0 * turn


def evaluate(gaits, pool):
    jobs = [(g, s) for g in gaits for s in SCENARIOS.values()]
    out = pool.map(run_one, jobs)
    n = len(SCENARIOS)
    return [(score(out[i * n:(i + 1) * n]), g, out[i * n:(i + 1) * n])
            for i, g in enumerate(gaits)]


def line(sc, g, res):
    fw = " ".join(f"{r['fwd_mm']:+5.0f}{'F' if r['fell'] else ' '}" for r in res)
    tn = max(abs(r["turn_deg"]) for r in res)
    return f"{sc:+8.1f}  {g}  fwd[{fw}]  max turn {tn:4.0f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", nargs="*", metavar="KEY=VALUE")
    ap.add_argument("--grid", nargs="*", default=[], metavar="KEY=v1,v2")
    ap.add_argument("--set", nargs="*", default=[], metavar="KEY=VALUE")
    a = ap.parse_args()
    base = {k: float(v) for k, v in (kv.split("=") for kv in a.set)}
    with mp.Pool() as pool:
        if a.check is not None:
            g = dict(base, **{k: float(v) for k, v in (kv.split("=") for kv in a.check)})
            sc, g, res = evaluate([g], pool)[0]
            print(f"score {sc:+.1f}  {g}")
            for name, r in zip(SCENARIOS, res):
                print(f"  {name:12s} fwd {r['fwd_mm']:+6.0f} mm  side {r['side_mm']:+5.0f}"
                      f"  turn {r['turn_deg']:+5.0f}  rock +-{r['roll_amp']:4.1f}"
                      f"{'  FELL' if r['fell'] else ''}")
            return
        grid = {} if a.grid else dict(GRID)     # --grid replaces the default
        for kv in a.grid:
            k, v = kv.split("=")
            grid[k] = [float(x) for x in v.split(",")]
        keys = list(grid)
        gaits = [dict(base, **dict(zip(keys, vals)))
                 for vals in itertools.product(*grid.values())]
        print(f"{len(gaits)} gaits x {len(SCENARIOS)} scenarios: {list(SCENARIOS)}")
        ranked = sorted(evaluate(gaits, pool), key=lambda t: -t[0])
        for sc, g, res in ranked[:12]:
            print(line(sc, {k: g[k] for k in keys}, res))


if __name__ == "__main__":
    main()
