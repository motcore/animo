# Animo v4 family — mixed-cube check (2026-11-13)
# Can wall modules of different kinds sit side by side in one cube? Loads the
# north module of v4 (straight N20), v4b1 (continuous MG90D) and v4c (worm N20
# lying flat), puts one on the north wall and another on the east, west or
# south wall, and looks for any overlap, and for the north module hitting the
# other while it slides out. All 27 pairs passed once v4c's shaft moved to
# 16.4 off the output.
# Run headless:
#   "C:/Program Files/FreeCAD 1.0/bin/freecadcmd.exe" animo_v4_mix_check.py
import itertools
import os
import FreeCAD as App

CAD = os.environ.get("ANIMO_CAD") or os.path.dirname(os.path.abspath(__file__))
TYPES = {"v4": "animo_v4_cube.py", "v4b1": "animo_v4b1_cube.py", "v4c": "animo_v4c_cube.py"}
MOD = {}
for k, f in TYPES.items():
    src = open(os.path.join(CAD, f), encoding="utf-8").read()
    src = src.replace("EXPORT_STL = True", "EXPORT_STL = False").replace(
        "RUN_CHECKS = None", "RUN_CHECKS = False")
    ns = {"__name__": "mix_" + k, "__file__": os.path.join(CAD, f)}
    exec(compile(src, f, "exec"), ns)
    MOD[k] = [(n, sh) for n, sh, c, t in ns["MODULE"]]
    print(f"loaded {k}: {len(MOD[k])} parts")

ROT = {"N": 0, "W": 90, "S": 180, "E": 270}


def place(sh, rot):
    s = sh.copy()
    if rot:
        s.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), rot)
    return s


def hits(a_parts, b_parts):
    out = []
    for na, sa in a_parts:
        for nb, sb in b_parts:
            if sa.BoundBox.intersect(sb.BoundBox):
                v = sa.common(sb).Volume
                if v > 1e-3:
                    out.append(f"{na}/{nb} {v:.1f} mm3")
    return out


def slide_hits(a_parts, b_parts):
    """Module A (north) slides out +Y past module B."""
    out = set()
    for na, sa in a_parts:
        if na.startswith("Tech"):
            continue
        for dy in range(1, 40, 2):
            s = sa.copy()
            s.translate(App.Vector(0, dy, 0))
            for nb, sb in b_parts:
                if nb.startswith("Tech"):
                    continue
                if s.BoundBox.intersect(sb.BoundBox) and s.common(sb).Volume > 1e-3:
                    out.add(f"{na}/{nb}")
    return sorted(out)


print("=" * 72)
bad = 0
for a, b in itertools.product(TYPES, TYPES):
    A = [(n, place(s, ROT["N"])) for n, s in MOD[a]]
    for side in ("E", "W", "S"):
        B = [(n, place(s, ROT[side])) for n, s in MOD[b]]
        h = hits(A, B)
        sl = slide_hits(A, B) if side != "S" else []
        ok = not h and not sl
        bad += not ok
        msg = "OK " if ok else "FAIL"
        extra = ""
        if h:
            extra += " clash: " + "; ".join(h[:6]) + (" ..." if len(h) > 6 else "")
        if sl:
            extra += " slide-out: " + ", ".join(sl[:6])
        print(f"  [{msg}] N={a:5s} {side}={b:5s}{extra}")
print("-" * 72)
print(f"  {bad} failing pairs of {len(TYPES) ** 2 * 3}")
print("=" * 72)
