# Animo brick — the frame, and which face goes on which side
# FreeCAD Python Macro. Pick a face type for each side in CUBE below and run
# it: the brick is assembled with its faces, shown (one group per side) and,
# headless, checked as a whole — interference, the arms at their stops, every
# face sliding out, the battery and the boards' room. The decks' STL goes to
# stl/animo_brick/.
#
# Face types (one file each, see animo_lib.py for what they share):
#   "mg90d"     animo_face_mg90d.py     continuous MG90D (the one being built)
#   "n20"       animo_face_n20.py       straight N20
#   "n20_worm"  animo_face_n20_worm.py  worm N20 lying flat on its wall plate
#
# ANIMO_MIX=1 (headless) instead tries every type on the north face against
# every type on the east, west and south faces: overlap and sliding out only.
#
# Headless:  "C:/Program Files/FreeCAD 1.0/bin/freecadcmd.exe" animo_brick.py

import importlib
import itertools
import os
import sys

import FreeCAD as App

_HERE = os.path.dirname(os.path.abspath(
    globals().get("__file__") or os.path.join(App.getUserMacroDir(True), "_")))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import animo_lib as lib          # noqa: E402
importlib.reload(lib)

CUBE = {"N": "mg90d", "W": "mg90d", "S": "mg90d", "E": "mg90d"}

FILES = {"mg90d": "animo_face_mg90d", "n20": "animo_face_n20", "n20_worm": "animo_face_n20_worm"}


def load(kind):
    """A wall type, from its file, without running that file's own cube."""
    sys._animo_importing = True
    try:
        mod = importlib.import_module(FILES[kind])
        mod = importlib.reload(mod)
    finally:
        sys._animo_importing = False
    return mod.wall()


def mix():
    """Every type on the north wall against every type beside and opposite it."""
    types = {k: load(k) for k in FILES}
    out = {"N": (0, 1)}
    bad = 0
    print("=" * 76)
    for a, b in itertools.product(types, types):
        A = [(f"{n}_N", sh) for n, sh, c, t in types[a].parts if n != "ModelKey"]
        for side, rot in lib.SIDES[1:]:
            B = [(f"{n}_{side}", lib.place(sh, rot)) for n, sh, c, t in types[b].parts
                 if n != "ModelKey"]
            by = dict(A + B)
            clash = [f"{na}/{nb}" for na, sa in A for nb, vol in lib._hits(sa, [x for x, y in B], by)]
            slide = set()
            if side != "S":
                for na, sa in A:
                    for k in range(1, 40, 2):
                        s = sa.copy()
                        s.translate(lib.v(0, k, 0))
                        slide.update(f"{na}/{nb}" for nb, vol in lib._hits(s, [x for x, y in B], by))
            ok = not clash and not slide
            bad += not ok
            extra = (" clash: " + ", ".join(clash[:5]) if clash else "") + \
                    (" slide-out: " + ", ".join(sorted(slide)[:5]) if slide else "")
            print(f"  [{'OK ' if ok else 'FAIL'}] N={a:9s} {side}={b:9s}{extra}")
    print(f"  {bad} failing of {len(types) ** 2 * 3}")
    print("=" * 76)


if os.environ.get("ANIMO_MIX") == "1":
    mix()
else:
    walls = {}
    cache = {}
    for side, kind in CUBE.items():
        if kind not in cache:
            cache[kind] = load(kind)
        walls[side] = cache[kind]
    parts, deck, z0, board = lib.assemble(walls)
    lib.show("animo_brick", parts, show_only=None)
    pdir = os.path.join(_HERE, "stl", "animo_brick", lib.MATERIAL)
    os.makedirs(pdir, exist_ok=True)
    deck.exportStl(os.path.join(pdir, "animo_brick_Deck.stl"))
    if not getattr(App, "GuiUp", 0):
        lib.check(walls, parts, z0, board, None,
                  "Animo brick — " + ", ".join(f"{s} {k}" for s, k in CUBE.items()))
    else:
        print("animo_brick: geometry only, checks skipped (they run headless)")
