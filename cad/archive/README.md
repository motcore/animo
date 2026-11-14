# Archived cubes

Earlier versions of the cube, kept for reference. The current design is
`../animo_brick.py` with its face files (`../animo_face_*.py`) and
`../animo_lib.py`. These macros still run from here (their STLs land in
`archive/stl/`).

| File | What it was |
|------|-------------|
| `animo_v1_cube.py` | 56 mm, female D output and magnets, brain as a half cube on top |
| `animo_v2_cube.py` | 56 mm, LEGO Technic faces (cross output, pin holes), servo pinwheel, 1:1 gears |
| `animo_v3_cube.py` | 72 mm, brain inside, AS5600 on a 1:1 sensor gear |
| `animo_v4_cube.py` | first double swing-gear clutch, straight N20 (now `animo_face_n20.py`) |
| `animo_v4b1_cube.py` | the same with continuous MG90D servos |
| `animo_v4c_cube.py` | the same with a worm N20 lying flat (now `animo_face_n20_worm.py`) |
| `animo_v4_mix_check.py` | v4 / v4b1 / v4c walls side by side (now `ANIMO_MIX=1 animo_brick.py`) |
| `animo_v5_cube.py` | v4b1 on a 6700ZZ bearing, trapezoid coupling, no LEGO (now `animo_face_mg90d.py`) |
