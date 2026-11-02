# The cube's servo calibration: what the dial (cad/animo_dial.py) reads on
# each wall at 1050 / 1500 / 1950 us (firmware/calibrate.py, the guide in
# notas/montaje-calibrar.md). Degrees as read: 0 = needle straight down,
# + = clockwise seen from outside the wall.
# The walk firmwares turn degrees into us through us() below, so every wall
# moves the same for the same angle. Copy it to the board next to them:
#   mpremote connect COM4 cp cal.py :cal.py
# A wall module moved to another plug takes its line with it.

READ = {                # GPIO: (at 1050, at 1500, at 1950)
    4: (57.5, 1.0, -52.5),      # west, 2026-11-01; back from 1050 it stops 2-3 high (backlash)
    5: (57.5, 0.0, -57.5),      # north, 2026-11-01
    6: (55.0, -5.0, -60.0),     # south, 2026-11-01
    7: (55.0, -6.0, -65.0),     # east, 2026-11-01, read twice; no backlash felt
}


def us(pin, deg):
    """The pulse that puts this output at deg (0 = cross square, + = clockwise
    seen from outside). Straight lines through the three readings: each side
    of the centre has its own slope."""
    lo, mid, hi = READ[pin]
    t = deg - mid
    if (t > 0) == (hi > mid):
        return 1500 + 450 * t / (hi - mid)
    return 1500 - 450 * t / (lo - mid)
