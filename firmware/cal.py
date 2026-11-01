# The cube's servo calibration: what the dial (cad/animo_dial.py) reads on
# each wall at 1050 / 1500 / 1950 us (firmware/calibrate.py, the guide in
# notas/montaje-calibrar.md). Degrees as read: 0 = needle straight down,
# + = clockwise seen from outside the wall.
# The walk firmwares turn degrees into us through us() below, so every wall
# moves the same for the same angle. Copy it to the board next to them:
#   mpremote connect COM4 cp cal.py :cal.py
# A wall module moved to another plug takes its line with it.

READ = {                # GPIO: (at 1050, at 1500, at 1950) — not read yet
    4: (-45.0, 0.0, 45.0),
    5: (-45.0, 0.0, 45.0),
    6: (-45.0, 0.0, 45.0),
    7: (-45.0, 0.0, 45.0),
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
