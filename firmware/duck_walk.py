# Duckling walker v1 (O2.2-a): two legs on the side walls, nothing else.
# Open loop, no sensors: the legs swing in scissors, +-LEG_DEG at FREQ, and
# the body rocks side to side on the doubly curved feet (cad/animo_duck.py).
# The values are the simulator's (sim/trex_sim.py, 2026-10-31; the study in
# docs/walker-sim/).
#
# MODE = "park": both legs to their centre, one at a time, and hold. Put the
#   legs on hanging straight down while it holds.
# MODE = "walk": the legs grow to full swing over FADE_S and keep going.
# MODE = "sweep_freq": walking through SWEEP_FREQ, STEP_S each. Before each
#   step the legs stop 2 s and the LED blinks the step's number (1, 2...).
#   Changes the rhythm without a USB upload for each try.
#
# Which plug drives which wall and which way is "forward" differ per cube:
# fill PINS and SIGN by hand (see notas/montaje-patito.md).
# The Wi-Fi radio stays on to keep the DAOKAI awake (O1.2-f).
# Copy it as main.py to run it from the battery (servos UNPLUGGED while on
# USB: the USB's 5 V reaches the split board through the ESP32's 5V pin):
#   mpremote connect COM4 cp duck_walk.py :main.py

import math
import network
import time
from machine import Pin, PWM

MODE = "park"

PINS = {"right": 4, "left": 5}          # GPIO of each wall's plug
SIGN = {"right": 1, "left": -1}         # + = foot forward
TRIM = {"right": 0, "left": 0}          # us: straight down at 0

US_PER_DEG = 10.0       # MG90D: ~1000 us for ~90 deg (to check)
# The simulator (2026-10-31): +-15 deg at 1.2 Hz walks ~10 cm/s on the
# rounded feet; the window is about +-5 % of rhythm. The right leg starts
# going BACK (phase 180): that start picks "forward" most often (~4 in 5;
# the rest walk backward or fall - pick it up and start again).
FREQ = 1.2              # Hz
LEG_DEG = 15            # the legs' swing each way (checked to 25: the macro's)
PHASE = 180             # deg: 180 = the right leg starts going back
LEAN = 0                # deg: both legs turned together, + = feet forward
FADE_S = 2              # s: the swing grows from 0 over this
SWEEP_FREQ = (1.10, 1.15, 1.20, 1.25, 1.30)
STEP_S = 20

led = Pin(8, Pin.OUT, value=1)
time.sleep(1)                           # let the supply settle

ap = network.WLAN(network.AP_IF)        # keeps the DAOKAI awake
ap.active(True)
ap.config(essid="animo-duck")

servo = {}
for name in ("right", "left"):          # one at a time: servos jumping to
    servo[name] = PWM(Pin(PINS[name]), freq=50)   # centre at once reset
    servo[name].duty_ns((1500 + TRIM[name]) * 1000)  # the ESP32 (2026-10-26)
    time.sleep(0.5)


def put(name, deg):
    us = 1500 + TRIM[name] + SIGN[name] * deg * US_PER_DEG
    servo[name].duty_ns(int(us * 1000))


if MODE == "park":
    while True:                         # slow blink: parked
        led.value(0)
        time.sleep(0.1)
        led.value(1)
        time.sleep(1.9)


def walk(freq, secs=None):
    """Legs from still, the swing growing over FADE_S; for secs, or for ever."""
    t0 = time.ticks_ms()
    while True:
        t = time.ticks_diff(time.ticks_ms(), t0) / 1000
        if secs and t > secs:
            return
        w = 2 * math.pi * freq * t
        legs = LEG_DEG * min(1.0, t / FADE_S)
        lean = LEAN * min(1.0, t / 2)
        step = legs * math.sin(w - math.radians(PHASE))
        put("right", lean + step)       # one forward, the other back
        put("left", lean - step)
        led.value(0 if step > 0 else 1)  # LED on: right foot ahead
        time.sleep_ms(20)


def pause(n):
    put("right", LEAN)
    put("left", LEAN)
    time.sleep(0.5)
    for _ in range(n):
        led.value(0)
        time.sleep(0.15)
        led.value(1)
        time.sleep(0.25)
    time.sleep(max(0, 1.5 - 0.4 * n))


if MODE == "sweep_freq":
    while True:
        for i, f in enumerate(SWEEP_FREQ):
            pause(i + 1)
            walk(f, STEP_S)
else:
    walk(FREQ)
