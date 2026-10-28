# T-rex walker v1 (O2.2-a): legs on the side walls, tail on the back one.
# Open loop, no sensors: the tail swings at the rocking frequency to pump the
# side-to-side rocking on the arched feet, and the legs step in time with it
# (one leg forward while the other pushes back). The tuning is here, in
# FREQ / TAIL_DEG / LEG_DEG / LEG_PHASE, not in the printed feet.
#
# MODE = "park": every servo to its centre, one at a time, and hold. Put the
#   legs on hanging straight down and the tail level while it holds.
# MODE = "walk": tail alone for TAIL_ONLY_S, then the legs fade in.
# MODE = "sweep_freq": tail only, through SWEEP_FREQ, STEP_S each.
# MODE = "sweep_phase": walking at FREQ, through SWEEP_PHASE, STEP_S each.
#   Sweeps change things without a USB upload for each try: before each
#   step everything stops 2 s and the LED blinks the step's number (1, 2...).
#
# Which plug drives which wall and which way is "forward" differ per cube:
# fill PINS and SIGN by hand (see notas/montaje-trex.md).
# The Wi-Fi radio stays on to keep the DAOKAI awake (O1.2-f).
# Copy it as main.py to run it from the battery (servos UNPLUGGED while on
# USB: the USB's 5 V reaches the split board through the ESP32's 5V pin):
#   mpremote connect COM4 cp trex_walk.py :main.py

import math
import network
import time
from machine import Pin, PWM

MODE = "park"

# The "head" is a second tail on the front wall (the same printed part): it
# balances the tail fore and aft and swings with it. Without it the cube
# leans back onto its tail (MuJoCo, sim/trex_sim.py, 2026-10-28).
PINS = {"right": 4, "left": 5, "tail": 6, "head": 7}   # GPIO of each wall's plug
SIGN = {"right": 1, "left": -1, "tail": 1, "head": -1}  # legs: + = foot forward;
                                            # tail and head: both tips to the SAME side
TRIM = {"right": 0, "left": 0, "tail": 0, "head": 0}   # us: straight down at 0

US_PER_DEG = 10.0       # MG90D: ~1000 us for ~90 deg (to check)
# Starting values from the simulator (sim/trex_sim.py, 2026-10-28): it rocks
# most at ~1.0 Hz; tail and head at +-15 give +-15..20 deg of rocking, more
# and it falls over sideways. Walking is very sensitive: 1.0 Hz with the legs
# 180 deg behind went forward, but 5 % off or 20 deg off turned or backed.
FREQ = 1.0              # Hz: the rocking's own frequency
TAIL_DEG = 15           # tail and head swing each way (max 35: the macro's)
LEG_DEG = 15            # the legs' swing each way (max 20: the macro's)
LEG_PHASE = 180         # deg: how far the legs lag the tail
LEAN = 0                # deg: both legs turned together, to level the body:
                        # + = feet forward = the body leans FORWARD. With the
                        # head on, 0; a tail alone (6 g) wanted ~+5
TAIL_ONLY_S = 6         # s: rocking builds up before the legs join
FADE_S = 4              # s: legs fade in over this
SWEEP_FREQ = (0.8, 0.9, 1.0, 1.1, 1.2)
SWEEP_PHASE = (135, 160, 180, 200, 225)
STEP_S = 15

led = Pin(8, Pin.OUT, value=1)
time.sleep(1)                           # let the supply settle

ap = network.WLAN(network.AP_IF)        # keeps the DAOKAI awake
ap.active(True)
ap.config(essid="animo-trex")

servo = {}
for name in ("tail", "head", "right", "left"):  # one at a time: all four jumping to
    servo[name] = PWM(Pin(PINS[name]), freq=50)   # centre at once reset the
    servo[name].duty_ns((1500 + TRIM[name]) * 1000)  # ESP32 (2026-10-26)
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



def walk(freq, leg_deg, phase, secs=None):
    """Tail from rest, legs fading in; for secs, or for ever."""
    t0 = time.ticks_ms()
    while True:
        t = time.ticks_diff(time.ticks_ms(), t0) / 1000
        if secs and t > secs:
            return
        w = 2 * math.pi * freq * t
        tail = TAIL_DEG * min(1.0, t / 2)   # 2 s ramp from rest
        legs = leg_deg * min(1.0, max(0.0, (t - TAIL_ONLY_S) / FADE_S))
        lean = LEAN * min(1.0, t / 2)
        put("tail", tail * math.sin(w))
        put("head", tail * math.sin(w))
        step = legs * math.sin(w - math.radians(phase))
        put("right", lean + step)           # one forward, the other back
        put("left", lean - step)
        led.value(0 if math.sin(w) > 0 else 1)   # LED on: tail to one side
        time.sleep_ms(20)


def pause(n):
    put("tail", 0)
    put("head", 0)
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
            walk(f, 0, LEG_PHASE, STEP_S)
elif MODE == "sweep_phase":
    while True:
        for i, ph in enumerate(SWEEP_PHASE):
            pause(i + 1)
            walk(FREQ, LEG_DEG, ph, STEP_S)
else:
    walk(FREQ, LEG_DEG, LEG_PHASE)
