# Servo calibration with the dial (cad/animo_dial.py; the guide in
# notas/montaje-calibrar.md). Every output turns the same once cal.py holds
# what the dial reads.
#
# MODE = "read": raw pulses, cal.py ignored. All four to 1500 us, one at a
#   time; then, wall after wall, the LED blinks the wall's number (1 = GPIO4
#   ... 4 = GPIO7) and that output goes 1500 -> 1050 -> 1500 -> 1950 -> 1500,
#   HOLD_S at each. Read the dial at 1050, 1500 and 1950 and write them in
#   cal.py. The others hold 1500. Then it starts over.
# MODE = "check": the same round in degrees through cal.py: 0, -45, 0, +45.
#   The dial should read just that on every wall.
# MODE = "centre": all four to 1500 us (raw) and hold, to reseat a cross.
#
# The Wi-Fi radio stays on to keep the DAOKAI awake (O1.2-f). Servos
# UNPLUGGED while on USB. Copy both files, then run from the battery:
#   mpremote connect COM4 cp calibrate.py :main.py
#   mpremote connect COM4 cp cal.py :cal.py

import network
import time
from machine import Pin, PWM

MODE = "read"

PINS = (4, 5, 6, 7)
HOLD_S = 6              # s at each position: time to read the dial
CHECK_DEG = 45

led = Pin(8, Pin.OUT, value=1)
time.sleep(1)                           # let the supply settle

ap = network.WLAN(network.AP_IF)        # keeps the DAOKAI awake
ap.active(True)
ap.config(essid="animo-cal")

servo = {}
for p in PINS:                          # one at a time: servos jumping to
    servo[p] = PWM(Pin(p), freq=50)     # centre at once reset the ESP32
    servo[p].duty_ns(1500 * 1000)       # (2026-10-26)
    time.sleep(0.5)


def put_us(p, us):
    servo[p].duty_ns(int(us * 1000))


def blink(n):
    for _ in range(n):
        led.value(0)
        time.sleep(0.15)
        led.value(1)
        time.sleep(0.25)


if MODE == "centre":
    while True:                         # slow blink: holding
        blink(1)
        time.sleep(1.6)

if MODE == "check":
    import cal
    STEPS = [cal.us(p, d) for p in PINS for d in (0, -CHECK_DEG, 0, CHECK_DEG, 0)]
    for p in PINS:
        put_us(p, cal.us(p, 0))
        time.sleep(0.5)
else:
    STEPS = [us for p in PINS for us in (1500, 1050, 1500, 1950, 1500)]

while True:
    for i, p in enumerate(PINS):
        blink(i + 1)
        time.sleep(2)
        for us in STEPS[5 * i:5 * i + 5]:
            put_us(p, us)
            led.value(0)                # LED on: moving to it
            time.sleep(0.3)
            led.value(1)
            time.sleep(HOLD_S - 0.3)
