# Split board test: sweeps the four servo channels (GPIO 4, 5, 6, 7) through
# 1000 / 1500 / 2000 us, one after another, so each plug can be checked in
# turn: plug a servo into each and watch that only that one moves on its turn.
# The Wi-Fi radio stays on to keep the DAOKAI awake. The on-board LED (GPIO8,
# active low) blinks once per channel number: 1 blink = GPIO4 ... 4 = GPIO7.
# Copy it as main.py to run it from the battery:
#   mpremote connect COM4 cp four_servos_test.py :main.py

import network
import time
from machine import Pin, PWM

PINS = (4, 5, 6, 7)

ap = network.WLAN(network.AP_IF)        # keeps the DAOKAI awake
ap.active(True)
ap.config(essid="animo-test")

led = Pin(8, Pin.OUT, value=1)
servos = [PWM(Pin(p), freq=50, duty_ns=1500 * 1000) for p in PINS]


def blink(n):
    for _ in range(n):
        led.value(0)
        time.sleep(0.15)
        led.value(1)
        time.sleep(0.25)


while True:
    for i, servo in enumerate(servos):
        blink(i + 1)
        for us in (1000, 2000, 1500):
            servo.duty_ns(us * 1000)
            time.sleep(0.8)
        time.sleep(0.6)
