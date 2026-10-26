# Split board test: moves the four servo channels (GPIO 4, 5, 6, 7) one at a
# time through 1000 / 2000 / 1500 us, so each plug can be checked in turn:
# plug a servo into each and watch that only that one moves on its turn.
# No pulses at start-up and each servo is let go after its turn: four servos
# jumping to centre at once drew more than the DAOKAI gives and the ESP32
# kept resetting (2026-10-26). The Wi-Fi radio stays on to keep the DAOKAI
# awake. The on-board LED (GPIO8, active low) blinks the channel number
# first: 1 blink = GPIO4 ... 4 = GPIO7.
# Copy it as main.py to run it from the battery (servos UNPLUGGED while on
# USB: the USB's 5 V reaches the split board through the ESP32's 5V pin):
#   mpremote connect COM4 cp four_servos_test.py :main.py

import network
import time
from machine import Pin, PWM

PINS = (4, 5, 6, 7)

led = Pin(8, Pin.OUT, value=1)
time.sleep(1)                           # let the supply settle

ap = network.WLAN(network.AP_IF)        # keeps the DAOKAI awake
ap.active(True)
ap.config(essid="animo-test")

for p in PINS:                          # no pulses yet: plain low outputs
    Pin(p, Pin.OUT, value=0)


def blink(n):
    for _ in range(n):
        led.value(0)
        time.sleep(0.15)
        led.value(1)
        time.sleep(0.25)


while True:
    for i, p in enumerate(PINS):
        blink(i + 1)
        servo = PWM(Pin(p), freq=50)    # set the pulse after: a duty given
        for us in (1000, 2000, 1500):   # here leaves it "inactive" (v1.29)
            servo.duty_ns(us * 1000)
            time.sleep(0.8)
        servo.deinit()                  # let it go: it stops drawing current
        Pin(p, Pin.OUT, value=0)        # (a PWM can't idle at duty 0 here)
        time.sleep(0.6)
