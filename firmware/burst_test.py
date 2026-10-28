# DAOKAI wake-up by Wi-Fi bursts (O1.2-f, plan B: the DAOKAI has no KEY pad,
# KEY is only the button's own line). Below 50 mA the board switches its
# output off after ~30 s; the Wi-Fi radio on all the time keeps it awake
# (2026-10-21). Here the radio only comes on for BURST_S every EVERY_S, to
# see whether a short burst over 50 mA restarts the DAOKAI's timer.
# Servos idle: between bursts the ESP32 alone draws far below 50 mA.
# The on-board LED (GPIO8, active low) stays lit during each burst.
# Copy it as main.py to run it from the battery (servos UNPLUGGED while on
# USB: the USB's 5 V reaches the split board through the ESP32's 5V pin):
#   mpremote connect COM4 cp burst_test.py :main.py

import network
import time
from machine import Pin

EVERY_S = 15                # the cut-off is at ~30 s
BURST_S = 2                 # if it fails, try 5 before giving up

led = Pin(8, Pin.OUT, value=1)
ap = network.WLAN(network.AP_IF)

time.sleep(1)               # let the supply settle
n = 0
while True:
    led.value(0)
    ap.active(True)
    ap.config(essid="animo-test")
    time.sleep(BURST_S)
    ap.active(False)
    led.value(1)
    n += 1
    print("burst", n, "at", (n - 1) * EVERY_S, "s")
    time.sleep(EVERY_S - BURST_S)
