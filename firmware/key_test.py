# DAOKAI KEY test (O1.2-f): keeps the DAOKAI awake by "pressing" its KEY
# pin from the ESP32 instead of keeping the Wi-Fi radio on. Below 50 mA the
# board switches its output off after ~30 s; its listing says a pulse to GND
# on KEY every ~20 s keeps it on.
# KEY wired to GPIO3 (next to GPIO4, on the 5V/G side of the SuperMini).
# The pin is never driven high: it idles as an input (KEY's own pull-up holds
# it) and only goes low for the pulse, like the button would. One short
# press only: two within ~1 s switch the DAOKAI OFF.
# Wi-Fi stays off, servos idle: the ESP32 alone draws far below 50 mA, so if
# the output stays on past a minute it is the pulses that keep it awake.
# The on-board LED (GPIO8, active low) flashes once at every pulse.
# Copy it as main.py to run it from the battery (servos UNPLUGGED while on
# USB: the USB's 5 V reaches the split board through the ESP32's 5V pin):
#   mpremote connect COM4 cp key_test.py :main.py

import time
from machine import Pin

KEY = 3
EVERY_S = 15                # the listing says ~20 s; the cut-off is at ~30
PRESS_MS = 200              # a short press: long presses may do other things

led = Pin(8, Pin.OUT, value=1)
key = Pin(KEY, Pin.IN)      # released: no pull-up of ours


def press():
    key.init(Pin.OUT, value=0)
    led.value(0)
    time.sleep_ms(PRESS_MS)
    key.init(Pin.IN)
    led.value(1)


time.sleep(2)               # let the supply settle before the first press
n = 0
while True:
    press()
    n += 1
    print("press", n, "at", n * EVERY_S - EVERY_S, "s")
    time.sleep(EVERY_S)
