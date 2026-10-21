# Power test for the DAOKAI charge/boost board: does its 5 V stay on with the
# ESP32-C3 drawing a realistic current? Turns the Wi-Fi radio on as an access
# point (~80-100 mA) and blinks the on-board LED (GPIO8, active low), so you
# can see the ESP32 is really running and not just its power LED glowing.
# Copy it as main.py to run it from the battery:
#   mpremote connect COM4 cp power_test.py :main.py

import network
import time
from machine import Pin

led = Pin(8, Pin.OUT)

ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid="animo-test")

while True:
    led.value(0)
    time.sleep(0.1)
    led.value(1)
    time.sleep(0.9)
