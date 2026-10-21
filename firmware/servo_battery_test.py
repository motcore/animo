# First servo on the battery: the DAOKAI's 5 V feeds the ESP32-C3 and the
# servo; the Wi-Fi radio stays on, or the DAOKAI switches itself off within
# seconds (power_test.py). Sweeps one servo on GPIO4 through 1000 / 1500 /
# 2000 us forever and blinks the on-board LED (GPIO8, active low) at each step.
# Wiring: signal (orange) to GPIO4, red to 5V, brown to G.
# Copy it as main.py to run it from the battery:
#   mpremote connect COM4 cp servo_battery_test.py :main.py

import network
import time
from machine import Pin, PWM

SERVO_PIN = 4

ap = network.WLAN(network.AP_IF)        # keeps the DAOKAI awake
ap.active(True)
ap.config(essid="animo-test")

led = Pin(8, Pin.OUT)
servo = PWM(Pin(SERVO_PIN), freq=50)

while True:
    for us in (1000, 1500, 2000, 1500):
        servo.duty_ns(us * 1000)
        led.value(0)
        time.sleep(0.1)
        led.value(1)
        time.sleep(1.9)
