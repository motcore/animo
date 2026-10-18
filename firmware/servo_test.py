# Servo test for the ESP32-C3: sweeps one servo through 1000/1500/2000 us.
# Wiring: signal (orange) to GPIO4, red to 5V, brown to GND.
# Run without copying to the board: mpremote connect COM4 run servo_test.py

from machine import Pin, PWM
import time

SERVO_PIN = 4

servo = PWM(Pin(SERVO_PIN), freq=50)


def pulse(us):
    servo.duty_ns(us * 1000)


try:
    for _ in range(10):
        for us in (1000, 1500, 2000, 1500):
            print(us)
            pulse(us)
            time.sleep(2)
finally:
    servo.deinit()
