import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

TRIG = 23
ECHO = 24

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def get_distance():
    GPIO.output(TRIG, False)
    time.sleep(0.5)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    echo_duration = pulse_duration * 1000
    distance = pulse_duration * 17150

    distance = round(distance, 2)
    echo_duration = round(echo_duration, 2)

    return distance, echo_duration

try:
    while True:
        distance, echo_duration = get_distance()
        print("Distance:", distance, "cm" "  |  Echo duration:", echo_duration, "ms")
        time.sleep(0.1)
except KeyboardInterrupt:
    GPIO.cleanup()
