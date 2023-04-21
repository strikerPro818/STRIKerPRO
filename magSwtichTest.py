import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Add pull-down resistor

while True:
    if GPIO.input(21) == False:
        print("Triggered")
    else:
        print("NO")
