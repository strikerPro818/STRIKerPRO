# import Jetson.GPIO as GPIO
# import time
# GPIO.setmode(GPIO.BCM)
# GPIO.setwarnings(False)
# GPIO.setup(21, GPIO.OUT)
# GPIO.setup(20, GPIO.OUT)
#
# GPIO.output(20, GPIO.HIGH)
# GPIO.setup(21, GPIO.LOW)
#
# print('Leveling Down...')
# time.sleep(6)
#
# GPIO.output(20, GPIO.LOW)
# GPIO.setup(21, GPIO.HIGH)
# print()
# print('Leveling Up...')

import Jetson.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(40, GPIO.OUT)
GPIO.setup(38, GPIO.OUT)

GPIO.output(40, GPIO.HIGH)
GPIO.setup(38, GPIO.LOW)

print('Leveling Down...')
time.sleep(6)

GPIO.output(40, GPIO.LOW)
GPIO.setup(38, GPIO.HIGH)
print()
print('Leveling Up...')