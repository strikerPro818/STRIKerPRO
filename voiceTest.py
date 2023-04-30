import Jetson.GPIO as GPIO
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
import time

# Reset GPIO mode
try:
    GPIO.cleanup()
except RuntimeError:
    pass

# Use the i2c-8 bus
i2c_bus = busio.I2C(SCL, SDA)

pca_output = PCA9685(i2c_bus)

# Set the PWM frequency to 60Hz
pca_output.frequency = 100

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

while True:
    GPIO.setup(40, GPIO.IN)
    shooterDetect = GPIO.input(40)

    GPIO.setup(38, GPIO.IN)
    feederDetect = GPIO.input(38)

    if shooterDetect==True:
        pca_output.channels[0].duty_cycle = int(30 / 2 / 100 * 0xFFFF)
    else:
        pca_output.channels[0].duty_cycle = int(0 / 2 / 100 * 0xFFFF)

    if feederDetect==True:
        pca_output.channels[1].duty_cycle = int(100 / 2 / 100 * 0xFFFF)
    else:
        pca_output.channels[1].duty_cycle = int(0 / 2 / 100 * 0xFFFF)

    time.sleep(0.1)

GPIO.cleanup()
