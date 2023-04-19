import time

import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685

# Use the i2c-8 bus
i2c_bus = busio.I2C(SCL, SDA)

pca_output = PCA9685(i2c_bus)

# Set the PWM frequency to 60Hz
pca_output.frequency = 60


# pca_output.channels[3].duty_cycle = int(0 / 100 * 0xFFFF)
# while True:
pca_output.channels[14].duty_cycle = int(0 / 100 * 0xFFFF)
pca_output.channels[15].duty_cycle = int(0 / 100 * 0xFFFF)

