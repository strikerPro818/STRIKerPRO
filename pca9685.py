import time

import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685

# Use the i2c-8 bus
i2c_bus = busio.I2C(SCL, SDA)

pca_output = PCA9685(i2c_bus)

# Set the PWM frequency to 60Hz
pca_output.frequency = 100

# Set the PWM to 30 for channel 0
# channel = 0
# speed = 0
# pca_output.channels[channel].duty_cycle = int(speed / 100 * 0xFFFF)

# pca_output.channels[0].duty_cycle = int(0 / 100 * 0xFFFF)
# pca_output.channels[1].duty_cycle = int(100 / 100 * 0xFFFF)
#
# time.sleep(4)

pca_output.channels[0].duty_cycle = int(0/2 / 100 * 0xFFFF)
pca_output.channels[1].duty_cycle = int(0 / 100 * 0xFFFF)
# pca_output.channels[3].duty_cycle = int(50 / 100 * 0xFFFF)
# pca_output.channels[3].duty_cycle = int(0 / 100 * 0xFFFF)
# time.sleep(1)
# pca_output.channels[3].duty_cycle = int(20 / 100 * 0xFFFF)


# pca_output.channels[1].duty_cycle = int(0 / 100 * 0xFFFF)

# 23 - 17.19