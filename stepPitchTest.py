import time

from stepPitch import STEPITCH
import math
import struct

robot = STEPITCH(0x001)
robot.setup()
# robot.motor_run()

# robot.read_step_position()

robot.step_on()
# robot.step_calibreate()
# # robot.step_calibreate()
# robot.step_off()
robot.step_zero()

clockwise = 1
counterclockwise = 0
# direction = clockwise


angle_degrees = 0
if angle_degrees > 0:
    direction = clockwise
else:
    direction = counterclockwise
microsteps = int((abs(angle_degrees) / 1.8) * 16)  # Assuming 1.8 degrees per step and 16 microsteps per step

angle_bytes = microsteps.to_bytes(3, byteorder='big', signed=True)
speed = 0x04  # Example speed value, use the appropriate value for your motor (1 to 4)
direction_bytes = (direction << 4) | speed  # Assuming direction uses the first bit (bit 4), and speed occupies the next 3 bits (bits 0-3)
direction_bytes = direction_bytes.to_bytes(1, byteorder='big')
# print(angle_bytes)
# robot.step_on()
data = direction_bytes + angle_bytes
print(data)
robot.step_position(data)


# robot.step_off()
# robot.step_position(angle_bytes)
# robot.step_position(angle_bytes)
# robot.step_test()
# robot.step_off()

# robot.step_test()
# angle_degrees = 90
# angle_radians = math.radians(angle_degrees)
# angle_bytes = struct.pack('<f', angle_radians)
#
# speed = 1
# speed_bytes = struct.pack('<f', speed)
#
# data = angle_bytes + speed_bytes
# print(data)
#
# robot.targetSpeedMode(data)
print('reading angle...')
time.sleep(1)


readAngle = robot.read_step_position().data


raw_angle = int.from_bytes(readAngle[1:5], byteorder='big', signed=True)
angle_degrees = (raw_angle * 360) / 65536
print(angle_degrees)
