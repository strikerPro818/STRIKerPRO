from dmBot import DMBOT
import math
import struct

robot = DMBOT(0x101)
robot.setup()
robot.motor_run()

angle_degrees = 90
angle_radians = math.radians(angle_degrees)
angle_bytes = struct.pack('<f', angle_radians)

speed = 1
speed_bytes = struct.pack('<f', speed)

data = angle_bytes + speed_bytes
print(data)

robot.targetSpeedMode(data)
