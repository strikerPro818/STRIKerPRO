from src.rmd_x8 import RMD_X8
import time
import os
# Setup a new RMD_X8 motor with its identifier.
# sudo ip link set can0 up type can bitrate 1000000
# sudo ifconfig can0 txqueuelen 65536

# os.system('sudo modprobe can')
# os.system('sudo modprobe can-raw')
# os.system('sudo modprobe mttcan')
#
# os.system('sudo ip link set can0 up type can bitrate 500000')
# # os.system('sudo ifconfig can0 txqueuelen 65536')




robot = RMD_X8(0x141)
robot.setup()
# robot.motor_run()

angle = 0
target = int(angle * 100)
desired_yaw_speed = 60
angle_bytes = target.to_bytes(4, 'little', signed=True)
speed_bytes = desired_yaw_speed.to_bytes(2, 'little', signed=True)
# data = list(speed_bytes) + list(angle_bytes)
data = list(speed_bytes) + list(angle_bytes)

print("Executed Angle:", angle)
robot.position_closed_loop_2(data)

readAngle = robot.read_multi_turns_angle().data
raw_angle = int.from_bytes(readAngle[4:8], byteorder='little', signed=True)

print(raw_angle * 0.01)