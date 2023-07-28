# from src.rmd_x8 import RMD_X8
import time
import os
from STEP57YAW import STEP57YAW
from STEP57PITCH import STEP57PITCH
from flask import Flask, request, jsonify
from flask_socketio import SocketIO
from threading import Thread

from adafruit_pca9685 import PCA9685
import busio
from board import SCL, SDA
from liftMultiTest import *

os.system('sudo ip link set can0 up type can bitrate 1000000')
os.system('sudo ifconfig can0 txqueuelen 65536')
# os.system('sudo ifconfig can0 txqueuelen 1000')


# robot = RMD_X8(0x141)
yawMotor = STEP57YAW(0x0100)
yawMotor.setup()


yawMotor.step_write_pid(30000, 120000, 1)

yawMotor.initiate_senseless_zero()


yawMotor.step_on()
# yawMotor.step_on()

yaw_gear_ratio = 10.5

pitchMotor = STEP57PITCH(0x0200)
pitch_gear_ratio = 23
pitchMotor.setup()
pitchMotor.read_initiation_parameters()
pitchMotor.step_on()
pitchMotor.step_write_pid(43000,200000,1)
pitchMotor.initiate_senseless_zero()


# pitchMotor.step_position(-40 * pitch_gear_ratio, 500)

#

# def level_down_multithread():
#     def level_down_thread():
#         level_down()
#
#     t = Thread(target=level_down_thread)
#     t.start()

#
# def level_up_multithread():
#     def level_up_thread():
#         level_up()
#
#     t = Thread(target=level_up_thread)
#     t.start()


def move_up_multithread():
    def move_up_thread():
        move_up()
        print('moving!!!!!')

    t = Thread(target=move_up_thread)
    t.start()


start_time = time.time()  # Start time
move_up_multithread()

# move_up()

while time.time() - start_time < 10:  # Run loop for 4 seconds
#     # print(time.time())
#     # Your code here


    yawMotor.step_position(-94 * yaw_gear_ratio,4000)
    time.sleep(0.01)  # 10 milliseconds delay

    pitchMotor.step_position(-40 * pitch_gear_ratio, 500)
    time.sleep(0.01)  # 10 milliseconds delay



# yawMotor.read_initiation_parameters()


# while time.time() - start_time < 10:  # Run loop for 4 seconds




# pitchMotor.step_write_pid(80000,476000,1)

# pitchMotor.step_write_pid(20000, 76000, 1)



app = Flask(__name__)
socketio = SocketIO(app)
desired_yaw_speed = 100

i2c_bus = busio.I2C(SCL, SDA)

pca_output = PCA9685(i2c_bus)
pca_output.frequency = 1000

@app.route('/tracking_data', methods=['POST'])
def handle_tracking_data():
    if request.method == 'POST':
        payload = request.get_json()

        # Check the payload and print it
        if 'z' in payload:
            angle = payload['z']
            print(f"Received angle: {angle}")
            panAngle(angle)
        else:
            print("Payload does not contain 'z' key")

        return jsonify({'status': 'success'}), 200
    else:
        return jsonify({'status': 'method not allowed'}), 405

# def panAngle(angle):
#     global desired_yaw_speed
#     target = int(angle * 100)
#     angle_bytes = target.to_bytes(4, 'little', signed=True)
#     speed_bytes = desired_yaw_speed.to_bytes(2, 'little', signed=True)
#     data = list(speed_bytes) + list(angle_bytes)
#     print("Executed Angle:", angle)
#     robot.position_closed_loop_2(data)
def panAngle(angle):
    global yaw_gear_ratio
    yawMotor.step_position(((angle-94) * yaw_gear_ratio), 4000)

def panAnglePITCH(angle):
    global pitch_gear_ratio
    print('Pitch Current Angle:', angle)
    pitchMotor.step_position((angle-45) * pitch_gear_ratio, 4000)
@app.route('/pitch/angle', methods=['POST'])
def handle_pitch_angle():
    # Retrieve the angle value from the request's JSON payload
    payload = request.get_json()
    angle = payload.get('angle', 30)
    print('here!', int(angle))

    # Call panAnglePITCH function with the retrieved angle
    panAnglePITCH(int(angle))

    # Return a success status
    return jsonify({'status': 'success'}), 200
@app.route('/shooter/start', methods=['POST'])
def handle_shooter_start():
    # Retrieve the speed value from the request's JSON payload
    payload = request.get_json()
    speed = payload.get('speed', 0)
    startShooter(int(speed))
    return jsonify({'status': 'success'}), 200


@app.route('/shooter/stop', methods=['POST'])
def handle_shooter_stop():
    stopShooter()
    return jsonify({'status': 'success'}), 200


@app.route('/feeder/start', methods=['POST'])
def handle_feeder_start():
    # Retrieve the speed value from the request's JSON payload
    payload = request.get_json()
    speed = payload.get('speed', 0)

    startFeeder(speed)
    return jsonify({'status': 'success'}), 200


@app.route('/feeder/stop', methods=['POST'])
def handle_feeder_stop():
    stopFeeder()
    return jsonify({'status': 'success'}), 200


def startShooter(speed):
    print('Shooter Current Speed:', speed)
    # shooter.ChangeDutyCycle(speed)
    pca_output.channels[0].duty_cycle = int(speed / 2 / 100 * 0xFFFF)


def stopShooter():
    print('Shooter Stopped')
    # shooter.ChangeDutyCycle(0)
    pca_output.channels[0].duty_cycle = int(0 / 100 * 0xFFFF)



def startFeeder(speed):
    print('Shooter Current Speed:', speed)
    # feeder.ChangeDutyCycle(speed)
    pca_output.channels[1].duty_cycle = int(speed / 100 * 0xFFFF)


def stopFeeder():
    print('Feeder Stopped')
    # feeder.ChangeDutyCycle(0)
    pca_output.channels[1].duty_cycle = int(0 / 100 * 0xFFFF)
# def level_down_multithread():
#     def level_down_thread():
#         level_down()
#
#     t = Thread(target=level_down_thread)
#     t.start()

if __name__ == '__main__':
    # move_up_multithread
    # panAnglePITCH(0)\
    # level_down_multithread()
    socketio.run(app, host='169.254.93.151', port=9090, allow_unsafe_werkzeug=True)
    # socketio.run(app, host='172.20.10.2', port=9090, allow_unsafe_werkzeug=True)

