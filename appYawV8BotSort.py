from flask import Flask, render_template, Response, request
from flask_socketio import SocketIO, emit
from src.rmd_x8 import RMD_X8
from jtop import jtop
import numpy as np
import cv2
from turbojpeg import TurboJPEG, TJPF_BGR, TJFLAG_FASTDCT
import time
from ultralytics import YOLO
import Jetson.GPIO as GPIO
import busio
from board import SCL, SDA
import math
from adafruit_pca9685 import PCA9685
# import jsonify
import requests

# GPIO.setwarnings(False)
#
# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(33, GPIO.OUT)
# shooter = GPIO.PWM(33, 100)
# shooter.start(0)
#
# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(32, GPIO.OUT)
# feeder = GPIO.PWM(32, 100)
# feeder.start(0)



# Use the i2c-8 bus
i2c_bus = busio.I2C(SCL, SDA)

pca_output = PCA9685(i2c_bus)

# Set the PWM frequency to 60Hz
pca_output.frequency = 100

model = YOLO("/home/striker/.local/lib/python3.8/site-packages/yolov5/yolov8n.engine", task='detect')
# model = YOLO('/home/striker/Downloads/yolov8n-pose.pt',task='detect')
results = model.track(source=0, show=False, stream=True, tracker="botsort.yaml")
# results = model.track(source=0, show=False, stream=True, tracker="bytetrack.yaml")



turboJPG = TurboJPEG()
robot = RMD_X8(0x141)
robot.setup()
cam_size = 640
FRAME_W = 1920
FRAME_H = 1080
cam_pan = 90
angleVector = 8.5
desired_yaw_speed = 55
prev_z_angle = None

highlighted_id = None
selected_reid_feature = None
reid_feature = None
latest_result = None
app = Flask(__name__)
# app = Flask(__name__, static_folder='static', static_url_path='')

socketio = SocketIO(app)


def startShooter(speed):
    print('Shooter Current Speed:', speed)
    # shooter.ChangeDutyCycle(speed)
    pca_output.channels[0].duty_cycle = int(speed / 100 * 0xFFFF)


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



# def panAngle(angle):
#     target = int(angle * 100 * 6)
#     bb = target.to_bytes(4, 'little', signed=True)
#     print("Executed Angle:", angle)
#     robot.position_closed_loop_1(bb)

def panAngle(angle):
    global desired_yaw_speed
    target = int(angle * 100)
    angle_bytes = target.to_bytes(4, 'little', signed=True)
    speed_bytes = desired_yaw_speed.to_bytes(2, 'little', signed=True)
    data = list(speed_bytes) + list(angle_bytes)
    print("Executed Angle:", angle)
    robot.position_closed_loop_2(data)
    # time.sleep(0.1)


def getAngle():
    readAngle = robot.read_multi_turns_angle().data
    raw_angle = int.from_bytes(readAngle[4:8], byteorder='little', signed=True)
    actualAngle = raw_angle * 0.01
    print('Current Angle:',actualAngle)
    return actualAngle

def autoTrack(x1, x2):
    global cam_pan, FRAME_W, angleVector

    trackX = x1
    trackW = x2 - x1
    trackX = trackX + (trackW / 2)
    turn_x = float(trackX - (FRAME_W / 2))
    turn_x /= float(FRAME_W / 2)
    turn_x *= angleVector  # VFOV
    print(turn_x)
    cam_pan += -turn_x
    cam_pan = max(0, min(180, cam_pan))
    print('Move:', int(cam_pan), 'Auto Turn: ', (cam_pan - 90))
    panAngle((cam_pan - 90))
    # time.sleep(0.1)
    return cam_pan


def autoTrack_angle(x1, x2, current_angle):
    global cam_pan, FRAME_W, angleVector

    trackX = x1
    trackW = x2 - x1
    trackX = trackX + (trackW / 2)
    turn_x = float(trackX - (FRAME_W / 2))
    turn_x /= float(FRAME_W / 2)
    turn_x *= angleVector  # VFOV
    print(turn_x)

    cam_pan = current_angle + -turn_x
    cam_pan = max(0, min(180, cam_pan))
    print('Move:', int(cam_pan), 'Auto Turn: ', (cam_pan - 90))
    panAngle((cam_pan - 90))
    return cam_pan





# @app.route('/gyro_data', methods=['POST'])
# def gyro_data():
#     global z_rotation_angle
#     data = request.json  # Get the JSON data from the request
#     print(data)
#     z_rotation_angle = data.get('z')  # Extract the z-rotation angle from the JSON data
#     gyroTrack(z_rotation_angle, True)  # Call the gyroTrack function to pan the camera if gyroTrack_on is True
#
#     return 'OK'  # Return a response to the POST request
# @app.route('/gyro_data', methods=['POST'])
# def gyro_data():
#     data = request.get_json()
#     print('Received gyro data:', data)
#     # Do something with the received data
#     return '', 204

@app.route('/gyro_data', methods=['POST'])
def handle_gyro_data():
    global z_angle, gyroTrack_on
    if request.method == 'POST':
        gyro_data = request.get_json()
        z_angle = gyro_data.get('z')
        return '', 200
    else:
        gyroTrack_on = False
        return '', 405


def gyroTrack(z_rotation_angle, gyroTrack_on):
    global cam_pan, angleVector, prev_z_angle

    angle_change = 1  # Adjust this value to control the sensitivity of the angle change per degree of rotation
    angle_threshold = 1.0  # Only update the angle if the absolute change is greater than this threshold
    if z_rotation_angle == 0:
        panAngle(0)
    elif gyroTrack_on and (prev_z_angle is None or abs(z_rotation_angle - prev_z_angle) > angle_threshold):
        panAngle(z_rotation_angle)

        # prev_z_angle = z_rotation_angle
        # cam_pan += angle_change * z_rotation_angle
        # cam_pan = max(0, min(180, cam_pan))  # Ensure the angle is within the valid range
        # panAngle(int(cam_pan - 90))


@app.route('/')
def index():
    return render_template('index.html')


def gen_frames():
    global results, model, tracker, reid_model, outputs, cam_pan, device, latest_result

    for result in results:
        img = result.orig_img
        latest_result = result
        # print(img)
        if result.boxes is not None:

            boxes = result.boxes.xyxy.to("cpu").numpy()
            classes = result.boxes.cls.to("cpu").numpy()

            if gyroTrack_on:
                gyroTrack(z_angle, True)

            if result.boxes.id is not None:
                ids = result.boxes.id.numpy()
                for box, cls, obj_id in zip(boxes, classes, ids):
                    if cls == 0:  # "person" class index is 0
                        x1, y1, x2, y2 = map(int, box)
                        # print('x1:', x1, 'x2:', x2, 'y1:', y1, 'y2:', y2)
                        w, h = x2 - x1, y2 - y1
                        center_x, center_y = x1 + w // 2, y1 + h // 2
                        track_id = int(obj_id.item())

                        if highlighted_id is not None and track_id == highlighted_id:
                            color = (0, 0, 255)
                            text = f'Selected ID: {int(track_id)}'
                            # print(x1, x2, center_x)
                            cam_pan = autoTrack(x1, x2)
                            # time.sleep(0.05)

                            # cam_pan = autoTrack_angle(x1,x2,getAngle())

                        else:
                            color = (0, 255, 0)
                            text = f'Track ID: {int(track_id)}'

                        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                        cv2.putText(img, text, (int(x1), int(y1) - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                        # send_data_update(center_x, center_y, x1, y1, x2, y2,cam_pan - 90)

        encoded_image = turboJPG.encode(img, quality=80, pixel_format=TJPF_BGR, flags=TJFLAG_FASTDCT)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + encoded_image + b'\r\n\r\n')


@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), content_type='multipart/x-mixed-replace; boundary=frame')


@socketio.on('connect')
def on_connect():
    print("Client connected")


@socketio.on('click_event')
def handle_click_event(data):
    global highlighted_id, latest_result
    x, y = data['x'], data['y']
    # print('clicked x:',x,'clicked y:', y)

    found_id = None
    if latest_result is not None:
        boxes = latest_result.boxes.xyxy.to("cpu").numpy()
        classes = latest_result.boxes.cls.to("cpu").numpy()
        ids = latest_result.boxes.id.numpy()
        for box, cls, obj_id in zip(boxes, classes, ids):
            if cls == 0:  # "person" class index is 0
                x1, y1, x2, y2 = map(int, box)
                track_id = int(obj_id.item())

                if x1 <= x <= x2 and y1 <= y <= y2:
                    found_id = int(track_id)
                    break

    if highlighted_id is not None and highlighted_id == found_id:
        highlighted_id = None
    else:
        highlighted_id = found_id

    # Pass highlighted_id argument to gen_frames function
    socketio.emit('frame_update', {'data': 'update frame', 'highlighted_id': highlighted_id})


shooter_on, feeder_on, gyroTrack_on = False, False, False
z_rotation_angle = 0


@socketio.on('button_click')
def handle_button_click(data):
    global cam_pan, shooter, feeder, shooter_on, feeder_on, gyroTrack_on, z_rotation_angle

    angle_change = 10  # Adjust the value to control the angle change per click

    if data.get('direction') == 'left':
        cam_pan -= angle_change
        cam_pan = max(0, min(180, cam_pan))  # Ensure the angle is within the valid range
        panAngle(int(cam_pan - 90))
    elif data.get('direction') == 'right':
        cam_pan += angle_change
        cam_pan = max(0, min(180, cam_pan))  # Ensure the angle is within the valid range
        panAngle(int(cam_pan - 90))
    elif data.get('direction') == 'bottom':
        cam_pan = 90
        cam_pan = max(0, min(180, cam_pan))  # Ensure the angle is within the valid range
        panAngle(int(cam_pan - 90))
    elif data.get('direction') == 'shooter':
        if shooter_on:
            stopShooter()
            shooter_on = False
        else:
            startShooter(25)
            shooter_on = True
    elif data.get('direction') == 'feeder':
        if feeder_on:
            stopFeeder()
            feeder_on = False
        else:
            startFeeder(100)
            feeder_on = True


    # elif data.get('direction') == 'gyroTrack':
    #     if gyroTrack_on:
    #         print('disable gyro',z_angle)
    #         panAngle(0)
    #         gyroTrack_on = False
    #     else:
    #         print('turn gyro',z_angle)
    #         rollingFactor = 0.6
    #         panAngle(z_angle*rollingFactor)
    #         gyroTrack_on = True

    elif data.get('direction') == 'gyroTrack':
        gyroTrack_on = not gyroTrack_on  # Toggle the gyroTrack_on value
        gyroTrack(z_rotation_angle, gyroTrack_on)


last_sent = 0
update_interval = 1  # Send data every 0.1 seconds


def send_data_update(center_x, center_y, x1, y1, x2, y2, angleInput):
    global last_sent, update_interval

    current_time = time.time()
    if current_time - last_sent < update_interval:
        return

    last_sent = current_time
    x = center_x
    y = center_y
    w = x2 - x1
    h = y2 - y1
    angle = angleInput

    with jtop() as jetson:
        temperature = jetson.temperature

    data = {
        'temperature': temperature['CPU'],
        'motorAngle': angle,
        'x': x,
        'y': y,
        'w': w,
        'h': h
    }
    socketio.emit('data_update', data)


# if __name__ == '__main__':
#     socketio.run(app, host='192.168.31.177', port=9090, allow_unsafe_werkzeug=True)

if __name__ == '__main__':
    print('here')
    # socketio.run(app, host='192.168.31.177', port=9090, allow_unsafe_werkzeug=True)
    # socketio.run(app, host='172.20.10.2', port=9090, allow_unsafe_werkzeug=True)
    socketio.run(app, host='0.0.0.0', port=9090, allow_unsafe_werkzeug=True)
    # socketio.run(app, host='192.168.12.1', port=6061, allow_unsafe_werkzeug=True)



