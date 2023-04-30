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
import threading
from threading import Event

frame_event = Event()

# GPIO.setwarnings(False)
# GPIO.setmode(GPIO.BOARD)
# shooterDetect = GPIO.setup(40, GPIO.IN)


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
pca_output.frequency = 1000

# model = YOLO("/home/striker/Downloads/yolov8n-pose.engine", task='pose')
# model = YOLO('/home/striker/Downloads/JetsonBackup/yolov8n-pose-half16.engine', task='pose')  # load an official model
model = YOLO('/home/striker/Downloads/models/yolov8n-pose-int8.engine', task='pose')  # load an official model



results = model.track(source=0, show=False, stream=True, tracker="botsort.yaml")
# model = YOLO('/home/striker/Downloads/yolov8n-pose.pt',task='detect')
# results = model.track(source=0, show=False, stream=True, tracker="bytetrack.yaml")



turboJPG = TurboJPEG()
robot = RMD_X8(0x141)
robot.setup()
cam_size = 640
FRAME_W = 1920
FRAME_H = 1080
cam_pan = 90
angleVector = 8.5
desired_yaw_speed = 59
prev_z_angle = None

highlighted_id = None
selected_reid_feature = None
reid_feature = None
latest_result = None
latest_frame = None
frame_lock = threading.Lock()
app = Flask(__name__)
# app = Flask(__name__, static_folder='static', static_url_path='')

socketio = SocketIO(app)
lock = threading.Lock()



def startShooter(speed):
    print('Shooter Current Speed:', speed)
    # shooter.ChangeDutyCycle(speed)
    pca_output.channels[0].duty_cycle = int(speed/ 2 / 100 * 0xFFFF)


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



def detect_arm_raise(result):
    if result.keypoints is not None and len(result.keypoints) > 0:
        keypoints = result.keypoints[0][:17]  # Get keypoints for the first person detected
        # Extract the coordinates for the left and right shoulders and wrists
        left_shoulder_x, left_shoulder_y = int(keypoints[5][0]), int(keypoints[5][1])
        right_shoulder_x, right_shoulder_y = int(keypoints[6][0]), int(keypoints[6][1])
        left_wrist_x, left_wrist_y = int(keypoints[7][0]), int(keypoints[7][1])
        right_wrist_x, right_wrist_y = int(keypoints[8][0]), int(keypoints[8][1])
        # Check if either the left or right arm is raised above the head
        if (left_wrist_y < left_shoulder_y) or (right_wrist_y < right_shoulder_y):
            # Send a message to the client indicating that the arm is raised
            socketio.emit('arm_raised', True)
            print('arm_raised')
            pca_output.channels[1].duty_cycle = int(100  / 100 * 0xFFFF)

            return True
        else:
            pca_output.channels[1].duty_cycle = int(0 / 100 * 0xFFFF)

            return False

def extend_arm_both_side(result):
    if result.keypoints is not None and len(result.keypoints) > 0:
        keypoints = result.keypoints[0][:17]  # Get keypoints for the first person detected
        # Extract the coordinates for the left and right shoulders and wrists
        left_shoulder_x, left_shoulder_y = int(keypoints[5][0]), int(keypoints[5][1])
        right_shoulder_x, right_shoulder_y = int(keypoints[6][0]), int(keypoints[6][1])
        left_wrist_x, left_wrist_y = int(keypoints[7][0]), int(keypoints[7][1])
        right_wrist_x, right_wrist_y = int(keypoints[8][0]), int(keypoints[8][1])
        # Calculate the angle between the left and right arms
        angle = np.arctan2(right_wrist_y - right_shoulder_y, right_wrist_x - right_shoulder_x) - np.arctan2(left_wrist_y - left_shoulder_y, left_wrist_x - left_shoulder_x)
        angle_degrees = np.degrees(angle)
        # Check if the angle is within the tolerance range
        if abs(angle_degrees - 180) <= 10:
            # Send a message to the client indicating that the arms are extended
            socketio.emit('arms_extended', True)
            print('arms_extended')
            return True
        else:
            False





def is_facing_camera(result, tolerance=0.8):
    if result.keypoints is not None and len(result.keypoints) > 0:
        keypoints = result.keypoints[0][:17]  # Get keypoints for the first person detected

        # Extract the coordinates for the left eye, right eye, left ear, right ear, and nose
        left_eye_x, left_eye_y, left_eye_visibility = int(keypoints[1][0]), int(keypoints[1][1]), keypoints[1][2]
        right_eye_x, right_eye_y, right_eye_visibility = int(keypoints[2][0]), int(keypoints[2][1]), keypoints[2][2]
        left_ear_x, left_ear_y, left_ear_visibility = int(keypoints[3][0]), int(keypoints[3][1]), keypoints[3][2]
        right_ear_x, right_ear_y, right_ear_visibility = int(keypoints[4][0]), int(keypoints[4][1]), keypoints[4][2]
        nose_x, nose_y, nose_visibility = int(keypoints[0][0]), int(keypoints[0][1]), keypoints[0][2]

        if all([left_eye_visibility > 0, right_eye_visibility > 0, left_ear_visibility > 0, right_ear_visibility > 0, nose_visibility > 0]):
            eye_distance = np.sqrt((left_eye_x - right_eye_x) ** 2 + (left_eye_y - right_eye_y) ** 2)
            left_ear_nose_distance = np.sqrt((left_ear_x - nose_x) ** 2 + (left_ear_y - nose_y) ** 2)
            right_ear_nose_distance = np.sqrt((right_ear_x - nose_x) ** 2 + (right_ear_y - nose_y) ** 2)

            if abs(left_ear_nose_distance / eye_distance - right_ear_nose_distance / eye_distance) <= tolerance:
                print('Detect facing')
                return True
            else:
                print('Not detect facing')
                return False
        else:
            print('Not detect facing')
            return False
    else:
        print('Not detect facing')
        return False

def detect_ready_position(result):
    if result.keypoints is not None and len(result.keypoints) > 0:
        keypoints = result.keypoints[0][:17]  # Get keypoints for the first person detected

        # Extract coordinates for left and right shoulders, wrists, elbows, and knees
        left_shoulder_x, left_shoulder_y = int(keypoints[5][0]), int(keypoints[5][1])
        right_shoulder_x, right_shoulder_y = int(keypoints[6][0]), int(keypoints[6][1])
        left_wrist_x, left_wrist_y = int(keypoints[7][0]), int(keypoints[7][1])
        right_wrist_x, right_wrist_y = int(keypoints[8][0]), int(keypoints[8][1])
        left_elbow_x, left_elbow_y = int(keypoints[9][0]), int(keypoints[9][1])
        right_elbow_x, right_elbow_y = int(keypoints[10][0]), int(keypoints[10][1])
        left_knee_x, left_knee_y = int(keypoints[13][0]), int(keypoints[13][1])
        right_knee_x, right_knee_y = int(keypoints[14][0]), int(keypoints[14][1])

        # Check if both arms are stretched out and slightly bent
        arms_stretched = (left_wrist_y > left_elbow_y) and (right_wrist_y > right_elbow_y)
        elbows_bent = (abs(left_wrist_y - left_elbow_y) > 15) and (abs(right_wrist_y - right_elbow_y) > 15)

        # Check if knees are bent
        knees_bent = (abs(left_knee_y - right_knee_y) < 30) and (abs(left_knee_y - left_shoulder_y) > 30)

        # Check if the player is in the ready position
        if arms_stretched and elbows_bent and knees_bent:
            # The player is in the ready position
            print('Player ready')
            pca_output.channels[1].duty_cycle = int(100 / 100 * 0xFFFF)

            # Perform additional actions here
        else:
            # The player is not in the ready position
            print('Player not ready')
            pca_output.channels[1].duty_cycle = int(0 / 100 * 0xFFFF)

            # Perform additional actions here

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
    global results, model, tracker, reid_model, outputs, cam_pan, device, latest_result, latest_frame, frame_lock, frame_event
    # with lock:
    for result in results:
        img = result.plot(conf=False,
            line_width=None,
            font_size=None,
            font='Arial.ttf',
            pil=False,
            img=None,
            img_gpu=True,
            kpt_line=True,
            labels=False,
            boxes=False,
            masks=False,
            probs=False)

        print(result.speed)

        latest_result = result
        # print(img)
        if result.boxes is not None:

            if gyroTrack_on:
                gyroTrack(z_angle, True)

            boxes = result.boxes.xyxy.to("cpu").numpy()

            if result.boxes.id is not None:
                ids = result.boxes.id.numpy()
                for box, obj_id in zip(boxes, ids):
                    x1, y1, x2, y2 = map(int, box)
                    track_id = int(obj_id.item())

                    # if (highlighted_id is not None and track_id == highlighted_id) or is_facing_camera(result):
                    if (highlighted_id is not None and track_id == highlighted_id):


                        color = (0, 0, 255)
                        text = f'Selected ID: {int(track_id)}'
                        cam_pan = autoTrack(x1, x2)
                        # detect_arm_raise(result)
                        detect_ready_position(result)




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
            startShooter(42)
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
    # print('here')
    # socketio.run(app, host='192.168.31.177', port=9090, allow_unsafe_werkzeug=True)
    # socketio.run(app, host='172.20.10.2', port=9090, allow_unsafe_werkzeug=True)
    socketio.run(app, host='0.0.0.0', port=8080, allow_unsafe_werkzeug=True)
    # socketio.run(app, host='192.168.12.1', port=6061, allow_unsafe_werkzeug=True)



