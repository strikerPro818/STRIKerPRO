import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from flask import Flask, render_template
from flask_socketio import SocketIO
from ultralytics import YOLO
import cv2
import base64
from turbojpeg import TurboJPEG
import numpy as np

i2c_bus = busio.I2C(SCL, SDA)
pca_output = PCA9685(i2c_bus)
pca_output.frequency = 1000


app = Flask(__name__)
socketio = SocketIO(app)
# model = YOLO("/home/striker/Downloads/yolov8n-pose.engine", task='pose')
model = YOLO('/home/striker/Downloads/JetsonBackup/yolov8n-pose.engine', task='pose')  # load an official model

# model = YOLO('/home/striker/Downloads/yolov8n-pose.engine', task='pose')  # load an official model

results = model.track(source=0, show=False, stream=True, tracker="botsort.yaml")

jpeg = TurboJPEG()

@app.route('/')
def index():
    return render_template('video.html')

@socketio.on('connect')
def test_connect():
    print('Client connected')

@socketio.on('disconnect')
def test_disconnect():
    print('Client disconnected')


def extend_arm(keypoints):
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
            pca_output.channels[0].duty_cycle = int(30 / 2 / 100 * 0xFFFF)
        else:
            pass
            pca_output.channels[0].duty_cycle = int(0 / 2 / 100 * 0xFFFF)


def is_facing_camera(result, tolerance=0.3):
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

@socketio.on('start_stream')
def start_stream():

    # Loop through the results and send each frame to the client
    for result in results:
        img = result.plot(conf=True,
            line_width=None,
            font_size=None,
            font='Arial.ttf',
            pil=False,
            img=None,
            img_gpu=True,
            kpt_line=True,
            labels=False,
            boxes=False,
            masks=True,
            probs=True)

        if result.boxes:
            for box in result.boxes.xyxy:
                x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
                cv2.rectangle(img, (x1, y1), (x2, y2), color=(255, 0, 0), thickness=2)

        if result.keypoints is not None:
            detect_ready_position(result)

        jpeg_data = jpeg.encode(img)
        jpeg_bytes = base64.b64encode(jpeg_data).decode('utf-8')
        socketio.emit('frame', jpeg_bytes)

if __name__ == '__main__':
    # socketio.run(app, host='192.168.31.177', port=9090, debug=False, allow_unsafe_werkzeug=True)
    socketio.run(app, host='0.0.0.0', port=9090, debug=False, allow_unsafe_werkzeug=True)

