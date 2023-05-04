from ultralytics import YOLO
from flask import Flask, render_template, Response, request
from flask_socketio import SocketIO
import cv2
from turbojpeg import TurboJPEG, TJPF_BGR, TJFLAG_FASTDCT
import requests
from threading import Thread
import numpy as np
FRAME_W = 1920
FRAME_H = 1080
angleVector = 3
cam_pan = 90

model = YOLO('/Users/epc_striker_pro/Downloads/yolov8n-pose.mlmodel',task='pose')
results = model.track(source=0, show=False, stream=True, tracker="botsort.yaml")
app = Flask(__name__)

highlighted_id = None
turboJPG = TurboJPEG()
socketio = SocketIO(app)


# def panAngle(inputAngle):
#     '''I wanna send the angle'''
#     print(inputAngle, "panAngle")
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
            # pca_output.channels[1].duty_cycle = int(100 / 100 * 0xFFFF)

            # Perform additional actions here
        else:
            # The player is not in the ready position
            print('Player not ready')
            # pca_output.channels[1].duty_cycle = int(0 / 100 * 0xFFFF)
def panAngle(angle):
    url = 'http://192.168.31.180:9090/tracking_data'
    payload = {'z': angle}
    headers = {'Content-Type': 'application/json'}

    response = requests.post(url, json=payload, headers=headers)

    if response.status_code == 200:
        print("Angle sent successfully")
    else:
        print("Error sending angle:", response.status_code)
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
    # panAngle((cam_pan - 90))
    # time.sleep(0.1)
    return cam_pan
def autoTrack_thread(x1, x2):
    global cam_pan

    cam_pan = autoTrack(x1, x2)
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

            # if gyroTrack_on:
            #     gyroTrack(z_angle, True)

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
                        auto_track_thread = Thread(target=autoTrack_thread, args=(x1, x2))
                        auto_track_thread.start()
                        detect_ready_position(result)
                        # cam_pan = autoTrack(x1, x2)
                        # detect_arm_raise(result)
                        # detect_ready_position(result)




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

@socketio.on('click_event')
def handle_click_event(data):
    global highlighted_id, latest_result
    x, y = data['x'], data['y']

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
            # stopShooter()
            shooter_on = False
        else:
            # startShooter(42)
            shooter_on = True
    elif data.get('direction') == 'feeder':
        if feeder_on:
            # stopFeeder()
            feeder_on = False
        else:
            # startFeeder(100)
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
        # gyroTrack(z_rotation_angle, gyroTrack_on)
@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return render_template('index.html')



# if __name__ == '__main__':
#     app.run(host='0.0.0.0', port='9090', debug=False)
if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=9090, allow_unsafe_werkzeug=True)