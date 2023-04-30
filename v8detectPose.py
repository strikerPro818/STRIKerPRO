from flask import Flask, render_template
from flask_socketio import SocketIO
from ultralytics import YOLO
import cv2
import base64
from turbojpeg import TurboJPEG
import numpy as np

app = Flask(__name__)
socketio = SocketIO(app)
model = YOLO("/home/striker/Downloads/yolov8n-pose.engine", task='pose')
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

@socketio.on('start_stream')
def start_stream():
    results = model.track(source=0, show=False, stream=True, tracker="botsort.yaml")

    # Loop through the results and send each frame to the client
    for result in results:
        for box in result.pred:
            kp = box['kp']
            if kp is not None:
                # Check if arm is fully extended horizontally
                left_shoulder = kp[2]
                left_elbow = kp[3]
                left_wrist = kp[4]
                right_shoulder = kp[5]
                right_elbow = kp[6]
                right_wrist = kp[7]
                left_arm_length = np.linalg.norm(left_shoulder - left_wrist)
                right_arm_length = np.linalg.norm(right_shoulder - right_wrist)
                if left_arm_length > 1.5 * np.linalg.norm(left_shoulder - left_elbow) and \
                        right_arm_length > 1.5 * np.linalg.norm(right_shoulder - right_elbow):
                    # Arm is fully extended horizontally
                    print("Arm fully extended horizontally")

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

        if result.boxes.id is not None:
            ids = result.boxes.id.numpy()
            for obj_id in ids:
                track_id = int(obj_id.item())
                text = f'Track ID: {int(track_id)}'
                cv2.putText(img, text, (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        jpeg_data = jpeg.encode(img)
        jpeg_bytes = base64.b64encode(jpeg_data).decode('utf-8')
        socketio.emit('frame', jpeg_bytes)

if __name__ == '__main__':
    socketio.run(app, host='192.168.31.177', port=9090, debug=False, allow_unsafe_werkzeug=True)
