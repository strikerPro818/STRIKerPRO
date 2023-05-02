from ultralytics import YOLO
from flask import Flask, render_template, Response, request
from flask_socketio import SocketIO
import cv2
from turbojpeg import TurboJPEG, TJPF_BGR, TJFLAG_FASTDCT
import requests
from threading import Thread

FRAME_W = 1920
FRAME_H = 1080
angleVector = 2
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

def send_tracking_data(angle):
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
    send_tracking_data((cam_pan - 90))
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