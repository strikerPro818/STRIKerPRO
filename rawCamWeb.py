import cv2
from flask import Flask, render_template, Response
from ultralytics import YOLO
import time
app = Flask(__name__)




# model = YOLO("/home/striker/Downloads/yolov8n-pose.engine", task='pose')
# model = YOLO("/home/striker/.local/lib/python3.8/site-packages/yolov5/yolov8n.engine")
# model = YOLO('/home/striker/Downloads/JetsonBackup/yolov8n-pose.engine', task='pose')  # load an official model
# model = YOLO('/home/striker/yolov8n-pose.engine', task='pose')  # load an official model





# model = YOLO("/home/striker/.local/lib/python3.8/site-packages/yolov5/yolov5s.engine",task='detect')
model = YOLO('./local/yolov8n-pose.mlmodel',task='pose')


results = model.track(source=1, show=False, stream=True, tracker="botsort.yaml")

# results = model.predict(source=0, show=False, stream=True)

def gen_frames():
    # camera = cv2.VideoCapture(0,cv2.CAP_V4L2)
    # prev_time = time.time()
    # while True:
    #     success, frame = camera.read()
    #     if not success:
    #         break
    #     else:


    for result in results:
        # frame = result.orig_img
        frame = result.plot(conf=True,
            line_width=None,
            font_size=None,
            font='Arial.ttf',
            pil=False,
            img=None,
            img_gpu=True,
            kpt_line=True,
            labels=True,
            boxes=True,
            masks=True,
            probs=True)
        # frame = result.plot()
        print(result.speed)
        # print(result.keypoints)
        # current_time = time.time()
        # time_elapsed = current_time - prev_time
        # fps = 1 / time_elapsed
        # prev_time = current_time
        # print("FPS:", fps)
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    # camera.release()


@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return render_template('videoFeed.html')

if __name__ == '__main__':
    # app.run( host='192.168.31.177', debug=False)
    app.run( host='0.0.0.0',port=9091, debug=False)

