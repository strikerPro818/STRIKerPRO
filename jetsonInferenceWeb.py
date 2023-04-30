from flask import Flask, Response, render_template
import jetson_inference
import jetson_utils
import cv2
import argparse

app = Flask(__name__)

# Parse command line arguments
parser = argparse.ArgumentParser(description="Track humans using PedNet and Jetson Inference")
parser.add_argument("--threshold", type=float, default=0.5, help="Detection confidence threshold")
args = parser.parse_args()

# Load PedNet model
net = jetson_inference.poseNet("resnet18-body", threshold=0.4)

# Initialize video input
input_stream = jetson_utils.videoSource("/dev/video0")

def generate_frames():
    while True:
        # Capture frame
        frame = input_stream.Capture()
        poses = net.Process(frame)
        # for pose in poses:
        #     print(pose.Keypoints)

        # Convert frame to BGR format
        frame = jetson_utils.cudaToNumpy(frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

        # # Convert frame to CUDA memory
        # cuda_mem = jetson_utils.cudaFromNumpy(frame)

        # Detect humans in the frame


        # Draw bounding boxes around detected humans
        # for detection in detections:
        #     left = int(detection.Left)
        #     top = int(detection.Top)
        #     right = int(detection.Right)
        #     bottom = int(detection.Bottom)
        #     jetson_utils.cudaDrawRect(cuda_mem, (left, top, right, bottom), (0, 255, 0, 255))



        # Encode the output frame
        ret, buffer = cv2.imencode('.jpg', frame)

        # Yield the output frame
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')




@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return render_template('videoFeed.html')

if __name__ == '__main__':
    app.run( host='192.168.31.177', debug=False)





