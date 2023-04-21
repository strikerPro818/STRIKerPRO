from ultralytics import YOLO
model = YOLO('/home/striker/Downloads/yolov8n-pose.engine')  # load an official model
# print('here')
results = model(source=0,stream=True, show=True)
for result in results:
    pass