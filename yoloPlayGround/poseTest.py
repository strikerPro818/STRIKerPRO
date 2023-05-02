from ultralytics import YOLO
# model = YOLO('/home/striker/Downloads/yolov8n-pose.engine')  # load an official model
model = YOLO('/Users/epc_striker_pro/Downloads/yolov8n-pose.mlmodel',task='pose')

# print('here')
results = model.predict(source=0, stream=True, show=True)
for result in results:
    print(result.speed)