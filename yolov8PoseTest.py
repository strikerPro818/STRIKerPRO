from ultralytics import YOLO
# model = YOLO('/home/striker/Downloads/yolov8n-pose.engine')  # load an official model
model = YOLO('/home/striker/Downloads/JetsonBackup/yolov8n-pose.engine', task='pose')  # load an official model

# print('here')
results = model.predict(source=0, stream=True, show=True)
for result in results:
    print(result.speed)


# # Load a model
# model = YOLO('/home/striker/Downloads/yolov8n-pose.pt')  # load an official model
# # model = YOLO('path/to/best.pt')  # load a custom trained
#
# # Export the model
# model.export(format='engine',device=0)