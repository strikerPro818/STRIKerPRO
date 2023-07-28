from ultralytics import YOLO


model = YOLO('yolov8n-pose.mlmodel', task='pose')
results = model.track(source=0, show=False, stream=True, tracker="botsort.yaml")
# print('here')

for result in results:
    print(result.speed)


# # Load a model
# model = YOLO('/home/striker/Downloads/yolov8n-pose.pt')  # load an official model
# # model = YOLO('path/to/best.pt')  # load a custom trained
#
# # Export the model
# model.export(format='engine',device=0)