from ultralytics import YOLO

# Load a model
model = YOLO('yolov8n-pose.pt')  # load an official model
# model = YOLO('path/to/best.pt')  # load a custom trained

# Export the model
model.export(format='engine', device=0, workspace=8)


