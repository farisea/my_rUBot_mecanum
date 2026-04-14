# train_model.py
from ultralytics import YOLO

model = YOLO("yolov8n.pt")

model.train(
    data="data.yaml",
    epochs=30,
    imgsz=640,
    device="cpu"
)

# After training, use:
# runs/detect/train/weights/best.pt
# and copy the best.pt file to a /model folder with a proper name: