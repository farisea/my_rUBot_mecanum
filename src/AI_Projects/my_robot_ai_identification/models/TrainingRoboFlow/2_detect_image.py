from ultralytics import YOLO

# Load trained model
#model = YOLO("runs/detect/train/weights/best.pt")
model = YOLO("yolov8n_custom_en.pt")
print(model.names)
# Run detection
#results = model("test/images/Right_2_jpg.rf.4c96df2f860cab0e0928670ffb58ce15.jpg")
results = model("Foto_2.jpg")

# Show result
results[0].show()