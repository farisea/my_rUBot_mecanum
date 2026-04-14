from ultralytics import YOLO
import cv2

# Load trained model
model = YOLO("yolov8n_custom_en.pt")

# Open USB camera
cap = cv2.VideoCapture(0)   # Try 0, if it fails try 1

# Optional: set resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("Error: Could not open USB camera.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    # Run inference on current frame
    results = model(frame)

    # Draw detections on frame
    annotated_frame = results[0].plot()

    # Show result
    cv2.imshow("YOLOv8 USB Camera Detection", annotated_frame)

    # Press q to quit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()