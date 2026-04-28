import cv2
import time
import os

# ==============================
# PARAMETERS (EDIT THESE)
# ==============================

interval_seconds = 0.5      # X → time between photos
filename_prefix = "image" # Y → base filename
start_index = 1          # Z → starting number
camera_index = 0          # usually 0 for default webcam

# ==============================
# CAMERA INITIALIZATION
# ==============================

cap = cv2.VideoCapture(camera_index)

if not cap.isOpened():
    print("Error: Camera not accessible")
    exit()

print("Camera started successfully")
print("Saving images in:", os.getcwd())

counter = start_index

try:
    while True:
        ret, frame = cap.read()

        if not ret:
            print("Error capturing image")
            break

        filename = f"{filename_prefix}_{counter}.jpg"

        cv2.imwrite(filename, frame)
        print(f"Saved: {filename}")

        counter += 1
        time.sleep(interval_seconds)

except KeyboardInterrupt:
    print("\nCapture stopped by user")

cap.release()
print("Camera released")