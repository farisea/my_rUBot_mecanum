import cv2
import os
import time

# =========================
# CONFIGURATION PARAMETERS
# =========================

output_folder = "handshake"   # Folder name (also used as filename prefix)
capture_interval = 2          # Seconds between captures
camera_index = 0              # 0 = default webcam (/dev/video0)

# =========================
# CREATE OUTPUT DIRECTORY
# =========================

os.makedirs(output_folder, exist_ok=True)

# Extract folder name for filename prefix
folder_name = os.path.basename(os.path.normpath(output_folder))

# =========================
# OPEN CAMERA
# =========================

cap = cv2.VideoCapture(camera_index)

if not cap.isOpened():
    print("Error: Cannot open camera")
    exit()

print("Camera opened successfully")
print("Press Ctrl+C to stop capturing\n")

image_counter = 1

try:
    while True:
        ret, frame = cap.read()

        if not ret:
            print("Error: Cannot read frame")
            break

        # Create filename with 3-digit numbering
        filename = f"{folder_name}_{image_counter:03d}.jpg"
        filepath = os.path.join(output_folder, filename)

        # Save image
        cv2.imwrite(filepath, frame)
        print(f"Saved: {filepath}")

        image_counter += 1

        time.sleep(capture_interval)

except KeyboardInterrupt:
    print("\nCapture stopped by user")

# Release camera
cap.release()
print("Camera released")