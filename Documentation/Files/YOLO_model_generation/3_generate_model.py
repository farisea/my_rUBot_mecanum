from ultralytics import YOLO


def main():

    print("Loading model...")

    model = YOLO("yolo11n-cls.pt")

    print("Starting training...")

    model.train(
        data="traffic_sign_dataset",   # Folder with train/val class subfolders
        epochs=50,                     # Maximum number of training epochs
        imgsz=640,                     # Input image size used during training
        batch=8,                       # Number of images processed per training step
        patience=10,                   # Stop if validation does not improve after 10 epochs

        optimizer="AdamW",             # Robust optimizer, usually good for small/medium datasets
        lr0=0.001,                     # Initial learning rate
        lrf=0.01,                      # Final learning rate fraction
        weight_decay=0.0005,           # Regularization to reduce overfitting

        device="cpu",                  # Use CPU. Change to 0 if using GPU

        degrees=10.0,                  # Small random rotations: useful for tilted signs
        translate=0.10,                # Small image shifts: useful if the sign is not centered
        scale=0.20,                    # Small zoom in/out variations
        shear=3.0,                     # Small geometric distortion
        perspective=0.0005,            # Very small perspective distortion

        fliplr=0.0,                    # Horizontal flip disabled: Left/Right signs would be confused
        flipud=0.0,                    # Vertical flip disabled: traffic signs are not upside down

        hsv_h=0.015,                   # Small hue variation: lighting/color changes
        hsv_s=0.40,                    # Saturation variation
        hsv_v=0.30,                    # Brightness variation

        auto_augment="randaugment",    # Automatic light augmentation for classification
        erasing=0.10,                  # Randomly hides small image regions to improve robustness

        dropout=0.10,                  # Reduces overfitting in classification training
        seed=42,                       # Makes training more reproducible

        workers=0,                     # Safer on Windows; use 2 or 4 on Linux
        project="runs/classify",       # Output folder
        name="train",                  # Experiment name
        exist_ok=True,                 # Overwrite/reuse folder if it already exists

        plots=True,                    # Save training plots
        save=True,                     # Save trained model
        verbose=True                   # Show training information
    )

    print("Training finished.")


if __name__ == "__main__":
    main()