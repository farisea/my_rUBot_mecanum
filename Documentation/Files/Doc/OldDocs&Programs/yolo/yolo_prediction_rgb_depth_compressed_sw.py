"""
This module contains a node that reads RGB and depth images,
runs YOLO inference, and publishes:
- an annotated prediction image
- an InferenceData message with class name and depth
"""

import os
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CompressedImage
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

from my_robot_depth_navigation.services.images_service import *
from custom_msgs.msg import InferenceData


YOLO_PREDICTIONS_TOPIC = "/yolo/predictions"

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)


class YoloPredictionNode(Node):

    def __init__(self) -> None:
        super().__init__("yolo_prediction_node")

        # ---------------- Model ----------------
        self.declare_parameter('model', 'yolov8n_custom.pt')
        model_name = self.get_parameter('model').get_parameter_value().string_value

        package_path = Path(get_package_share_directory('my_robot_ai_identification'))
        model_file = package_path / 'models' / model_name

        if not os.path.exists(model_file):
            self.get_logger().error(f"YOLO model not found: {model_file}")
            raise FileNotFoundError(model_file)

        self.yolo = YOLO(model_file)
        self.get_logger().info(f"Loaded YOLO model: {model_file}")
        self.get_logger().info(f"YOLO classes: {self.yolo.names}")

        # ---------------- Topics ----------------
        self.declare_parameter('color_topic', '/camera/image_raw/compressed')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')

        color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value

        self.get_logger().info(f"Subscribing color topic: {color_topic}")
        self.get_logger().info(f"Subscribing depth topic: {depth_topic}")

        self.create_subscription(
            CompressedImage,
            color_topic,
            self.color_image_callback,
            qos
        )

        self.create_subscription(
            Image,
            depth_topic,
            self.depth_image_callback,
            qos
        )

        # ---------------- Publishers ----------------
        self.prediction_publisher = self.create_publisher(
            Image,
            f'{YOLO_PREDICTIONS_TOPIC}/predictions_image',
            10
        )

        self.coordinates_publisher = self.create_publisher(
            InferenceData,
            f'{YOLO_PREDICTIONS_TOPIC}/predictions_data',
            10
        )

        # ---------------- Internal state ----------------
        self.color_image = None
        self.depth_image = None

        self.create_timer(0.1, self.run)

    def color_image_callback(self, msg: CompressedImage) -> None:
        self.color_image = msg

    def depth_image_callback(self, msg: Image) -> None:
        self.depth_image = msg

    def run(self) -> None:
        if self.color_image is None or self.depth_image is None:
            return

        color_msg = self.color_image
        depth_msg = self.depth_image

        # Clear buffered messages
        self.color_image = None
        self.depth_image = None

        color_image = compressed_to_cv(color_msg)
        depth_image = ros_to_cv(depth_msg, encoding='32FC1')

        if color_image is None or depth_image is None:
            self.get_logger().warn("Invalid RGB or depth frame after conversion")
            return

        predictions = self.get_predictions(color_image)

        if not predictions or len(predictions[0].boxes) == 0:
            return

        self.publish_detections(color_image.copy(), depth_image.copy(), predictions[0])

    def get_predictions(self, img):
        return self.yolo(img)

    def publish_detections(self, color_image, depth_image, prediction) -> None:
        signal_prediction = prediction.boxes[0]
        signal_xyxy = signal_prediction.xyxy[0].to('cpu').detach().numpy().copy()

        x1, y1, x2, y2 = map(int, signal_xyxy)
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        # Clamp centroid to image bounds
        h, w = depth_image.shape[:2]
        cx = max(0, min(cx, w - 1))
        cy = max(0, min(cy, h - 1))

        inference_data = InferenceData()
        inference_data.class_name = self.yolo.names[int(signal_prediction.cls)]

        confidence = float(signal_prediction.conf) * 100.0
        label = f"{inference_data.class_name}: {confidence:.2f}%"

        cv2.putText(
            color_image,
            label,
            (x1, max(0, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2
        )

        cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(color_image, (cx, cy), 5, (0, 255, 0), -1)

        depth_value = float(depth_image[cy, cx])
        inference_data.depth = depth_value

        prediction_image = cv_to_ros(color_image)
        self.prediction_publisher.publish(prediction_image)
        self.coordinates_publisher.publish(inference_data)


def main() -> None:
    rclpy.init()
    node = YoloPredictionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()