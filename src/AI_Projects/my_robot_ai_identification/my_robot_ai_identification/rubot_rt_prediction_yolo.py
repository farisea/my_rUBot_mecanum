#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from custom_msgs.msg import InferenceResult, Yolov8Inference

import cv2
import time

bridge = CvBridge()


class YoloObjectDetection(Node):

    def __init__(self) -> None:
        super().__init__('object_detection')

        # Load YOLO model
        self.model = YOLO('/home/biorob/Desktop/my_rUBot_mecanum/src/AI_Projects/my_robot_ai_identification/models/yolov8n_custom.pt')

        self.yolov8_inference = Yolov8Inference()

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.camera_callback,
            10
        )

        # Publishers
        self.yolov8_pub = self.create_publisher(
            Yolov8Inference,
            "/Yolov8_Inference",
            1
        )

        self.img_pub = self.create_publisher(
            Image,
            "/inference_result",
            1
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

    # --------------------------------------------------
    # CALLBACK
    # --------------------------------------------------
    def camera_callback(self, msg: Image) -> None:

        img = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Resize to match training conditions
        img_resized = cv2.resize(img, (640, 640))

        # IMPORTANT: Only best detection
        results = self.model(
            img_resized,
            conf=0.4,        # confidence threshold
            max_det=1,       # only BEST detection
            verbose=False
        )

        # Header
        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        detected_signs = []

        # --------------------------------------------------
        # PROCESS RESULTS
        # --------------------------------------------------
        for r in results:
            boxes = r.boxes

            for box in boxes:

                self.inf_result = InferenceResult()

                b = box.xyxy[0].cpu().numpy()
                c = int(box.cls[0])
                conf = float(box.conf[0])

                class_name = self.model.names[c]
                detected_signs.append(class_name)

                # Debug log
                self.get_logger().info(
                    f"Detected: {class_name} | conf={conf:.2f}"
                )

                # Fill message
                self.inf_result.class_name = class_name

                self.inf_result.left = int(b[0])
                self.inf_result.top = int(b[1])
                self.inf_result.right = int(b[2])
                self.inf_result.bottom = int(b[3])

                self.inf_result.box_width = self.inf_result.right - self.inf_result.left
                self.inf_result.box_height = self.inf_result.bottom - self.inf_result.top

                self.inf_result.x = self.inf_result.left + self.inf_result.box_width / 2.0
                self.inf_result.y = self.inf_result.top + self.inf_result.box_height / 2.0

                self.yolov8_inference.yolov8_inference.append(self.inf_result)

        # --------------------------------------------------
        # (OPTIONAL) BEHAVIOUR LOGIC
        # --------------------------------------------------
        """
        if "STOP" in detected_signs:
            self.get_logger().info("STOP detected")
            self.stop_robot()
            time.sleep(3)
        """

        # --------------------------------------------------
        # PUBLISH RESULTS
        # --------------------------------------------------
        annotated_frame = results[0].plot()

        img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")

        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)

        # Clear for next frame
        self.yolov8_inference.yolov8_inference.clear()

    # --------------------------------------------------
    # AUX FUNCTIONS
    # --------------------------------------------------
    def stop_robot(self):
        self.publish_velocity(0.0, 0.0)

    def publish_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)


# --------------------------------------------------
# MAIN
# --------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)

    node = YoloObjectDetection()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()