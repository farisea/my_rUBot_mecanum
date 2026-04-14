#!/usr/bin/env python3

import os
import cv2

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data


class ImageSaverNode(Node):

    def __init__(self):
        super().__init__('image_saver_node')

        # Parameters
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('output_folder', 'handshake')
        self.declare_parameter('capture_interval', 2.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.output_folder = self.get_parameter('output_folder').value
        self.capture_interval = self.get_parameter('capture_interval').value

        os.makedirs(self.output_folder, exist_ok=True)

        self.folder_name = os.path.basename(self.output_folder)

        self.bridge = CvBridge()
        self.latest_frame = None
        self.image_counter = 1
        self.last_save_time = 0.0

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Image saver node started")

    def image_callback(self, msg):

        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Conversion error: {e}")

    def timer_callback(self):

        if self.latest_frame is None:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9

        if current_time - self.last_save_time >= self.capture_interval:

            filename = f"{self.folder_name}_{self.image_counter:03d}.jpg"
            filepath = os.path.join(self.output_folder, filename)

            cv2.imwrite(filepath, self.latest_frame)

            self.get_logger().info(f"Saved: {filepath}")

            self.image_counter += 1
            self.last_save_time = current_time


def main(args=None):

    rclpy.init(args=args)
    node = ImageSaverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()