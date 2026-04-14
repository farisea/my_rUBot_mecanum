#!/usr/bin/env python3

import os
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')

        # =========================
        # Parameters
        # =========================
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('output_folder', 'handshake')
        self.declare_parameter('capture_interval', 2.0)

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.output_folder = self.get_parameter('output_folder').get_parameter_value().string_value
        self.capture_interval = self.get_parameter('capture_interval').get_parameter_value().double_value

        # =========================
        # Setup
        # =========================
        self.bridge = CvBridge()
        self.latest_frame = None
        self.last_save_time = 0.0

        # Create output folder if it does not exist
        Path(self.output_folder).mkdir(parents=True, exist_ok=True)

        # Use folder name as filename prefix
        self.folder_name = os.path.basename(os.path.normpath(self.output_folder))

        # Find the next available image number
        self.image_counter = self._get_next_image_number()

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        # Timer for periodic saving
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f'Subscribed to topic: {self.image_topic}')
        self.get_logger().info(f'Output folder: {self.output_folder}')
        self.get_logger().info(f'Capture interval: {self.capture_interval:.2f} s')
        self.get_logger().info(f'Starting image numbering at: {self.image_counter:03d}')

    def _get_next_image_number(self):
        """
        Scan the output folder and return the next available image number.
        """
        existing_files = list(Path(self.output_folder).glob(f'{self.folder_name}_*.jpg'))

        max_number = 0
        for file_path in existing_files:
            stem = file_path.stem  # e.g. handshake_003
            parts = stem.split('_')
            if len(parts) >= 2 and parts[-1].isdigit():
                number = int(parts[-1])
                if number > max_number:
                    max_number = number

        return max_number + 1

    def image_callback(self, msg):
        """
        Store the latest frame received from the ROS 2 image topic.
        """
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def timer_callback(self):
        """
        Save one image every capture_interval seconds.
        """
        if self.latest_frame is None:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9

        if current_time - self.last_save_time >= self.capture_interval:
            filename = f'{self.folder_name}_{self.image_counter:03d}.jpg'
            filepath = os.path.join(self.output_folder, filename)

            success = cv2.imwrite(filepath, self.latest_frame)
            if success:
                self.get_logger().info(f'Saved: {filepath}')
                self.image_counter += 1
                self.last_save_time = current_time
            else:
                self.get_logger().error(f'Failed to save image: {filepath}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Image capture stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()