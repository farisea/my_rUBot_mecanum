#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DepthToMono8(Node):
    def __init__(self):
        super().__init__("depth_to_mono8")
        self.declare_parameter("in_topic", "/camera/depth_image")
        self.declare_parameter("out_topic", "/camera/depth_image_mono8")
        self.declare_parameter("min_m", 0.2)
        self.declare_parameter("max_m", 3.0)

        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            self.get_parameter("in_topic").value,
            self.cb,
            10,
        )
        self.pub = self.create_publisher(
            Image,
            self.get_parameter("out_topic").value,
            10,
        )

    def cb(self, msg: Image):
        # Expect 32FC1
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().error(f"cv_bridge convert failed: {e}")
            return

        mn = float(self.get_parameter("min_m").value)
        mx = float(self.get_parameter("max_m").value)
        if mx <= mn:
            mx = mn + 1e-3

        # Replace NaNs/Infs with max range (black after inversion) or mn
        depth = np.nan_to_num(depth, nan=mx, posinf=mx, neginf=mn)

        # Clip and scale to 0..255
        depth_clipped = np.clip(depth, mn, mx)
        mono = (255.0 - (depth_clipped - mn) * (255.0 / (mx - mn))).astype(np.uint8)

        out = self.bridge.cv2_to_imgmsg(mono, encoding="mono8")
        out.header = msg.header
        self.pub.publish(out)


def main():
    rclpy.init()
    node = DepthToMono8()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()