#!/usr/bin/env python3
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from cv_bridge import CvBridge
from custom_msgs.msg import InferenceResult, Yolov8Inference

from ament_index_python.packages import get_package_share_directory
import os
import math
import yaml

import rclpy.time
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from tf_transformations import euler_from_quaternion, quaternion_from_euler


class YoloObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')

        # --------------------------------------------------
        # ROS 2 parameters (only the minimum needed)
        # --------------------------------------------------
        self.declare_parameter('modelYolo', 'yolov8n_custom.pt')
        self.declare_parameter('topic', '/image_raw')
        self.declare_parameter('front_distance', 1.0)
        self.declare_parameter('signs_file', '')

        model_file = self.get_parameter('modelYolo').value
        self.image_topic = self.get_parameter('topic').value
        self.front_distance = float(self.get_parameter('front_distance').value)
        signs_file = self.get_parameter('signs_file').value

        if not signs_file:
            raise ValueError("Parameter 'signs_file' is empty")

        # --------------------------------------------------
        # Fixed internal constants
        # --------------------------------------------------
        self.sign_frame = 'map'
        self.base_frame = 'base_link'

        self.hold_stop_s = 3.0
        self.hold_prohibido_s = 5.0
        self.hold_ceda_s = 2.0
        self.cooldown_repeat_s = 5.0

        self.wp_forward_m = 0.8
        self.wp_lateral_m = 0.65

        # --------------------------------------------------
        # Load sign positions from external YAML file
        # --------------------------------------------------
        self.sign_positions = self.load_sign_positions(signs_file)

        # --------------------------------------------------
        # TF
        # --------------------------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --------------------------------------------------
        # YOLO model path
        # --------------------------------------------------
        package_path = get_package_share_directory('my_robot_ai_identification')
        self.model_path = os.path.join(package_path, 'models', model_file)

        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"YOLO model not found: {self.model_path}")

        self.get_logger().info(f"Loaded YOLO model: {self.model_path}")
        self.get_logger().info(f"Image topic: {self.image_topic}")
        self.get_logger().info(f"front_distance: {self.front_distance} m")
        self.get_logger().info(f"signs_file: {signs_file}")
        self.get_logger().info(f"sign_positions: {self.sign_positions}")

        # --------------------------------------------------
        # Setup
        # --------------------------------------------------
        self.bridge = CvBridge()
        self.model = YOLO(self.model_path)
        self.get_logger().info(f"YOLO classes: {self.model.names}")

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.camera_callback,
            qos_profile_sensor_data
        )

        self.yolov8_pub = self.create_publisher(Yolov8Inference, '/Yolov8_Inference', 1)
        self.img_pub = self.create_publisher(Image, '/inference_result', 1)

        self.waypoint_pub = self.create_publisher(PoseStamped, '/traffic_waypoint', 10)

        # Optional odom debug
        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = None

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Non-blocking reaction state
        self.hold_until = 0.0
        self.last_trigger_time = {}

    # --------------------------------------------------
    # Load sign positions from YAML
    # Expected format:
    # sign_positions:
    #   Izquierda: [1.1, 0.0]
    #   STOP: [2.3, -1.0]
    # --------------------------------------------------
    def load_sign_positions(self, filepath):
        if not os.path.isfile(filepath):
            raise FileNotFoundError(f"Signs YAML file not found: {filepath}")

        with open(filepath, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

        if data is None:
            raise ValueError(f"Signs YAML file is empty: {filepath}")

        if 'sign_positions' not in data:
            raise ValueError(f"Missing 'sign_positions' key in: {filepath}")

        raw_positions = data['sign_positions']
        if not isinstance(raw_positions, dict):
            raise ValueError("'sign_positions' must be a dictionary")

        sign_positions = {}
        for name, coords in raw_positions.items():
            if not isinstance(coords, (list, tuple)) or len(coords) != 2:
                raise ValueError(f'Sign "{name}" must be [x, y]. Got: {coords}')

            sign_positions[str(name)] = (float(coords[0]), float(coords[1]))

        return sign_positions

    # --------------------------------------------------
    # ODOM debug
    # --------------------------------------------------
    def odom_callback(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.odom_yaw = yaw

    # --------------------------------------------------
    # TF helpers
    # --------------------------------------------------
    def get_robot_xy_yaw_in_map(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.sign_frame,
                self.base_frame,
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"TF lookup failed ({self.sign_frame} -> {self.base_frame}): {e}"
            )
            return None

        x = tf.transform.translation.x
        y = tf.transform.translation.y

        q = tf.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        return (x, y, yaw)

    def get_sign_distance(self, sign_name: str):
        if sign_name not in self.sign_positions:
            return None

        robot_pose = self.get_robot_xy_yaw_in_map()
        if robot_pose is None:
            return None

        rx, ry, _ = robot_pose
        sx, sy = self.sign_positions[sign_name]

        return math.hypot(sx - rx, sy - ry)

    def should_react(self, sign_name: str) -> bool:
        d = self.get_sign_distance(sign_name)
        return (d is not None) and (d <= self.front_distance)

    # --------------------------------------------------
    # Waypoint generation near sign
    # --------------------------------------------------
    def make_waypoint_near_sign(self, sign_name: str, dx_forward: float, dy_left: float):
        if sign_name not in self.sign_positions:
            return None

        robot_pose = self.get_robot_xy_yaw_in_map()
        if robot_pose is None:
            return None

        _, _, yaw = robot_pose
        sx, sy = self.sign_positions[sign_name]

        wx = sx + dx_forward * math.cos(yaw) - dy_left * math.sin(yaw)
        wy = sy + dx_forward * math.sin(yaw) + dy_left * math.cos(yaw)

        pose = PoseStamped()
        pose.header.frame_id = self.sign_frame
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(wx)
        pose.pose.position.y = float(wy)
        pose.pose.position.z = 0.0

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)

        return pose

    # --------------------------------------------------
    # Camera callback
    # --------------------------------------------------
    def camera_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        #results = self.model(img)
        results = self.model(img, conf=0.10, verbose=False)

        yolov8_msg = Yolov8Inference()
        yolov8_msg.header.frame_id = 'inference'
        yolov8_msg.header.stamp = self.get_clock().now().to_msg()

        detected_signs = []

        for r in results:
            for box in r.boxes:
                inf = InferenceResult()

                b = box.xyxy[0].to('cpu').numpy()
                c = int(box.cls)
                class_name = self.model.names[c]

                detected_signs.append(class_name)

                inf.class_name = class_name
                inf.left, inf.top, inf.right, inf.bottom = map(int, b)
                inf.box_width = inf.right - inf.left
                inf.box_height = inf.bottom - inf.top
                inf.x = inf.left + inf.box_width / 2.0
                inf.y = inf.top + inf.box_height / 2.0

                yolov8_msg.yolov8_inference.append(inf)
        if detected_signs:
            self.get_logger().info(f"Detected signs: {detected_signs}")
            
        self.handle_signs(detected_signs)

        if results:
            annotated = results[0].plot()
            img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            self.img_pub.publish(img_msg)

        self.yolov8_pub.publish(yolov8_msg)

    # --------------------------------------------------
    # Debug log
    # --------------------------------------------------
    def log_pose_comparison(self, sign_name: str, dist_map: float):
        robot_pose_map = self.get_robot_xy_yaw_in_map()

        if robot_pose_map is None:
            if self.odom_x is not None and self.odom_y is not None and self.odom_yaw is not None:
                odom_raw = f"({self.odom_x:.2f},{self.odom_y:.2f},{self.odom_yaw:.2f})"
            else:
                odom_raw = "(n/a)"

            self.get_logger().info(
                f"[SIGN] {sign_name} | dist(map)={dist_map:.2f} m | "
                f"TF(map->{self.base_frame}) unavailable | odom_pose={odom_raw}"
            )
            return

        mx, my, myaw = robot_pose_map

        if self.odom_x is not None and self.odom_y is not None and self.odom_yaw is not None:
            odom_str = f"odom_pose=({self.odom_x:.2f},{self.odom_y:.2f},{self.odom_yaw:.2f})"
        else:
            odom_str = "odom_pose=(n/a)"

        self.get_logger().info(
            f"[SIGN] {sign_name} | dist(map)={dist_map:.2f} m | "
            f"map_pose(TF)=({mx:.2f},{my:.2f},{myaw:.2f}) | {odom_str}"
        )

    # --------------------------------------------------
    # Sign logic
    # --------------------------------------------------
    def handle_signs(self, detected_signs):
        now = self.get_clock().now().nanoseconds / 1e9

        if now < self.hold_until:
            return

        def can_trigger(sign_name: str) -> bool:
            last = self.last_trigger_time.get(sign_name, -1e9)
            return (now - last) >= self.cooldown_repeat_s

        if 'Prohibido' in detected_signs and self.should_react('Prohibido') and can_trigger('Prohibido'):
            dist = self.get_sign_distance('Prohibido') or 0.0
            self.log_pose_comparison('Prohibido', dist)
            self.get_logger().info(' -> hold + bypass waypoint')

            self.hold_until = now + self.hold_prohibido_s
            self.last_trigger_time['Prohibido'] = now

            wp = self.make_waypoint_near_sign(
                'Prohibido',
                dx_forward=self.wp_forward_m,
                dy_left=+self.wp_lateral_m
            )
            if wp is not None:
                self.waypoint_pub.publish(wp)

        elif 'STOP' in detected_signs and self.should_react('STOP') and can_trigger('STOP'):
            dist = self.get_sign_distance('STOP') or 0.0
            self.log_pose_comparison('STOP', dist)
            self.get_logger().info(' -> hold + forward waypoint')

            self.hold_until = now + self.hold_stop_s
            self.last_trigger_time['STOP'] = now

            wp = self.make_waypoint_near_sign(
                'STOP',
                dx_forward=self.wp_forward_m,
                dy_left=0.0
            )
            if wp is not None:
                self.waypoint_pub.publish(wp)

        elif 'Ceda' in detected_signs and self.should_react('Ceda') and can_trigger('Ceda'):
            dist = self.get_sign_distance('Ceda') or 0.0
            self.log_pose_comparison('Ceda', dist)
            self.get_logger().info(' -> short hold + forward waypoint')

            self.hold_until = now + self.hold_ceda_s
            self.last_trigger_time['Ceda'] = now

            wp = self.make_waypoint_near_sign(
                'Ceda',
                dx_forward=self.wp_forward_m,
                dy_left=0.0
            )
            if wp is not None:
                self.waypoint_pub.publish(wp)

        elif 'Derecha' in detected_signs and self.should_react('Derecha') and can_trigger('Derecha'):
            dist = self.get_sign_distance('Derecha') or 0.0
            self.log_pose_comparison('Derecha', dist)
            self.get_logger().info(' -> right waypoint')

            self.last_trigger_time['Derecha'] = now

            wp = self.make_waypoint_near_sign(
                'Derecha',
                dx_forward=self.wp_forward_m,
                dy_left=-self.wp_lateral_m
            )
            if wp is not None:
                self.waypoint_pub.publish(wp)

        elif 'Izquierda' in detected_signs and self.should_react('Izquierda') and can_trigger('Izquierda'):
            dist = self.get_sign_distance('Izquierda') or 0.0
            self.log_pose_comparison('Izquierda', dist)
            self.get_logger().info(' -> left waypoint')

            self.last_trigger_time['Izquierda'] = now

            wp = self.make_waypoint_near_sign(
                'Izquierda',
                dx_forward=self.wp_forward_m,
                dy_left=+self.wp_lateral_m
            )
            if wp is not None:
                self.waypoint_pub.publish(wp)


def main(args=None):
    rclpy.init(args=args)
    node = YoloObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()