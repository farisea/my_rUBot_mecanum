import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile,QoSReliabilityPolicy,QoSHistoryPolicy,QoSDurabilityPolicy


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        # Parameters
        self.declare_parameter('distance_limit', 0.5)    # desired distance to right wall
        self.declare_parameter('forward_speed', 0.20)    # linear speed
        self.declare_parameter('turn_speed', 0.40)       # angular speed
        self.declare_parameter('time_to_stop', 30.0)     # auto-stop
        self.declare_parameter('tolerance', 0.05)        # band around base_distance (RIGHT)

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.v_ang = float(self.get_parameter('turn_speed').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)
        self.tol = float(self.get_parameter('tolerance').value)

        # Last commanded twist (will be published periodically)
        self.cmd = Twist()

        # ROS 2 entities
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.laser_callback,
            scan_qos,
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timers
        self.info_timer = self.create_timer(1.0, self.log_info)
        self.stop_timer = self.create_timer(0.05, self.stop_watchdog)

        # Periodic cmd_vel publisher at 10 Hz (0.1 s)
        self.cmd_timer = self.create_timer(0.1, self.cmd_publish_timer_cb)

        self._state_action = "Idle"
        self._last_action_logged = None
        self._shutting_down = False

        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info(
            "WallFollower (RIGHT tol, BACK_RIGHT when closest) - differential drive."
        )

    #--------------------------------------------------------------------
    def stop_watchdog(self):
        """Stop the robot after time_to_stop seconds."""
        if self._shutting_down:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time_s >= self.time_to_stop:
            self.get_logger().info("Stopping due to timeout.")
            self.stop()

    #--------------------------------------------------------------------
    def stop(self):
        """Safe stop: set cmd to zero Twist, try to publish once, stop timers."""
        self._shutting_down = True

        # Set last command to zero
        self.cmd = Twist()

        # Try a final publish (publisher may still be valid even if shutdown started)
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            # Context/publisher may already be invalid -> ignore
            pass

        # Cancel timers safely
        for t in [self.info_timer, self.stop_timer, self.cmd_timer]:
            try:
                t.cancel()
            except Exception:
                pass

    #--------------------------------------------------------------------
    def cmd_publish_timer_cb(self):
        """Periodic publisher: send the latest cmd_vel at 10 Hz."""
        if self._shutting_down:
            return

        try:
            self.publisher.publish(self.cmd)

        except Exception:
            # If the context or publisher is invalid, ignore
            pass

    #--------------------------------------------------------------------
    def laser_callback(self, scan):
        """Compute control action from LIDAR and update self.cmd."""
        if self._shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        closest_distance = math.inf
        for i, distance in enumerate(scan.ranges):
            # Angle on robot
            angle_robot_deg =angle_min + i * angle_inc

            if distance < scan.range_min or distance > scan.range_max:
                continue

            # Filter valid readings within [-180°, 0°]
            if 0 < angle_robot_deg < 180.0:
                continue

            # Replace closest distance if this one is smaller
            if  distance < closest_distance:
                closest_distance, angle_closest_distance = distance, angle_robot_deg

        if closest_distance is math.inf:
            return

        # Guardar el último ángulo y distancia para mostrar en el timer_callback junto con la velocidad actual
        self._last_closest_distance = closest_distance
        self._last_closest_angle = angle_closest_distance

        twist = Twist()
        action = ""

        # No nos movemos en Y
        twist.linear.y = 0.0
        twist.linear.x = self.v_lin * 0.5  # default forward speed (can be reduced by rules below)
        twist.angular.z = 0.0  # default rotation (can be set by rules below)

        # Comprobamos que sea menor a la distancia límite para reaccionar (si no, dejamos cmd a cero → robot se detiene)
        if closest_distance < self.base_distance:

            # Diferencia de ángulo respecto a -90° (pared a la derecha) y ratio de distancia respecto a la distancia límite
            angle_error = angle_closest_distance + 90
            distance_ratio = max(0.0, min(closest_distance / self.base_distance, 1.0))
            
            #----------------------------------------------------------
            # RULE 1: speed proportional to angle and distance of closest obstacle
            #----------------------------------------------------------
            # Velocidad en X proporcional al error de ángulo y a la distancia al obstáculo
            # (más cerca → más lento, más lejos dentro del límite → más rápido)
            twist.linear.x = self.v_lin * (1 - abs(angle_error) / 90) * distance_ratio

            #----------------------------------------------------------
            # RULE 2: turn proportional to angle and distance of closest obstacle
            #----------------------------------------------------------

            # Velocidad de giro proporcional al error de ángulo y a la cercanía del obstáculo
            # (más cerca → más giro)
            twist.angular.z = self.v_ang * (angle_error / 10) * (2.0 - distance_ratio)


        # Update last commanded twist (periodic timer will publish it)
        self.cmd = twist

        # Update state for logging
        self._state_action = f"Movement: {twist.linear.x:.2f} m/s, {twist.angular.z:.2f} rad/s"

    #--------------------------------------------------------------------
    def log_info(self):
        if not self._shutting_down:
            # Log the last distance and angle to the closest obstacle and the current action
            self.get_logger().info(
                f"[DETECTION] Distance: {self._last_closest_distance:.2f} m | "
                f"Angle: {self._last_closest_angle:.0f}° | "
                f"State: {self._state_action}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
