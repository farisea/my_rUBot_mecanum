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
        self.declare_parameter('distance_limit', 0.35)    # desired distance to right wall
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
        # Best-effort scanning
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
            "WallFollower holonomic enabled (uses vx, vy and wz for mecanum)."
        )

        # Ticks de LiDAR mirando el frente
        self.ticks_front = 0
        self.ticks_angular_z = 0

        # Máx ticks virtual
        self.max_front = 15
        self.max_z = 12

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

        self.cmd = Twist()

        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass

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
            pass

    #--------------------------------------------------------------------
    def _clamp(self, value, low, high):
        return max(low, min(high, value))

    #--------------------------------------------------------------------
    def _saturate(self, value, endpoint):
        return self._clamp(value, -endpoint, endpoint)

    #--------------------------------------------------------------------
    def laser_callback(self, scan):
        """Compute control action from LIDAR and update self.cmd."""
        if self._shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        # Dividimos los 360° en 6 zonas. Referencia: 0° = frente, -90° = derecha, +90° = izquierda.
        # Usamos 6 zonas en lugar de las típicas 3 porque el movimiento holonómico
        # nos permite reaccionar de forma diferente según de dónde venga el obstáculo,
        # sin necesidad de girar el robot entero.
        min_front       = math.inf
        min_left        = math.inf   # detecta si hay pared a la izquierda 
        min_fr_right    = math.inf   # frente-derecha: junto con BACK_RIGHT permite calcular el ángulo con la pared
        min_right       = math.inf
        min_back_right  = math.inf   # atrás-derecha: junto con FR_RIGHT permite calcular el ángulo con la pared
        min_back        = math.inf   # solo se usa si ninguna otra zona tiene obstáculo cercano
        
        
        # Ángulos mínimos para las zonas principales
        min_front_angle = math.inf
        min_right_angle = math.inf

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            ang = angle_min + i * angle_inc

            if   -30  <= ang <=  40:
                if d < min_front:
                    min_front = d
                    min_front_angle = ang

            elif  40  <  ang <= 140:
                if d < min_left:
                    min_left = d

            elif -40  <= ang <  -30:
                if d < min_fr_right:
                    min_fr_right = d

            elif -130 <= ang <  -40:
                if d < min_right:
                    min_right = d
                    min_right_angle = ang

            elif -140 <= ang < -130:
                if d < min_back_right:
                    min_back_right = d

            elif ang < -140 or ang > 140:
                if d < min_back:
                    min_back = d

        twist  = Twist()
        action = ""

        # Umbral de reacción: si hay algo a menos de (base_distance + tolerance) se activa la evasión.
        # La tolerancia evita que el robot corrija constantemente por pequeñas oscilaciones.
        reaction_limit = self.base_distance + self.tol

        # BACK solo tiene en cuenta si ninguna zona prioritaria tiene un obstáculo cercano,
        # así evitamos que una pared trasera lejana interfiera con la evasión frontal.
        zone_min = {
            'FRONT':       min_front,
            'FRONT_RIGHT': min_fr_right,
            'RIGHT':       min_right,
            'BACK_RIGHT':  min_back_right,
        }
        if not any(math.isfinite(v) and v < reaction_limit for v in zone_min.values()):
            zone_min['BACK'] = min_back

        closest_zone, closest_distance = min(
            zone_min.items(), key=lambda item: item[1]
        )

        # PRIORIDAD 1: giro si ha pasado mucho tiempo con obstáculo en el frente o ve algo a la izquierda
        if self.ticks_front > self.max_front or self.ticks_angular_z > 0:
            # Si los angular ticks son mayores que 0, significa que ya hemos empezado a girar, así que seguimos girando hasta completar la maniobra
            # Paramos cuando llegue a 12 ticks angular
            if self.ticks_angular_z > self.max_z:
                self.ticks_front = 0
                self.ticks_angular_z = 0
                action = f"FRONT completed turn left. FINAL turn LEFT tick={self.ticks_angular_z}"
            else:
                # Giramos a la izquierda
                twist.linear.x  = 0.0
                twist.linear.y  = 0.0
                twist.angular.z = self.v_ang * 3

                # Reiniciamos los ticks para evitar que se acumulen indefinidamente
                self.ticks_front = 0
                self.ticks_angular_z += 1
                action = f"FRONT time-out ({closest_distance:.2f} m) -> turn LEFT tick={self.ticks_angular_z}"
            
            
        # PRIORIDAD 2: obstáculo cercano → evasión holonómica reactiva
        elif math.isfinite(closest_distance) and closest_distance < reaction_limit:

            # Para cada zona, la reacción se adapta a la dirección del obstáculo:
            if closest_zone == 'FRONT':
                # Añadimos un tick
                self.ticks_front += 1
                """
                if math.isfinite(min_left) and min_left < self.base_distance * 0.9:
                    # Esquina interior (bloqueado por delante y por la izquierda):
                    # no hay espacio para strafear → retrocedemos un poco y rotamos
                    # en sentido antihorario para sacar el frente de la esquina.
                    twist.linear.x  =  0.0
                    twist.linear.y  =  0.0
                    twist.angular.z =  self.v_ang
                    action = (f"FRONT+LEFT corner ({closest_distance:.2f} m, "
                              f"left={min_left:.2f} m) -> GIRAR")
                else:
                    """
                # Obstáculo solo por delante, izquierda libre:
                # Si está muy cerca, damos más prioridad al retroceso para evitar chocar
                if closest_distance < self.base_distance * 0.5:
                    twist.linear.x  = -self.v_lin * 0.3
                    twist.linear.y  =  0.0
                    twist.angular.z =  0.0
                # Si está a una distancia moderada, movemos hacia la izquierda y corregimos el ángulo hasta 0
                else:
                    angular_correction_ratio = min_front_angle * 0.2 # Proporcional a lo cerca que esté del umbral frontal
                    twist.linear.x  =  0.0
                    twist.linear.y  =  self.v_lin
                    twist.angular.z =  self._saturate(angular_correction_ratio * self.v_ang, self.v_ang)
                action = f"FRONT {closest_distance:.2f} m -> move LEFT + rotate to 0°. Ticks = {self.ticks_front}"

            elif closest_zone == 'FRONT_RIGHT':
                # Reset de ticks
                self.ticks_front = 0

                # Obstáculo en diagonal delantera-derecha:
                # movimiento oblicuo hacia delante-izquierda
                twist.linear.x  =  self.v_lin * 0.5
                twist.linear.y  =  self.v_lin * 0.5
                twist.angular.z =  0.0
                action = f"FRONT-RIGHT {closest_distance:.2f} m -> move FRONT-LEFT"

            elif closest_zone == 'RIGHT':
                # Reset de ticks
                self.ticks_front = 0

                # Demasiado cerca de la pared derecha
                if closest_distance < self.base_distance * 0.5:
                    # Si está muy cerca, damos más prioridad a ir a la izquierda
                    twist.linear.x  =  0.0
                    twist.linear.y  =  self.v_lin * 0.3
                    twist.angular.z =  0.0
                    action = f"RIGHT too CLOSE ({closest_distance:.2f} m) -> move LEFT"
                
                # Si está a una distancia moderada, movemos hacia la delante y corregimos el ángulo
                else:
                    twist.linear.x = self.v_lin
                    twist.linear.y = 0.0
                    # Corrección de ángulo dependiendo del min_right_angle (queremos -90°):
                    angular_correction = (min_right_angle + 90) * 0.2
                    twist.angular.z = self._saturate(angular_correction * self.v_ang, self.v_ang)
                    action = (f"RIGHT {min_right:.2f} m -> follow wall "
                            f"(vy={twist.linear.y:.2f}, wz={twist.angular.z:.2f})")

            elif closest_zone == 'BACK_RIGHT':
                # Reset de ticks
                self.ticks_front = 0

                # La pared ha quedado detrás-derecha (el robot se alejó demasiado):
                # movimiento diagonal adelante-derecha a 45° para recuperar
                # la posición de seguimiento sin girar.
                twist.linear.x  =  self.v_lin * 0.5
                twist.linear.y  = -self.v_lin * 0.5
                twist.angular.z =  0.0
                action = f"BACK-RIGHT {closest_distance:.2f} m -> move FRONT-RIGHT"

            elif closest_zone == 'BACK':
                # Reset de ticks
                self.ticks_front = 0
                
                # Pared justo detrás (solo activo si las demás zonas están despejadas):
                # strafe puro a la derecha para ir a buscar la pared lateral.
                twist.linear.x  =  0.0
                twist.linear.y  = -self.v_lin
                twist.angular.z =  0.0
                action = f"BACK {closest_distance:.2f} m -> move RIGHT"

        # PRIORIDAD 3: sin pared visible → búsqueda activa
        # El robot avanza en diagonal hacia la derecha y gira levemente en sentido
        # horario para barrer el espacio hasta encontrar la pared derecha.
        else:
            # Reset de ticks
            self.ticks_front = 0
            
            twist.linear.x  =  0.0
            twist.linear.y  = -self.v_lin * 0.4
            twist.angular.z =  0.0
            action = "No wall detected -> search RIGHT wall"

        self.cmd = twist

        if action != self._last_action_logged:
            self.get_logger().info(action if action else "No action (stopped).")
            self._last_action_logged = action

        self._state_action = action if action else "Stopped (no wall detected)"

    #--------------------------------------------------------------------
    def log_info(self):
        if not self._shutting_down:
            self.get_logger().info(self._state_action)


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
