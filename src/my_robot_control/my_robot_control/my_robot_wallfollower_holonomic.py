import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


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
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
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
        FRONT      = []
        LEFT       = []   # detecta si hay pared a la izquierda 
        FR_RIGHT   = []   # frente-derecha: junto con BACK_RIGHT permite calcular el ángulo con la pared
        RIGHT      = []
        BACK_RIGHT = []   # atrás-derecha: junto con FR_RIGHT permite calcular el ángulo con la pared
        BACK       = []   # solo se usa si ninguna otra zona tiene obstáculo cercano

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            ang = angle_min + i * angle_inc

            if   -20  <= ang <=  20:          FRONT.append(d)
            elif  20  <  ang <= 110:          LEFT.append(d)
            elif -70  <= ang <  -20:          FR_RIGHT.append(d)
            elif -110 <= ang <  -70:          RIGHT.append(d)
            elif -160 <= ang < -110:          BACK_RIGHT.append(d)
            elif ang < -160 or ang > 160:     BACK.append(d)

        min_front      = min(FRONT)      if FRONT      else float('inf')
        min_fr_right   = min(FR_RIGHT)   if FR_RIGHT   else float('inf')
        min_right      = min(RIGHT)      if RIGHT      else float('inf')
        min_back_right = min(BACK_RIGHT) if BACK_RIGHT else float('inf')
        min_back       = min(BACK)       if BACK       else float('inf')
        min_left       = min(LEFT)       if LEFT       else float('inf')

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

        # PRIORIDAD 1: obstáculo cercano → evasión holonómica reactiva
        # La idea clave es usar vy como primera respuesta,
        # sin necesidad de frenar y girar como haría un robot diferencial.
        # wz solo se usa para alineación, no para evadir.
        if math.isfinite(closest_distance) and closest_distance < reaction_limit:

            if closest_zone == 'FRONT':
                if math.isfinite(min_left) and min_left < self.base_distance * 0.9:
                    # Esquina interior (bloqueado por delante y por la izquierda):
                    # no hay espacio para strafear → retrocedemos un poco y rotamos
                    # en sentido antihorario para sacar el frente de la esquina.
                    twist.linear.x  = -self.v_lin * 0.3
                    twist.linear.y  =  0.0
                    twist.angular.z =  self.v_ang
                    action = (f"FRONT+LEFT corner ({closest_distance:.2f} m, "
                              f"left={min_left:.2f} m) -> recul + GIRAR")
                else:
                    # Obstáculo solo por delante, izquierda libre:
                    # strafe lateral izquierdo puro + pequeño retroceso para ganar margen.
                    # No giramos → el robot mantiene su orientación y esquiva más rápido.
                    twist.linear.x  = -self.v_lin * 0.3
                    twist.linear.y  =  self.v_lin
                    twist.angular.z =  0.0
                    action = f"FRONT {closest_distance:.2f} m -> recul + move LEFT"

            elif closest_zone == 'FRONT_RIGHT':
                # Obstáculo en diagonal delantera-derecha:
                # movimiento oblicuo hacia delante-izquierda (más lateral que retroceso)
                # para alejarse de la esquina sin perder demasiado avance.
                twist.linear.x  = -self.v_lin * 0.2
                twist.linear.y  =  self.v_lin * 0.8
                twist.angular.z =  0.0
                action = f"FRONT-RIGHT {closest_distance:.2f} m -> recul + move FRONT-LEFT"

            elif closest_zone == 'RIGHT':
                # Demasiado cerca de la pared derecha:
                # control proporcional directo sobre vy — el error de distancia
                # se convierte en velocidad lateral sin necesidad de girar.
                lateral_error  = self.base_distance - min_right
                twist.linear.x = self.v_lin
                twist.linear.y = self._clamp(lateral_error * 1.8, -self.v_lin, self.v_lin)
                # Corrección de ángulo: comparamos la distancia delantera-derecha con la
                # trasera-derecha. Si difieren, el robot está girado respecto a la pared
                # → wz proporcional para enderezarlo. Dead-band de 0.08 m para evitar
                # oscilaciones cuando el robot ya está suficientemente alineado.
                if math.isfinite(min_fr_right) and math.isfinite(min_back_right):
                    align_error = min_back_right - min_fr_right
                    if abs(align_error) > 0.08:
                        twist.angular.z = self._clamp(
                            align_error * 0.4, -self.v_ang * 0.5, self.v_ang * 0.5
                        )
                    else:
                        twist.angular.z = 0.0
                action = (f"RIGHT {min_right:.2f} m -> follow wall "
                          f"(vy={twist.linear.y:.2f}, wz={twist.angular.z:.2f})")

            elif closest_zone == 'BACK_RIGHT':
                # La pared ha quedado detrás-derecha (el robot se alejó demasiado):
                # movimiento diagonal adelante-derecha a 45° para recuperar
                # la posición de seguimiento sin girar.
                twist.linear.x  =  self.v_lin * 0.7
                twist.linear.y  = -self.v_lin * 0.7
                twist.angular.z =  0.0
                action = f"BACK-RIGHT {closest_distance:.2f} m -> move FRONT-RIGHT"

            elif closest_zone == 'BACK':
                # Pared justo detrás (solo activo si las demás zonas están despejadas):
                # strafe puro a la derecha para ir a buscar la pared lateral.
                twist.linear.x  =  0.0
                twist.linear.y  = -self.v_lin
                twist.angular.z =  0.0
                action = f"BACK {closest_distance:.2f} m -> move RIGHT"

        # PRIORIDAD 2: seguimiento normal de la pared derecha
        # Igual que el caso RIGHT de arriba pero con ganancia más alta (3.0 vs 1.8)
        # porque aquí no hay urgencia de evasión y podemos ser más precisos con
        # la distancia objetivo. vy y wz actúan de forma independiente y simultánea.
        
        elif math.isfinite(min_right):
            lateral_error  = self.base_distance - min_right
            twist.linear.x = self.v_lin
            # Higher gain (3.0) to stay close to the target distance
            twist.linear.y = self._clamp(lateral_error * 3.0,
                                          -0.8 * self.v_lin, 0.8 * self.v_lin)
            # Alignment correction with dead-band to avoid oscillation
            if math.isfinite(min_fr_right) and math.isfinite(min_back_right):
                align_error = min_back_right - min_fr_right
                if abs(align_error) > 0.08:
                    twist.angular.z = self._clamp(
                        align_error * 0.4, -self.v_ang * 0.5, self.v_ang * 0.5
                    )
                else:
                    twist.angular.z = 0.0
            action = (f"TRACK RIGHT ({min_right:.2f} m, target {self.base_distance:.2f}) "
                      f"-> vx={twist.linear.x:.2f}, vy={twist.linear.y:.2f}, "
                      f"wz={twist.angular.z:.2f}")

        # PRIORIDAD 3: sin pared visible → búsqueda activa
        # El robot avanza en diagonal hacia la derecha y gira levemente en sentido
        # horario para barrer el espacio hasta encontrar la pared derecha.
        else:
            twist.linear.x  =  self.v_lin * 0.4
            twist.linear.y  = -self.v_lin * 0.4
            twist.angular.z = -self.v_ang * 0.3
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