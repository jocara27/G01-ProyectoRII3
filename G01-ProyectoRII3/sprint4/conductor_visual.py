import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

import json
import threading
import time
import math


class ConductorVisual(Node):
    def __init__(self):
        super().__init__('conductor_visual_node')

        # =========================
        # CONFIGURACIÓN
        # =========================
        self.ID_ROBOT = 3
        self.ID_DESTINO = 22

        # OJO: aquí deben estar (sí o sí) los IDs que realmente publicas
        self.listen_ids = [3, 8, 20, 21, 22, 23, self.ID_ROBOT]

        # Control
        self.kp_angular = 0.3
        self.kp_linear = 0.002
        self.distancia_minima = 60.0

        self.max_linear = 0.30
        self.max_angular = 1.50

        # Si no se actualiza un ArUco en X s, lo consideramos perdido
        self.stale_timeout = 0.50

        # Debug
        self.debug = True
        self._last_dbg = 0.0
        self._dbg_period = 1.0  # segundos: log 1 vez/segundo

        # Estado
        self.raw = {}
        self.lock = threading.Lock()

        # PUBLICADOR
        self.pub_cmd = self.create_publisher(Twist, '/robot1/cmd_vel', 10)

        # SUSCRIPCIONES
        self.subs = []
        for tid in sorted(set(self.listen_ids)):
            topic = f'/overhead_camera/aruco_{tid}'
            sub = self.create_subscription(
                String,
                topic,
                lambda msg, tid=tid: self.aruco_cb(msg, tid),
                10
            )
            self.subs.append(sub)
            if self.debug:
                self.get_logger().info(f'Suscrito a: {topic}')

        # TIMER DE CONTROL
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f'Iniciado. Robot={self.ID_ROBOT}, Destino={self.ID_DESTINO}, stale_timeout={self.stale_timeout}s'
        )

    def _dbg_throttle(self, msg: str):
        """Log con throttling para no spamear."""
        if not self.debug:
            return
        now = time.time()
        if (now - self._last_dbg) >= self._dbg_period:
            self._last_dbg = now
            self.get_logger().info(msg)

    def aruco_cb(self, msg: String, tid: int):
        """
        Espera JSON en msg.data, por ejemplo:
        {"id":20,"px":1085.6,"py":415.1,"orientation":178.7,...}
        """
        # 1) Ver si llega algo (si esto no se imprime nunca, NO estás recibiendo)
        if self.debug:
            # Muestra solo un trocito del mensaje para no saturar
            self._dbg_throttle(f'Recibiendo en tid={tid}: {msg.data[:80]}...')

        # 2) Parseo JSON
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'JSON inválido en tid={tid}: {e} | raw="{msg.data[:120]}"')
            return

        # 3) Extraer campos
        try:
            px = float(data.get('px'))
            py = float(data.get('py'))
            orientation = float(data.get('orientation'))
        except Exception as e:
            self.get_logger().warn(f'Campos faltantes/mal en tid={tid}: {e} | data={data}')
            return

        # 4) Guardar
        with self.lock:
            self.raw[tid] = {
                'px': px,
                'py': py,
                'orientation': orientation,
                'last_seen': time.time()
            }

    def control_loop(self):
        now = time.time()

        with self.lock:
            robot = self.raw.get(self.ID_ROBOT)
            target = self.raw.get(self.ID_DESTINO)

        # Debug: ¿tenemos datos?
        if robot is None or target is None:
            missing = []
            if robot is None:
                missing.append(f'robot(ID={self.ID_ROBOT})')
            if target is None:
                missing.append(f'destino(ID={self.ID_DESTINO})')

            self._dbg_throttle(
                f'Faltan datos: {", ".join(missing)}. '
                f'IDs vistos={sorted(list(self.raw.keys()))}'
            )
            self.pub_cmd.publish(Twist())
            return

        # Debug: ¿están stale?
        age_robot = now - robot['last_seen']
        age_target = now - target['last_seen']
        if age_robot > self.stale_timeout or age_target > self.stale_timeout:
            self._dbg_throttle(
                f'STALE: age_robot={age_robot:.2f}s, age_target={age_target:.2f}s '
                f'(timeout={self.stale_timeout:.2f}s). Paro.'
            )
            self.pub_cmd.publish(Twist())
            return

        # Coordenadas
        rx, ry = robot['px'], robot['py']
        tx, ty = target['px'], target['py']
        dx = tx - rx
        dy = ty - ry

        distancia = math.hypot(dx, dy)

        # Ángulo hacia objetivo
        angle_target = math.atan2(dy, dx)

        # Orientación robot (tu output parecía estar en grados)
        angle_robot = math.radians(robot['orientation'])

        # Error angular normalizado
        error_angle = angle_target - angle_robot
        error_angle = (error_angle + math.pi) % (2.0 * math.pi) - math.pi

        cmd = Twist()

        if distancia > self.distancia_minima:
            cmd.angular.z = self.kp_angular * error_angle

            if abs(error_angle) < 0.5:
                cmd.linear.x = self.kp_linear * distancia
            else:
                cmd.linear.x = 0.0

            # Saturación
            cmd.linear.x = max(-self.max_linear, min(self.max_linear, cmd.linear.x))
            cmd.angular.z = max(-self.max_angular, min(self.max_angular, cmd.angular.z))
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # Debug: imprimir cálculo y cmd
        self._dbg_throttle(
            f'r=({rx:.1f},{ry:.1f},ori={robot["orientation"]:.1f}deg) '
            f't=({tx:.1f},{ty:.1f}) dx={dx:.1f} dy={dy:.1f} '
            f'dist={distancia:.1f} err={error_angle:.2f}rad '
            f'cmd: lin={cmd.linear.x:.2f} ang={cmd.angular.z:.2f}'
        )

        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ConductorVisual()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
