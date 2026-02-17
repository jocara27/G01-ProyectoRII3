import json
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String

from calculo_angulos_funcion import calcular_angulo_giro
from calculo_distancia_funcion import calcular_distancia_pixeles
from rotar_funcion import rotar_grados
from avanzar_funcion import avanzar_pixels

from geometry_msgs.msg import Twist


ROBOT_ID = 8
ARUCOS_DISPONIBLES = [20, 21, 22, 23]

OFFSET_THETA = 90.0
SIGNO = 1

CMD_VEL_TOPIC = '/robot1/cmd_vel'

#  Timeouts (para tolerar tapado)
ROBOT_TIMEOUT_S = 0.7   # el robot debe estar "reciente"
OBJ_TIMEOUT_S = 3.0     # el objetivo puede estar tapado unos segundos y aún usar el último dato


class IrArucoMain(Node):
    def __init__(self, aruco_objetivo: int, velocidad_m_s: float):
        super().__init__('main_ir_aruco')

        self.aruco_objetivo = aruco_objetivo
        self.velocidad = velocidad_m_s

        # Estado (igual que antes)
        self.posicion_robot = None
        self.theta_inicial = None
        self.theta_actual = None
        self.posicion_objetivo = None

        # ⬇️ Última vez que vimos robot/objetivo (para que si se tapa no se muera)
        self.t_last_robot = None
        self.t_last_obj = None

        # Para evitar ejecutar dos veces por callbacks repetidos
        self.ejecutado = False

        # Publisher cmd_vel
        self.pub_cmd = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        # Subscripciones
        self.create_subscription(String, f'/overhead_camera/aruco_{ROBOT_ID}', self.callback_robot, 10)
        self.create_subscription(String, f'/overhead_camera/aruco_{aruco_objetivo}', self.callback_aruco_objetivo, 10)

        # ⬇️ Timer: intenta ejecutar aunque no lleguen más callbacks (si el aruco se tapa)
        self.create_timer(0.1, self.intentar_ejecutar)

        self.get_logger().info(f"Esperando Aruco robot {ROBOT_ID} y objetivo {aruco_objetivo}...")

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def callback_robot(self, msg: String):
        data = json.loads(msg.data)
        x = float(data["px"])
        y = float(data["py"])
        theta_camara = float(data["orientation"])

        self.posicion_robot = (x, y)
        self.t_last_robot = self._now_s()

        if self.theta_inicial is None:
            self.theta_inicial = theta_camara  # "pared" = orientación buena

        self.theta_actual = theta_camara - self.theta_inicial

    def callback_aruco_objetivo(self, msg: String):
        data = json.loads(msg.data)
        x = float(data["px"])
        y = float(data["py"])
        self.posicion_objetivo = (x, y)
        self.t_last_obj = self._now_s()

    def intentar_ejecutar(self):
        if self.ejecutado:
            return

        # Necesito haber visto AL MENOS una vez robot y objetivo
        if self.posicion_robot is None or self.theta_actual is None:
            return
        if self.posicion_objetivo is None:
            return
        if self.t_last_robot is None or self.t_last_obj is None:
            return

        now = self._now_s()

        # Robot debe estar reciente (si no, el ángulo puede salir mal)
        if (now - self.t_last_robot) > ROBOT_TIMEOUT_S:
            return

        # Objetivo puede estar tapado un rato, pero no infinito
        if (now - self.t_last_obj) > OBJ_TIMEOUT_S:
            return

        self.ejecutado = True  # bloqueamos para que no se repita

        x_obj, y_obj = self.posicion_objetivo

        angulo = calcular_angulo_giro(
            posicion_robot=self.posicion_robot,
            theta_actual=self.theta_actual,
            x_obj=x_obj,
            y_obj=y_obj,
            offset_theta=OFFSET_THETA,
            signo=SIGNO
        )

        distancia_px = calcular_distancia_pixeles(
            posicion_robot=self.posicion_robot,
            x_obj=x_obj,
            y_obj=y_obj
        )

        self.get_logger().info("=== OBJETIVO DETECTADO (o último válido) ===")
        self.get_logger().info(f"Aruco {self.aruco_objetivo} → Girar: {angulo:.2f}°")
        self.get_logger().info(f"Aruco {self.aruco_objetivo} → Distancia: {distancia_px:.2f} px")

        # Ejecutar movimiento (rotar -> esperar -> avanzar -> volver a mirar a la pared)
        rotar_grados(self, self.pub_cmd, angulo, g_s=100)
        time.sleep(2)  # espera 2 segundos después de rotar

        avanzar_pixels(self, self.pub_cmd, self.velocidad, distancia_px, pixel_a_cm=(1/3.3), cm_a_metros=0.01)

        # Volver a la orientación inicial ("pared")
        angulo_vuelta = -angulo
        self.get_logger().info(f"Volviendo a orientar a la pared: Girar {angulo_vuelta:.2f}°")
        rotar_grados(self, self.pub_cmd, angulo_vuelta, g_s=100)

        rclpy.shutdown()


def main():
    print(f"ARUCOS DISPONIBLES: {ARUCOS_DISPONIBLES}")
    aruco = int(input("¿A qué Aruco quieres ir?: ").strip())

    if aruco not in ARUCOS_DISPONIBLES:
        print("Aruco no válido. Saliendo.")
        return

    entrada_v = input("Velocidad (m/s) [por defecto 0.10]: ").strip()
    velocidad = float(entrada_v) if entrada_v else 0.10

    rclpy.init()
    node = IrArucoMain(aruco_objetivo=aruco, velocidad_m_s=velocidad)
    rclpy.spin(node)


if __name__ == "__main__":
    main()

