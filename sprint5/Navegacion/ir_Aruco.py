import math
import json
import time
from math import floor, radians

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


# -------------------------
# CONSTANTES (las tuyas)
# -------------------------
ROBOT_ID = 8
ARUCOS_DISPONIBLES = [20, 21, 22, 23]

OFFSET_THETA = 90.0
SIGNO = 1

CMD_VEL_TOPIC = '/robot1/cmd_vel'

PIXEL_A_CM = 1 / 4.5   # cm = pixels / 4.5
CM_A_METROS = 0.01     # m = cm * 0.01

G_S = 100  # grados por segundo (rotación)


class IrAruco(Node):
    def __init__(self, aruco_objetivo: int, velocidad_m_s: float):
        super().__init__('ir_a_aruco_unificado')

        self.aruco_objetivo = aruco_objetivo
        self.velocidad = velocidad_m_s

        # Estado (igual que tu lógica)
        self.posicion_robot = None
        self.theta_inicial = None
        self.theta_actual = None

        self.posicion_objetivo = None

        # Publisher cmd_vel
        self.pub_cmd = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        # Subs robot
        self.create_subscription(String, f'/overhead_camera/aruco_{ROBOT_ID}', self.callback_robot, 10)

        # Subs solo del aruco elegido
        self.create_subscription(String, f'/overhead_camera/aruco_{aruco_objetivo}', self.callback_aruco_objetivo, 10)

        self.get_logger().info(f"Esperando datos del robot Aruco {ROBOT_ID} y objetivo Aruco {aruco_objetivo}...")


    # =========================================================
    # 1) FUNCIÓN: Calcular ángulo (MISMA lógica que tu script)
    # =========================================================
    def calcular_angulo_giro(self, x_obj, y_obj) -> float:
        dx = x_obj - self.posicion_robot[0]
        dy = y_obj - self.posicion_robot[1]

        theta_obj = math.degrees(math.atan2(dy, dx))

        # Ajuste de orientación (OFFSET_THETA)
        theta_obj = SIGNO * theta_obj + OFFSET_THETA

        # Diferencia angular entre objetivo y orientación actual
        diff = theta_obj - self.theta_actual

        # Normalizar al rango [-180, 180]
        angulo_giro = (diff + 180) % 360 - 180

        return angulo_giro


    # =========================================================
    # 2) FUNCIÓN: Calcular distancia (MISMA lógica que tu script)
    # =========================================================
    def calcular_distancia_pixeles(self, x_obj, y_obj) -> float:
        dx = x_obj - self.posicion_robot[0]
        dy = y_obj - self.posicion_robot[1]
        return math.hypot(dx, dy)


    # =========================================================
    # 3) FUNCIÓN: Rotar (MISMA lógica que tu script)
    # =========================================================
    def rotar_grados(self, grados: float):
        self.get_logger().info(f"Rotando {grados:.2f} grados...")

        iteraciones = abs(grados) / G_S
        rotacion = radians(-G_S if grados >= 0 else G_S)

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = rotacion

        # Ejecutar rotación completa por segundos enteros
        for _ in range(floor(iteraciones)):
            self.pub_cmd.publish(twist)
            rclpy.spin_once(self, timeout_sec=0)
            time.sleep(1)

        # Parte fraccionaria
        fraccion = iteraciones % 1
        if fraccion:
            grados_restantes = fraccion * G_S
            twist.angular.z = radians(-grados_restantes if grados >= 0 else grados_restantes)
            self.pub_cmd.publish(twist)
            rclpy.spin_once(self, timeout_sec=0)
            time.sleep(1)

        # Detener
        twist.angular.z = 0.0
        self.pub_cmd.publish(twist)
        self.get_logger().info("Rotación completada.")


    # =========================================================
    # 4) FUNCIÓN: Avanzar (MISMA lógica que tu script)
    # =========================================================
    def avanzar_pixels(self, pixels: float):
        # Calcular distancia en metros
        distancia_cm = pixels * PIXEL_A_CM
        distancia_m = distancia_cm * CM_A_METROS

        if self.velocidad <= 0:
            self.get_logger().error("La velocidad debe ser mayor que 0")
            return

        tiempo = distancia_m / self.velocidad
        self.get_logger().info(f"Avanzando {distancia_m:.3f} m a {self.velocidad:.2f} m/s durante {tiempo:.2f} s")

        twist = Twist()
        twist.linear.x = self.velocidad
        twist.angular.z = 0.0

        start_time = self.get_clock().now().nanoseconds / 1e9
        current_time = start_time

        while current_time - start_time < tiempo and rclpy.ok():
            self.pub_cmd.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            current_time = self.get_clock().now().nanoseconds / 1e9

        # Detener
        twist.linear.x = 0.0
        self.pub_cmd.publish(twist)
        self.get_logger().info("Avance completado.")


    # -------------------------
    # Callbacks (robot / objetivo)
    # -------------------------
    def callback_robot(self, msg: String):
        data = json.loads(msg.data)
        x = float(data["px"])
        y = float(data["py"])
        theta_camara = float(data["orientation"])

        self.posicion_robot = (x, y)

        if self.theta_inicial is None:
            self.theta_inicial = theta_camara

        self.theta_actual = theta_camara - self.theta_inicial

        self.intentar_ejecutar()


    def callback_aruco_objetivo(self, msg: String):
        data = json.loads(msg.data)
        x = float(data["px"])
        y = float(data["py"])

        self.posicion_objetivo = (x, y)
        self.intentar_ejecutar()


    # -------------------------
    # Lógica principal: cuando tengo TODO, giro + avanzo y cierro
    # -------------------------
    def intentar_ejecutar(self):
        if self.posicion_robot is None:
            return
        if self.theta_actual is None:
            return
        if self.posicion_objetivo is None:
            return

        x_obj, y_obj = self.posicion_objetivo

        angulo = self.calcular_angulo_giro(x_obj, y_obj)
        distancia_px = self.calcular_distancia_pixeles(x_obj, y_obj)

        self.get_logger().info("=== OBJETIVO DETECTADO ===")
        self.get_logger().info(f"Aruco {self.aruco_objetivo} -> Girar: {angulo:.2f}°")
        self.get_logger().info(f"Aruco {self.aruco_objetivo} -> Distancia: {distancia_px:.2f} px")

        # Ejecutar movimiento (rotar -> avanzar)
        self.rotar_grados(angulo)
        self.avanzar_pixels(distancia_px)

        # Cerrar
        rclpy.shutdown()


def main():
    # Preguntar aruco por consola (como pediste)
    print(f"ARUCOS DISPONIBLES: {ARUCOS_DISPONIBLES}")
    aruco = int(input("¿A qué Aruco quieres ir?: ").strip())

    if aruco not in ARUCOS_DISPONIBLES:
        print("Aruco no válido. Saliendo.")
        return

    # Velocidad por consola (si no quieres preguntar, lo fijo a 0.10)
    entrada_v = input("Velocidad (m/s) [por defecto 0.10]: ").strip()
    velocidad = float(entrada_v) if entrada_v else 0.10

    rclpy.init()
    node = IrAruco(aruco_objetivo=aruco, velocidad_m_s=velocidad)
    rclpy.spin(node)


if __name__ == "__main__":
    main()

