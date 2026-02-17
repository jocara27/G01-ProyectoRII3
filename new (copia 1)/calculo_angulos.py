import math
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CalculoAngulos(Node):
    def __init__(self):
        super().__init__('calculo_angulos')

        self.ID_ROBOT = 8
        self.ARUCOS = [20, 21, 22, 23]

        # Configuración de cálculo de ángulos
        self.OFFSET_THETA = 90.0
        self.SIGNO = 1

        # Estado
        self.posicion_robot = None
        self.theta_inicial = None
        self.theta_actual = None
        self.posiciones_aruco = {}

        # Subscripción del robot
        self.create_subscription(String, '/overhead_camera/aruco_8', self.callback_robot, 10)

        # Subscripción de arucos objetivo
        for aid in self.ARUCOS:
            self.create_subscription(String, f'/overhead_camera/aruco_{aid}', self.callback_aruco, 10)

    # -------------------------
    # Callbacks
    # -------------------------
    def callback_robot(self, msg):
        data = json.loads(msg.data)
        x = float(data["px"])
        y = float(data["py"])
        theta_camara = float(data["orientation"])
        print(theta_camara)

        self.posicion_robot = (x, y)

        if self.theta_inicial is None:
            self.theta_inicial = theta_camara

        self.theta_actual = theta_camara - self.theta_inicial

        self.intentar_calculo()

    def callback_aruco(self, msg):
        data = json.loads(msg.data)
        aid = int(data["id"])
        x = float(data["px"])
        y = float(data["py"])

        self.posiciones_aruco[aid] = (x, y)
        self.intentar_calculo()

    # -------------------------
    # Cálculo de ángulo hacia un aruco
    # -------------------------
    def calcular_movimiento(self, x_obj, y_obj):
        dx = x_obj - self.posicion_robot[0]
        dy = y_obj - self.posicion_robot[1]

        theta_obj = math.degrees(math.atan2(dy, dx))

        # Ajuste de orientación (OFFSET_THETA)
        theta_obj = self.SIGNO * theta_obj + self.OFFSET_THETA

        # Diferencia angular entre objetivo y orientación actual
        diff = theta_obj - self.theta_actual

        # Normalizar al rango [-180, 180]
        angulo_giro = (diff + 180) % 360 - 180

        return angulo_giro

    # -------------------------
    # Calcular ángulos y mostrar resultados
    # -------------------------
    def intentar_calculo(self):
        if self.posicion_robot is None or len(self.posiciones_aruco) < len(self.ARUCOS):
            return

        self.get_logger().info("RESULTADOS DE ÁNGULOS:")
        for aid in self.ARUCOS:
            if aid in self.posiciones_aruco:
                x, y = self.posiciones_aruco[aid]
                angulo = self.calcular_movimiento(x, y)
                self.get_logger().info(f"Aruco {aid} → Girar: {angulo:.2f}°")

        # Terminar el nodo después de calcular todos los ángulos
        rclpy.shutdown()


# -------------------------
# MAIN
# -------------------------
def main():
    rclpy.init()
    node = CalculoAngulos()
    rclpy.spin(node)


if __name__ == "__main__":
    main()

