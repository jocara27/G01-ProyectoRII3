import math
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DistanciaAruco(Node):
    def __init__(self):
        super().__init__('distancia_aruco')

        self.ROBOT_ID = 8
        self.ARUCOS = [20, 21, 22, 23]

        # Estado
        self.posicion_robot = None
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
        self.posicion_robot = (x, y)
        self.intentar_calculo()

    def callback_aruco(self, msg):
        data = json.loads(msg.data)
        aid = int(data["id"])
        x = float(data["px"])
        y = float(data["py"])
        self.posiciones_aruco[aid] = (x, y)
        self.intentar_calculo()

    # -------------------------
    # Calcular distancia en pixeles
    # -------------------------
    def calcular_distancia(self, x_obj, y_obj):
        dx = x_obj - self.posicion_robot[0]
        dy = y_obj - self.posicion_robot[1]
        return math.hypot(dx, dy)

    # -------------------------
    # Intentar calcular distancias
    # -------------------------
    def intentar_calculo(self):
        if self.posicion_robot is None:
            return
        if len(self.posiciones_aruco) < len(self.ARUCOS):
            return

        self.get_logger().info("DISTANCIAS EN PIXELES DESDE ARUCO 8:")
        for aid in self.ARUCOS:
            if aid in self.posiciones_aruco:
                x, y = self.posiciones_aruco[aid]
                distancia = self.calcular_distancia(x, y)
                self.get_logger().info(f"Aruco {aid} → Distancia: {distancia:.2f} px")

        # Terminar nodo después de calcular
        rclpy.shutdown()


# -------------------------
# MAIN
# -------------------------
def main():
    rclpy.init()
    node = DistanciaAruco()
    rclpy.spin(node)


if __name__ == "__main__":
    main()

