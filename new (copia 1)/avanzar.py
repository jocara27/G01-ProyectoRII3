import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

PIXEL_A_CM = 1 / 3.6 # 1 cm = 4.5 px → cm = pixels / 4.5
CM_A_METROS = 0.01    # 1 cm = 0.01 m

class MoverLinea(Node):
    def __init__(self, velocidad, pixels):
        super().__init__('mover_linea_recta')
        self.publisher = self.create_publisher(Twist, '/robot1/cmd_vel', 10)

        # Calcular distancia en metros
        distancia_cm = pixels * PIXEL_A_CM
        self.distancia_m = distancia_cm * CM_A_METROS
        self.velocidad = velocidad

        # Calcular tiempo que debe moverse
        if velocidad <= 0:
            self.get_logger().error("La velocidad debe ser mayor que 0")
            rclpy.shutdown()
            return

        self.tiempo = self.distancia_m / self.velocidad
        self.get_logger().info(f"Mover {self.distancia_m:.3f} m a {self.velocidad:.2f} m/s durante {self.tiempo:.2f} s")

        self.mover_robot()

    def mover_robot(self):
        twist = Twist()
        twist.linear.x = self.velocidad
        twist.angular.z = 0.0

        start_time = self.get_clock().now().nanoseconds / 1e9
        current_time = start_time

        while current_time - start_time < self.tiempo and rclpy.ok():
            self.publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            current_time = self.get_clock().now().nanoseconds / 1e9

        # Detener robot
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        self.get_logger().info("Movimiento completado")
        rclpy.shutdown()


def main():
    if len(sys.argv) < 3:
        print("Uso: ros2 run paquete mover_linea.py <velocidad_m/s> <pixels>")
        sys.exit(1)

    velocidad = float(sys.argv[1])
    pixels = float(sys.argv[2])

    rclpy.init()
    node = MoverLinea(velocidad, pixels)


if __name__ == "__main__":
    main()

