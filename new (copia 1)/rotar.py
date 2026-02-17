import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import floor, radians

# Inicializar ROS2
rclpy.init()

node = rclpy.create_node('rotador')
publisher = node.create_publisher(Twist, '/robot1/cmd_vel', 10)

# Obtener grados desde argumento 0 (primer argumento tras el script)
if len(sys.argv) < 2:
    print("Uso: ros2 run paquete script.py <grados>")
    sys.exit(1)

grados = float(sys.argv[1])
print (grados)

g_s = 100  # grados por segundo
iteraciones = abs(grados) / g_s
rotacion = radians(-g_s if grados >= 0 else g_s)

twist = Twist()
twist.linear.x = 0.0
twist.angular.z = rotacion

# Ejecutar rotación completa por segundos enteros
for _ in range(floor(iteraciones)):
    publisher.publish(twist)
    rclpy.spin_once(node, timeout_sec=0)
    time.sleep(1)

# Parte fraccionaria
fraccion = iteraciones % 1
if fraccion:
    grados_restantes = fraccion * g_s
    twist.angular.z = radians(-grados_restantes if grados >= 0 else grados_restantes)
    publisher.publish(twist)
    rclpy.spin_once(node, timeout_sec=0)
    time.sleep(1)


# Detener robot
twist.angular.z = 0.0
publisher.publish(twist)

# Cerrar ROS2
node.destroy_node()
rclpy.shutdown()

