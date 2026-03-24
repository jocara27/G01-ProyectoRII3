import time
from math import floor, radians
from geometry_msgs.msg import Twist
import rclpy

def rotar_grados(node, publisher, grados, g_s=100):
    """
    MISMA lógica que tu script de rotación.
    Publica en cmd_vel durante el tiempo necesario.
    """
    iteraciones = abs(grados) / g_s
    rotacion = radians(-g_s if grados >= 0 else g_s)

    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = rotacion

    # Segundos enteros
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

    # Detener
    twist.angular.z = 0.0
    publisher.publish(twist)


