from geometry_msgs.msg import Twist
import rclpy

def avanzar_pixels(node, publisher, velocidad_m_s, pixels, pixel_a_cm=(1/4.5), cm_a_metros=0.01):
    """
    MISMA lógica que tu script de avanzar:
    - Convierte px -> cm -> m
    - Calcula tiempo = distancia/velocidad
    - Publica lineal.x durante ese tiempo
    """
    if velocidad_m_s <= 0:
        node.get_logger().error("La velocidad debe ser mayor que 0")
        return

    distancia_cm = pixels * pixel_a_cm
    distancia_m = distancia_cm * cm_a_metros
    tiempo = distancia_m / velocidad_m_s

    twist = Twist()
    twist.linear.x = velocidad_m_s
    twist.angular.z = 0.0

    start_time = node.get_clock().now().nanoseconds / 1e9
    current_time = start_time

    while current_time - start_time < tiempo and rclpy.ok():
        publisher.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.05)
        current_time = node.get_clock().now().nanoseconds / 1e9

    # Detener
    twist.linear.x = 0.0
    publisher.publish(twist)
    

