import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        # SUSCRIPCIÓN: Cámara cenital de Gazebo
        self.subscription = self.create_subscription(
            Image,
            '/overhead_camera/image_raw', 
            self.image_callback,
            10)
        
        self.br = CvBridge()
        
        # --- CORRECCIÓN PARA OPENCV 4.7+ ---
        # Antes era: cv2.aruco.Dictionary_get(...)
        # Ahora es:  cv2.aruco.getPredefinedDictionary(...)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        
        # Antes era: cv2.aruco.DetectorParameters_create()
        # Ahora es:  cv2.aruco.DetectorParameters()
        self.aruco_params = cv2.aruco.DetectorParameters()
        # -----------------------------------

        self.get_logger().info('Nodo detector de ArUcos iniciado. Esperando imágenes...')

    def image_callback(self, msg):
        try:
            current_frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error al convertir imagen: {e}')
            return

        # Detectar ArUcos
        corners, ids, rejected = cv2.aruco.detectMarkers(
            current_frame, 
            self.aruco_dict, 
            parameters=self.aruco_params
        )

        if ids is not None:
            self.get_logger().info(f'¡Detectado ArUco ID: {ids.flatten()}!')
            # Dibujar el cuadrado
            cv2.aruco.drawDetectedMarkers(current_frame, corners, ids)

        # Mostrar imagen
        cv2.imshow("Camara Cenital", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    detector = ArucoDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    detector.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
