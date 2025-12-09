import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R

class ArucoTFPublisher(Node):
    def __init__(self):
        super().__init__('aruco_tf_publisher')

        # 1. Suscripciones
        self.img_sub = self.create_subscription(
            Image, '/overhead_camera/image_raw', self.image_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/overhead_camera/camera_info', self.info_callback, 10)

        # 2. Configuración ArUco y CV
        self.br = CvBridge()
        # Ajusta esto según tu versión de OpenCV (esto funciona para versiones modernas > 4.7)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Tamaño del marcador en METROS (Importante para calcular la distancia real)
        # El del robot mide 0.20m (20cm) según lo que pusimos en el SDF
        self.marker_length = 0.20 

        # 3. Variables de cámara
        self.camera_matrix = None
        self.dist_coeffs = None

        # 4. Publicador de TF (Transformaciones)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info('Nodo ArUco TF Publisher iniciado. Esperando camera_info...')

    def info_callback(self, msg):
        # Guardamos la matriz de la cámara una sola vez
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('¡Calibración de cámara recibida!')

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return # No podemos hacer nada sin calibración

        # Convertir imagen
        try:
            frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error imagen: {e}')
            return

        # Detectar
        corners, ids, rejected = cv2.aruco.detectMarkers(
            frame, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            # Estimar Pose (Posición y Rotación)
            # Nota: rvecs (vector rotación), tvecs (vector traslación)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

            # Dibujar y Publicar para cada marcador encontrado
            for i in range(len(ids)):
                # Dibujar ejes en la imagen (Visualización 2D)
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, 
                                  rvecs[i], tvecs[i], 0.1)
                
                # Publicar TF (Visualización 3D en RViz y uso para navegación)
                self.publish_tf(ids[i][0], rvecs[i], tvecs[i])

        # Mostrar ventana
        cv2.imshow("Vision Cenital con Ejes", frame)
        cv2.waitKey(1)

    def publish_tf(self, marker_id, rvec, tvec):
        t = TransformStamped()

        # Cabecera
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "overhead_camera_link" # El padre es la cámara
        t.child_frame_id = f"aruco_marker_{marker_id}" # El hijo es el marcador

        # Traslación (Metros)
        t.transform.translation.x = tvec[0][0]
        t.transform.translation.y = tvec[0][1]
        t.transform.translation.z = tvec[0][2]

        # Rotación (De vector de Rodrigues a Cuaternio)
        # OpenCV usa Rodrigues, ROS usa Cuaternios (x,y,z,w)
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        quat = R.from_matrix(rotation_matrix).as_quat()

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Publicar
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
