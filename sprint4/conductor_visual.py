import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class ConductorVisual(Node):
    def __init__(self):
        super().__init__('conductor_visual_node')

        # 1. SUSCRIPCIONES Y PUBLICADORES
        self.sub_image = self.create_subscription(
            Image, '/overhead_camera/image_raw', self.procecar_imagen, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # 2. CONFIGURACIÓN
        self.br = CvBridge()
        # Ajusta el diccionario si usas otro (ej. DICT_5X5_100)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # IDs
        self.ID_ROBOT = 5       # El ID que lleva tu robot encima
        self.ID_DESTINO = 22    # Cambia esto al marcador al que quieras ir

        # Control PID (Ajusta estos valores si va muy rápido o oscila)
        self.kp_angular = 0.005  # Sensibilidad de giro
        self.kp_linear = 0.0005  # Sensibilidad de avance
        self.distancia_minima = 60 # En píxeles (cuándo parar)

        self.get_logger().info('Conductor Visual Iniciado. Buscando Robot ID 5...')

    def procecar_imagen(self, msg):
        # Convertir imagen ROS a OpenCV
        try:
            frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error imagen: {e}')
            return

        # Detectar ArUcos
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        if ids is None:
            return # No veo nada

        ids = ids.flatten()
        
        # Buscar si están el Robot y el Destino en la imagen
        if self.ID_ROBOT in ids and self.ID_DESTINO in ids:
            
            # --- 1. OBTENER COORDENADAS (EN PIXELES) ---
            # Índice del robot y del destino en la lista detectada
            idx_robot = np.where(ids == self.ID_ROBOT)[0][0]
            idx_dest = np.where(ids == self.ID_DESTINO)[0][0]

            # Esquinas del robot (para saber orientación)
            # Corner 0: Top-Left, 1: Top-Right, 2: Bottom-Right, 3: Bottom-Left
            c_robot = corners[idx_robot][0]
            center_robot = np.mean(c_robot, axis=0) # Centro del robot (x, y)
            
            # Calculamos el "frente" del robot (punto medio entre esquinas 0 y 1)
            front_robot = (c_robot[0] + c_robot[1]) / 2
            
            # Centro del destino
            c_dest = corners[idx_dest][0]
            center_dest = np.mean(c_dest, axis=0)

            # --- 2. CÁLCULOS MATEMÁTICOS ---
            
            # A) Ángulo actual del robot (Yaw en la imagen)
            # vector_robot = front - center
            angle_robot = math.atan2(front_robot[1] - center_robot[1], 
                                     front_robot[0] - center_robot[0])

            # B) Ángulo hacia el objetivo
            # vector_objetivo = destino - robot
            angle_target = math.atan2(center_dest[1] - center_robot[1], 
                                      center_dest[0] - center_robot[0])

            # C) Error de ángulo (¿Cuánto tengo que girar?)
            error_angle = angle_target - angle_robot
            # Normalizar ángulo entre -PI y PI (para que no de vueltas raras)
            error_angle = (error_angle + math.pi) % (2 * math.pi) - math.pi

            # D) Distancia al objetivo (Pitágoras)
            distancia = np.linalg.norm(center_dest - center_robot)

            # --- 3. DIBUJAR VISUALIZACIÓN (Opcional, para que tú veas qué piensa) ---
            cv2.line(frame, tuple(center_robot.astype(int)), tuple(center_dest.astype(int)), (255, 0, 0), 2) # Línea azul al destino
            cv2.arrowedLine(frame, tuple(center_robot.astype(int)), tuple(front_robot.astype(int)), (0, 255, 0), 2) # Flecha verde orientación
            
            # --- 4. CONTROLADOR (Mover el robot) ---
            cmd = Twist()

            if distancia > self.distancia_minima:
                # Si el ángulo es grande, giramos rápido y avanzamos poco
                # Si el ángulo es pequeño (ya estamos apuntando), corremos más
                cmd.angular.z = float(error_angle * self.kp_angular * -100) # El -100 es ajuste de escala, depende de tu simulador
                
                # Solo avanzamos si estamos mas o menos alineados (error < 0.5 radianes)
                if abs(error_angle) < 0.5:
                    cmd.linear.x = float(distancia * self.kp_linear)
                    if cmd.linear.x > 0.3: cmd.linear.x = 0.3 # Limitar velocidad máxima
                else:
                    cmd.linear.x = 0.0 # Gira en el sitio primero
            else:
                # Hemos llegado
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info('¡LLEGADA AL DESTINO!')

            # Publicar comando
            self.pub_cmd.publish(cmd)

        else:
            # Si pierdo de vista al robot o al destino, paro por seguridad
            self.pub_cmd.publish(Twist())

        # Mostrar ventana
        cv2.imshow("Navegacion Visual", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ConductorVisual()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
