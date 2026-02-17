import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import sys

# =========================
# CONFIGURACIÓN
# =========================
TEMPLATE_FILENAME = "template_almacen.png"
THRESHOLD = 0.75 

class AlmacenDetectorGlobal(Node):
    def __init__(self):
        super().__init__('almacen_detector_global')
        
        # 1. Cargar Template
        script_dir = os.path.dirname(os.path.realpath(__file__))
        template_path = os.path.join(script_dir, TEMPLATE_FILENAME)
        
        self.template = cv2.imread(template_path)
        if self.template is None:
            self.get_logger().error(f"¡ERROR! No encuentro: {template_path}")
            sys.exit(1)
            
        self.template_gray = cv2.cvtColor(self.template, cv2.COLOR_BGR2GRAY)
        self.h, self.w = self.template_gray.shape[:2]

        self.subscription = self.create_subscription(
            Image, '/overhead_camera/image_raw', self.image_callback, 10)
        
        self.publisher_ = self.create_publisher(PoseArray, '/almacenes_detectados', 10)
        
        self.br = CvBridge()
        self.get_logger().info('Detector ordenado iniciado.')

    def image_callback(self, msg):
        try:
            current_frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

        img_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        res = cv2.matchTemplate(img_gray, self.template_gray, cv2.TM_CCOEFF_NORMED)

        loc = np.where(res >= THRESHOLD)
        rectangles = []
        for pt in zip(*loc[::-1]): 
            rectangles.append([int(pt[0]), int(pt[1]), int(self.w), int(self.h)])
            
        rects, weights = cv2.groupRectangles(rectangles, 1, 0.2)
        
        # --- NUEVO: CONVERTIR A LISTA Y ORDENAR ---
        # groupRectangles devuelve una tupla, la convertimos a lista para ordenar
        rects_list = list(rects)
        
        # Ordenamos la lista según la coordenada X (rect[0]) de menor a mayor
        # Menor X = Izquierda, Mayor X = Derecha
        rects_list.sort(key=lambda r: r[0])

        pose_array = PoseArray()
        pose_array.header = msg.header
        
        # Nombres para mostrar en terminal (asumiendo que detecta los 3)
        # Si detecta menos, asignará nombres genéricos, pero ordenados.
        labels = ["IZQUIERDA", "CENTRO", "DERECHA"]

        if len(rects_list) > 0:
            print("\n" + "="*40)
            print(f" DETECCIÓN (Ordenada de Izq a Der)")
            print("="*40)
            
            for i, (x, y, w, h) in enumerate(rects_list):
                cx = x + w // 2
                cy = y + h // 2
                
                # Asignar etiqueta para el print
                label_name = labels[i] if i < len(labels) else f"EXTRA_{i}"
                
                # A) IMPRIMIR CLARO EN TERMINAL
                print(f" {label_name:<10} | X: {cx:<4} | Y: {cy:<4}")
                
                # B) DIBUJAR (Añadimos el texto en la imagen también)
                cv2.rectangle(current_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(current_frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(current_frame, label_name, (x, y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # C) MENSAJE ROS
                p = Pose()
                p.position.x = float(cx)
                p.position.y = float(cy)
                p.position.z = 0.0
                pose_array.poses.append(p)

            self.publisher_.publish(pose_array)

        cv2.imshow("Detector Ordenado", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = AlmacenDetectorGlobal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
