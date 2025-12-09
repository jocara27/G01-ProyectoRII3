import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import threading # Necesario para ejecutar input() y ROS a la vez

class ConductorInfinito(Node):
    def __init__(self):
        super().__init__('conductor_infinito_node')

        # ESTADO DEL ROBOT
        self.ID_ROBOT = 5 
        self.ID_DESTINO = None # Al principio no vamos a ningún lado
        self.meta_alcanzada = True # Asumimos que estamos parados
        self.ultima_distancia_conocida = 9999.0 

        # ROS
        self.sub_image = self.create_subscription(
            Image, '/overhead_camera/image_raw', self.procesar_imagen, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # VISION
        self.br = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # CONTROL
        self.kp_angular = 0.004
        self.kp_linear = 0.0006
        self.distancia_minima = 45

    def set_nuevo_destino(self, target_id):
        """Función para actualizar el destino desde el menú"""
        self.ID_DESTINO = target_id
        self.meta_alcanzada = False
        self.ultima_distancia_conocida = 9999.0
        self.get_logger().info(f'>>> NUEVO DESTINO RECIBIDO: {target_id} <<<')

    def procesar_imagen(self, msg):
        # Si no tengo destino o ya llegué, solo muestro la imagen (modo pasivo)
        if self.ID_DESTINO is None or self.meta_alcanzada:
            self.mostrar_imagen_pasiva(msg)
            return

        try:
            frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        except: return

        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
        
        deteccion_exitosa = False

        if ids is not None:
            ids = ids.flatten()
            if self.ID_ROBOT in ids and self.ID_DESTINO in ids:
                deteccion_exitosa = True
                
                # --- CALCULO DE RUTA ---
                idx_robot = np.where(ids == self.ID_ROBOT)[0][0]
                idx_dest = np.where(ids == self.ID_DESTINO)[0][0]

                c_robot = corners[idx_robot][0]
                center_robot = np.mean(c_robot, axis=0)
                front_robot = (c_robot[0] + c_robot[1]) / 2 
                
                c_dest = corners[idx_dest][0]
                center_dest = np.mean(c_dest, axis=0)

                angle_robot = math.atan2(front_robot[1] - center_robot[1], front_robot[0] - center_robot[0])
                angle_target = math.atan2(center_dest[1] - center_robot[1], center_dest[0] - center_robot[0])
                
                error_angle = (angle_target - angle_robot + math.pi) % (2 * math.pi) - math.pi
                distancia = np.linalg.norm(center_dest - center_robot)
                
                self.ultima_distancia_conocida = distancia

                # DIBUJOS
                cv2.line(frame, tuple(center_robot.astype(int)), tuple(center_dest.astype(int)), (0, 0, 255), 2)
                cv2.putText(frame, f"Go to {self.ID_DESTINO} | Dist: {int(distancia)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # MOVIMIENTO
                cmd = Twist()
                if distancia > self.distancia_minima:
                    cmd.angular.z = float(error_angle * self.kp_angular * -100)
                    if abs(error_angle) < 0.5:
                        cmd.linear.x = float(distancia * self.kp_linear)
                        if cmd.linear.x > 0.25: cmd.linear.x = 0.25
                    else:
                        cmd.linear.x = 0.0 
                    self.pub_cmd.publish(cmd)
                else:
                    self.finalizar_exito(frame)

        # GESTION DE PERDIDA (SOLAPAMIENTO)
        if not deteccion_exitosa:
            if self.ultima_distancia_conocida < 140: 
                self.finalizar_exito(frame)
            else:
                self.pub_cmd.publish(Twist()) # Parar y buscar
                cv2.putText(frame, "BUSCANDO...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("Conductor Infinito", frame)
        cv2.waitKey(1)

    def mostrar_imagen_pasiva(self, msg):
        """Muestra la cámara aunque el robot esté parado"""
        try:
            frame = self.br.imgmsg_to_cv2(msg, "bgr8")
            cv2.putText(frame, "ESPERANDO ORDEN...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            cv2.imshow("Conductor Infinito", frame)
            cv2.waitKey(1)
        except: pass

    def finalizar_exito(self, frame):
        self.pub_cmd.publish(Twist()) # Freno total
        self.meta_alcanzada = True
        self.get_logger().info(f' LLEGADA CONFIRMADA AL {self.ID_DESTINO}. Esperando siguiente orden.')
        
        # Mensaje visual
        cv2.putText(frame, "META ALCANZADA", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
        cv2.imshow("Conductor Infinito", frame)
        cv2.waitKey(1)


def menu_interactivo(node):
    """Este bucle corre en paralelo para pedir inputs"""
    print("\n" + "="*30)
    print("   NAVIGATOR INFINITO  ")
    print("==============================")
    print("Destinos válidos: 20, 21, 22, 23 (o 1)")
    print("Escribe 'q' para salir.")
    
    while rclpy.ok():
        try:
            entrada = input("\n Introduce siguiente ID destino: ")
            
            if entrada.lower() == 'q':
                print("Cerrando sistema...")
                rclpy.shutdown()
                break
            
            target_id = int(entrada)
            node.set_nuevo_destino(target_id)
            
        except ValueError:
            print(" Error: Introduce un número válido.")
        except Exception as e:
            break

def main(args=None):
    rclpy.init(args=args)
    node = ConductorInfinito()
    
    # Lanzamos el hilo del menú (para que input no bloquee a ROS)
    hilo_menu = threading.Thread(target=menu_interactivo, args=(node,))
    hilo_menu.start()
    
    # El hilo principal se queda procesando ROS
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    
    # Limpieza
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
