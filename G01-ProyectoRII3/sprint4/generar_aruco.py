import cv2
import numpy as np

# 1. Obtener el diccionario (Usando la función nueva que vimos antes)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# 2. Generar el marcador ID 5
# CAMBIO: En OpenCV nuevo, 'drawMarker' se llama 'generateImageMarker'
img = cv2.aruco.generateImageMarker(aruco_dict, 1, 300)

# 3. Añadir borde blanco (IMPORTANTE para que Gazebo lo vea bien)
img_con_borde = cv2.copyMakeBorder(
    img, 
    50, 50, 50, 50, # 50 píxeles de borde a cada lado
    cv2.BORDER_CONSTANT, 
    value=[255, 255, 255] # Color blanco
)

# 4. Guardar
cv2.imwrite("marker1.png", img_con_borde)
print("¡ArUco generado correctamente como 'marker5.png'!")
