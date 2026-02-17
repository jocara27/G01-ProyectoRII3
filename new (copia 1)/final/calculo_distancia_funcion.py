import math

def calcular_distancia_pixeles(posicion_robot, x_obj, y_obj):
    """
    MISMA lógica que tu script de distancia.
    Devuelve la distancia en píxeles.
    """
    dx = x_obj - posicion_robot[0]
    dy = y_obj - posicion_robot[1]
    return math.hypot(dx, dy)

