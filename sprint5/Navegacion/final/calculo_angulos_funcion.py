import math

def calcular_angulo_giro(posicion_robot, theta_actual, x_obj, y_obj, offset_theta=90.0, signo=1):
    """
    MISMA lógica que tu script de cálculo de ángulos.
    Devuelve el ángulo de giro normalizado a [-180, 180].
    """
    dx = x_obj - posicion_robot[0]
    dy = y_obj - posicion_robot[1]

    theta_obj = math.degrees(math.atan2(dy, dx))
    theta_obj = signo * theta_obj + offset_theta

    diff = theta_obj - theta_actual
    angulo_giro = (diff + 180) % 360 - 180

    return angulo_giro

