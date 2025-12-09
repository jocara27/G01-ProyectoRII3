import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # --- MEJORA: DETECCIÓN AUTOMÁTICA DE LA CARPETA ---
    # Esto obtiene la carpeta donde está guardado este archivo .launch.py
    # Así funciona siempre, tengas la carpeta donde la tengas (G01-ProyectoRII3, etc.)
    current_dir = os.path.dirname(os.path.realpath(__file__))
    
    # 1. Tu usuario (para las rutas de Gazebo)
    USER_HOME = os.path.expanduser('~')
    
    # 2. Nombres de tus archivos (¡ASEGÚRATE DE QUE SE LLAMAN ASÍ!)
    # Si tu mundo se llama 'tablero_eurobot_v1.world', cámbialo aquí.
    ARCHIVO_MUNDO = 'tablero_eurobot_v3.world' 
    ARCHIVO_SCRIPT = 'conductor_interactivo.py'

    # -----------------------------------------------------

    # Rutas completas construidas automáticamente
    world_path = os.path.join(current_dir, ARCHIVO_MUNDO)
    script_path = os.path.join(current_dir, ARCHIVO_SCRIPT)

    # Comprobación de seguridad (opcional, para que sepas si falta el archivo)
    if not os.path.exists(world_path):
        print(f" ERROR CRÍTICO: No encuentro el mundo en: {world_path}")
    
    if not os.path.exists(script_path):
        print(f" ERROR CRÍTICO: No encuentro el script en: {script_path}")

    # 3. Configurar Variables de Entorno (Para que Gazebo encuentre el robot y el tablero)
    model_path = os.path.join(USER_HOME, '.gazebo', 'models')
    turtlebot_path = '/opt/ros/foxy/share/turtlebot3_gazebo/models'
    
    # Sumamos tus modelos + los de ROS
    new_model_path = f"{model_path}:{turtlebot_path}"

    env_variable = SetEnvironmentVariable('GAZEBO_MODEL_PATH', new_model_path)
    turtlebot_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')

    # 4. Comando para lanzar Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # 5. Comando para lanzar tu Script en una TERMINAL NUEVA
    # Usamos 'gnome-terminal' para que puedas ver el menú y escribir
    conductor_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'python3', script_path],
        output='screen'
    )

    return LaunchDescription([
        env_variable,      # Configura rutas modelos
        turtlebot_model,   # Configura modelo robot
        gazebo,            # Lanza Gazebo
        conductor_node     # Lanza tu script en ventana aparte
    ])
