

````markdown
# Instrucciones de ejecución – g04_prii3_sprint4 (Eurobot 2026)

-----

## 1\. Configuración Previa (Modelos)

**IMPORTANTE:** Para que la simulación cargue correctamente las texturas del tablero y el robot modificado, es necesario copiar los modelos personalizados al directorio de Gazebo.

Ejecuta estos comandos para copiar las carpetas incluidas en este repositorio a tu carpeta de modelos:

```bash
# Asumiendo que estás en la carpeta del proyecto
cp -r Building_Tablerov1 ~/.gazebo/models/
cp -r turtlebot3_waffle1 ~/.gazebo/models/
````

-----

## 2\. Cargar el entorno

Abre una terminal y carga las variables de entorno necesarias (configurando el modelo `waffle` para el robot):

```bash
source /opt/ros/foxy/setup.bash
```

```bash
export TURTLEBOT3_MODEL=waffle
```

-----

## 3\. Demostración Funcional (Launch Único)

Para ejecutar la demostración completa donde se lanzan simultáneamente:

1.  **Gazebo:** Con el mundo `sprint4_completo.world` (Tablero Eurobot + Cámara Cenital).
2.  **Script Interactivo:** Nodo de control visual que abre un menú en una terminal separada.

Ejecuta este único comando desde la carpeta del proyecto:

```bash
ros2 launch sprint4.launch.py
```

**Comportamiento esperado:**

  - Se abrirá **Gazebo** con el robot en la zona de salida.
  - Se abrirá automáticamente una **segunda terminal negra** con el menú de control.
  - El sistema esperará a que introduzcas un **ID de destino** (20, 21, 22, 23).
  - El robot navegará usando **visión artificial** (sin mapa ni odometría) hasta el marcador.

-----

## 4\. Ejecución Paso a Paso (Opcional)

Si deseas ejecutar los componentes por separado para depuración:

### Paso 1: Lanzar Simulación (Mundo)

```bash
gazebo --verbose sprint4_completo.world
```

### Paso 2: Lanzar Conductor Interactivo

En otra terminal (recuerda exportar el modelo waffle):

```bash
python3 conductor_infinito.py
```

-----

## 5\. Descripción de Ficheros Clave

### Mundos y Modelos

  - **`sprint4_completo.world`**: Escenario final que incluye:
      - Tablero Eurobot 2026 con dimensiones y texturas oficiales ("Winter is Coming").
      - **Cámara Cenital**: Situada a 3 metros de altura para visión global.
      - **Marcadores ArUco**: Dispuestos en el suelo (IDs 20, 21, 22, 23) y zonas de salida.
  - **`Building_Tablerov1`**: Modelo SDF del tablero con colisiones optimizadas y texturas corregidas.

### Scripts de Navegación Visual

  - **`conductor_infinito.py`**: Nodo principal de navegación.
      - **Lógica**: Utiliza *Visual Servoing*. Calcula el vector de error entre la orientación del robot (ID 5) y el destino seleccionado.
      - **Control**: Implementa un controlador proporcional (P) para velocidad lineal y angular.
      - **Memoria de Cercanía**: Soluciona el problema de solapamiento visual al llegar al destino, recordando la última distancia válida para confirmar la llegada ("META ALCANZADA").
      - **Interfaz**: Permite enviar al robot a múltiples destinos secuencialmente sin reiniciar el nodo.

-----

## 6\. IDs de los Marcadores

Para interactuar con el menú, utiliza estos códigos:

| ID | Ubicación |
| :--- | :--- |
| **5** | **Robot (Posición Actual)** |
| **20** | Zona Inferior Izquierda |
| **21** | Zona Superior Izquierda |
| **22** | Zona Inferior Derecha |
| **23** | Zona Superior Derecha |

-----

## 7\. Finalizar la ejecución

Para detener la simulación y cerrar todos los procesos:

1.  Escribe `q` en la terminal del menú interactivo.
2.  Presiona `Ctrl + C` en la terminal principal.

<!-- end list -->

```
```
