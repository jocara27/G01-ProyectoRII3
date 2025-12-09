


# Instrucciones de ejecución – g04_prii3_sprint4 (Eurobot 2026)

## 1\. Configuración Previa (Modelos)

````markdown
**IMPORTANTE:** Para que la simulación cargue correctamente las texturas del tablero y el robot modificado, es
 necesario copiar los modelos personalizados al directorio de Gazebo.

Ejecuta estos comandos para copiar las carpetas incluidas en este repositorio a tu carpeta de modelos:

```bash
# Asumiendo que estás en la carpeta del proyecto
cp -r Building_Tablero ~/.gazebo/models/
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

<!-- end list -->

```
```
