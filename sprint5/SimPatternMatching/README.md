---

# Instrucciones de ejecución – Sprint 5 (Pattern Matching)

## 1. Lanzar el Mundo (Modelos)

Primero tenemos que lanzar nuestro mundo simulado con Gazebo para ejecutar la demostración donde se corre el script de Python realizado (`patternmatching.py`).

> **IMPORTANTE:** Para que la simulación cargue correctamente las texturas del tablero y el robot modificado, es necesario copiar los modelos personalizados al directorio de Gazebo.

Ejecuta estos comandos para copiar las carpetas incluidas en este repositorio a tu carpeta de modelos:

```bash
# Asumiendo que estás en la carpeta del proyecto
cp -r Building_Tablero ~/.gazebo/models/
cp -r Building_Tablerov1 ~/.gazebo/models/
cp -r turtlebot3_waffle1 ~/.gazebo/models/

```

Una vez tenemos esto en cuenta, ejecutamos el mundo:

```bash
gazebo tablero_eurobot_v3.world 

```

---

## 2. Demostración Funcional

Para ejecutar la demostración donde se ejecuta el script de Python realizado:

```bash
python3 patternmatching.py

```

**Comportamiento esperado:**

* Se abrirá una ventana con la imagen que ofrece la cámara del simulador.
* Se podrán ver los almacenes marcados con un indicador.
* Se informará a través de la terminal de las coordenadas de los almacenes detectados.

## 3. Publicar en el topic

Para visualizar la información de los almacenes publicada en el topic:

```bash
ros2 topic echo /almacenes_detectados

```
