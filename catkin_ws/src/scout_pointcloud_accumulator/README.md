# scout_pointcloud_accumulator

Acumulador ROS para mapear con LiDAR y RealSense D435.

El flujo normal arranca:

- LiDAR
- RealSense D435
- LeGO-LOAM
- acumulador de nubes
- RViz con 4 visualizaciones

## Uso rapido

```bash
cd /media/agilex/0123-4567/ros/catkin_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
```

Arrancar todo:

```bash
./scripts/start_lidar_mapping.sh
```

Arrancar sin RViz:

```bash
RVIZ=false ./scripts/start_lidar_mapping.sh
```

Abrir RViz:

```bash
rviz -d /media/agilex/0123-4567/ros/catkin_ws/src/scout_pointcloud_accumulator/rviz/accum.rviz
```

Guardar mapa:

```bash
./scripts/save_accumulated_map.sh
```

Parar:

```bash
./scripts/stop_lidar_mapping.sh
```

## Visualizaciones en RViz

```text
/accumulated_lidar_points   LiDAR acumulado
/registered_cloud           LiDAR instantaneo
/accumulated_camera_points  RealSense acumulada, XYZRGB
/camera/colored_points      RealSense instantanea, XYZRGB
```

El `Fixed Frame` de RViz debe ser:

```text
map
```

## Archivos guardados

Al llamar al servicio de guardado se generan sólo los tres PCD finales y el manifest:

```text
*_lidar.pcd   LiDAR solo, PointXYZI
*_camera.pcd  RealSense aceptada por quality gates, PointXYZRGB
*_fused_quality.pcd  LiDAR + RealSense aceptada por quality gates, PointXYZRGB
*_manifest.json  snapshot_id, parametros, contadores y hashes de artefactos
```

Directorio por defecto:

```text
/media/agilex/0123-4567/ros/maps/map_YYYYMMDD_HHMMSS/
```

## Flags de guardado

Solo LiDAR:

```bash
SAVE_LIDAR=true SAVE_CAMERA=false ./scripts/start_lidar_mapping.sh
```

Solo RealSense:

```bash
SAVE_LIDAR=false SAVE_CAMERA=true ./scripts/start_lidar_mapping.sh
```

LiDAR + RealSense:

```bash
SAVE_LIDAR=true SAVE_CAMERA=true ./scripts/start_lidar_mapping.sh
```

## Comprobaciones rapidas

Topics:

```bash
rostopic hz /registered_cloud
rostopic hz /accumulated_lidar_points
rostopic hz /camera/colored_points
rostopic hz /accumulated_camera_points
```

TF:

```bash
rosrun tf tf_echo map camera_color_optical_frame
rosrun tf tf_echo map camera_init
```



Estabilidad roll/pitch de LeGO-LOAM y de la cadena TF usada por la camara:

```bash
./scripts/check_lego_roll_pitch.py --duration 60 --warn-deg 1.0
```

Debe devolver `OK` para `/aft_mapped_to_init`, `/integrated_to_init`, `map -> base_link` y `map -> camera_link`. Si roll/pitch crecen mientras el robot esta quieto, las nubes acumuladas volveran a formar abanico vertical.

Logs:

```bash
tail -f /media/agilex/0123-4567/ros/agv_mapping/logs/realsense.log
tail -f /media/agilex/0123-4567/ros/agv_mapping/logs/lego_loam.log
tail -f /media/agilex/0123-4567/ros/agv_mapping/logs/accumulator.log
```

## Parametros utiles

```bash
MAPPING_PROFILE=quality
CAMERA_VOXEL_SIZE=0.05
CAMERA_VISUALIZATION_VOXEL_SIZE=0.02
CAMERA_ACCUMULATE_RATE=1.0
CAMERA_XYZ="0.16 0.0 0.20"
CAMERA_RPY="0 0 0"

LEGO_USE_IMU=false
LEGO_LOCK_ROLL_PITCH=true
```

`CAMERA_XYZ` y `CAMERA_RPY` definen la TF `base_link -> camera_link`.
Los valores actuales son iniciales; para mapas metricos precisos hay que calibrar fisicamente la orientacion.

## Perfil quality

`start_lidar_mapping.sh` es el entrypoint canonico. El perfil `quality` es el
unico perfil soportado; el flujo anterior sin quality queda deprecated.

Valores por defecto de `quality`:

```text
REALSENSE_DEPTH_WIDTH/HEIGHT=848x480
REALSENSE_DEPTH_FPS=15
REALSENSE_COLOR_WIDTH/HEIGHT=848x480
REALSENSE_COLOR_FPS=15
REALSENSE_FILTERS=decimation,spatial
CAMERA_MAX_RANGE=3.0
CAMERA_OUTLIER_FILTER=sor
CAMERA_KEYFRAME_MIN_TRANSLATION=0.05
CAMERA_KEYFRAME_MIN_ROTATION_DEG=3.0
```

Estos valores pueden sobrescribirse por entorno cuando haga falta, por ejemplo:
sobrescribirse por entorno, por ejemplo:

```bash
CAMERA_OUTLIER_FILTER=none CAMERA_MAX_RANGE=2.5 ./scripts/start_lidar_mapping.sh
```

Filtros disponibles en el acumulador:

```text
CAMERA_OUTLIER_FILTER=none|sor|ror
CAMERA_SOR_MEAN_K=24
CAMERA_SOR_STDDEV_MUL=1.0
CAMERA_ROR_RADIUS=0.08
CAMERA_ROR_MIN_NEIGHBORS=3
CAMERA_MIN_POINTS=30
CAMERA_SYNC_TOLERANCE=0.03
```

`use_latest_tf_on_failure` sigue desactivado por defecto. Si un frame de camara
no tiene TF en su timestamp, no entra al mapa de calidad.

## Productos quality

La camara publica y guarda sólo frames aceptados por los quality gates en
`/accumulated_camera_points`. El mapa fusionado usa esa misma salida, de modo
que los PCD finales son `*_lidar.pcd`, `*_camera.pcd` y
`*_fused_quality.pcd`.

Cada guardado publica el manifiesto `*_manifest.json` al final. Si falta ese
manifiesto, trata el conjunto de PCDs como snapshot parcial.

## Captura B0 y evaluacion offline

Para comparar cambios de manera reproducible, captura bags con manifiesto:

```bash
SCENARIO=S4_recta_pared SPLIT=training DURATION_SEC=60 ./scripts/capture_mapping_dataset.sh
```

El manifiesto guarda commit, topicos, calibracion, hash del bag y configuracion
resuelta. Usa al menos tres capturas por escenario para estimar la
variabilidad antes de cambiar parametros del perfil quality.

Evaluar un PCD contra un plano congelado:

```bash
rosrun scout_pointcloud_accumulator mapping_quality_eval.py \
  /ruta/map_camera.pcd --sensor-origin camera --plane 0 0 1 -1 \
  --output-json /tmp/camera_eval.json --output-csv /tmp/camera_eval.csv
```

El evaluador reporta conteo finito, bbox, percentiles de rango y, si das un
plano, `plane_p95_m` y `plane_thickness_m`.

## Compilar

```bash
cd /media/agilex/0123-4567/ros/catkin_ws
source /opt/ros/melodic/setup.bash
catkin_make --pkg lego_loam scout_pointcloud_accumulator
source devel/setup.bash
```

Pruebas locales utiles:

```bash
bash tests/test_mapping_startup_health.sh
bash tests/test_mapping_quality_eval.sh
```
