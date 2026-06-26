# AGV-Mapping

Repositorio ROS (Melodic/catkin) para mapear un AGV Scout con LiDAR, RealSense D435, LeGO-LOAM, acumulacion de nubes PCD y registro de metadatos GPS/DOBACK.

## Estructura del repositorio

```text
AGV-Mapping/
  README.md                         Guia principal de uso.
  catkin_ws/                        Workspace catkin de ROS.
    scripts/                        Scripts de operacion diaria.
    src/                            Paquetes ROS y dependencias fuente.
      scout_pointcloud_accumulator/ Acumulador LiDAR/RealSense + metadatos.
      scout_base/                   Bringup, descripcion y mensajes del Scout.
      LeGO-LOAM/                    SLAM/odometria LiDAR.
      realsense/                    Driver ROS Intel RealSense.
      velodyne/                     Driver y transformaciones LiDAR Velodyne.
      navigation/                   gmapping, pointcloud_to_laserscan, navegacion.
      rf2o_laser_odometry/          Odometria 2D desde laser.
      ugv_sdk/                      SDK base AgileX/UGV.
  maps/                             Mapas PCD generados por las sesiones.
  datos/                            CSV/JSON de GPS, DOBACK y trayectoria.
  agv_mapping/                      PIDs y logs de la ejecucion normal.
  agv_mapping_test/                 PIDs/logs usados en pruebas.
```

> Nota: `catkin_ws/build/`, `catkin_ws/devel/` y `catkin_ws/logs/` son salidas de compilacion. Se regeneran con `catkin_make` y estan ignoradas por git.

## Preparar el entorno

Ejecutar siempre desde la raiz del repo o desde `catkin_ws`:

```bash
cd /home/agilex/Documents/PhDAlex/AGV-Mapping
source /opt/ros/melodic/setup.bash
cd catkin_ws
catkin_make
source devel/setup.bash
```

Si se abre una terminal nueva, repetir al menos:

```bash
source /opt/ros/melodic/setup.bash
source /home/agilex/Documents/PhDAlex/AGV-Mapping/catkin_ws/devel/setup.bash
```

## Ejecucion rapida: mapeo completo

El script principal arranca sin tmux:

- driver LiDAR (`scout_bringup open_rslidar.launch`),
- RealSense D435,
- LeGO-LOAM,
- acumulador de nubes,
- logger de metadatos,
- RViz si `RVIZ=true`.

```bash
cd /home/agilex/Documents/PhDAlex/AGV-Mapping/catkin_ws
./scripts/start_lidar_mapping.sh
```

Arrancar sin RViz:

```bash
RVIZ=false ./scripts/start_lidar_mapping.sh
```

Guardar el mapa durante la sesion:

```bash
./scripts/save_accumulated_map.sh
```

Parar todo guardando antes de cerrar:

```bash
./scripts/stop_lidar_mapping.sh
```

## Donde se guardan archivos

### Mapas PCD

Por defecto se guardan en `maps/` dentro del repo:

```text
maps/map_YYYYMMDD_HHMMSS_lidar.pcd   nube acumulada LiDAR
maps/map_YYYYMMDD_HHMMSS_camera.pcd  nube acumulada RealSense
maps/map_YYYYMMDD_HHMMSS_fused.pcd   nube LiDAR + RealSense
```

Tambien se puede cambiar la carpeta pasando un argumento al script:

```bash
./scripts/start_lidar_mapping.sh /ruta/a/mis_mapas
```

O fijando el archivo base:

```bash
PCD_FILE=/home/agilex/maps/prueba.pcd ./scripts/start_lidar_mapping.sh
```

### Metadatos GPS, DOBACK y trayectoria

Cada sesion crea una carpeta nueva en `datos/`:

```text
datos/metadata_YYYYMMDD_HHMMSS/
  gps.csv                         GPS parseado
  gps_raw.jsonl                   mensajes GPS crudos recibidos por TCP
  doback.csv                      DOBACK parseado
  doback_raw.csv                  lineas DOBACK crudas y errores de parseo
  trayectoria_gps_doback.csv      GPS + DOBACK + pose map/base_link combinados
  trayectoria_agv_mapa.csv        trayectoria del AGV en el mapa LiDAR
  manifest.json                   resumen de la sesion y rutas de salida
```

Tambien existen sesiones historicas con nombre `datos/sesion_YYYYMMDD_HHMMSS/`.

### Logs y PIDs

La ejecucion normal usa:

```text
agv_mapping/pids                  procesos lanzados por start_lidar_mapping.sh
agv_mapping/logs/lidar.log
agv_mapping/logs/realsense.log
agv_mapping/logs/lego_loam.log
agv_mapping/logs/accumulator.log
agv_mapping/logs/metadata.log
```

Ver logs en vivo:

```bash
tail -f ../agv_mapping/logs/lidar.log
tail -f ../agv_mapping/logs/realsense.log
tail -f ../agv_mapping/logs/lego_loam.log
tail -f ../agv_mapping/logs/accumulator.log
tail -f ../agv_mapping/logs/metadata.log
```

## Ejecutar cada parte por separado

Antes de usar `roslaunch`, preparar entorno:

```bash
source /opt/ros/melodic/setup.bash
source /home/agilex/Documents/PhDAlex/AGV-Mapping/catkin_ws/devel/setup.bash
```

### LiDAR Velodyne / Robosense

```bash
roslaunch scout_bringup open_rslidar.launch enable_rf2o:=false publish_robot_description:=false
```

Tópicos esperados:

```bash
rostopic list | grep -E 'velodyne|registered_cloud|scan'
```

### RealSense D435

```bash
roslaunch scout_pointcloud_accumulator realsense_mapping.launch \
  camera:=camera \
  depth_width:=640 depth_height:=480 \
  color_width:=640 color_height:=480 \
  depth_fps:=6 color_fps:=6
```

Ver la camara con herramienta grafica:

```bash
./catkin_ws/scripts/view_realsense.sh
```

### LeGO-LOAM

```bash
roslaunch lego_loam run.launch rviz:=false use_imu:=false lock_roll_pitch:=true
```

Comprobaciones utiles:

```bash
rosnode list | grep -E 'imageProjection|featureAssociation|mapOptmization|transformFusion|camera_init_to_map'
rosrun tf tf_echo map base_link
```

### Acumulador LiDAR/RealSense

Si los sensores y LeGO-LOAM ya estan arrancados:

```bash
roslaunch scout_pointcloud_accumulator accumulate.launch \
  lidar_topic:=/registered_cloud \
  enable_lidar:=true \
  enable_camera:=true \
  target_frame:=map \
  output_pcd:=/home/agilex/Documents/PhDAlex/AGV-Mapping/maps/manual.pcd \
  rviz:=true
```

Solo LiDAR:

```bash
ENABLE_CAMERA=false SAVE_CAMERA=false ./catkin_ws/scripts/start_lidar_mapping.sh
```

Solo RealSense:

```bash
ENABLE_LIDAR=false SAVE_LIDAR=false ./catkin_ws/scripts/start_lidar_mapping.sh
```

### Logger de metadatos GPS/DOBACK

```bash
roslaunch scout_pointcloud_accumulator mapping_metadata.launch \
  output_pcd:=/home/agilex/Documents/PhDAlex/AGV-Mapping/maps/manual.pcd \
  metadata_dir:=/home/agilex/Documents/PhDAlex/AGV-Mapping/datos/metadata_manual \
  gps_tcp_enable:=true \
  gps_tcp_port:=29500 \
  doback_enable:=false
```

Monitorizar la ultima sesion CSV:

```bash
./catkin_ws/scripts/monitor_metadata_csv.py
./catkin_ws/scripts/monitor_metadata_csv.py --watch 2
```

### Robot Scout base

Bringup minimo por CAN/USB segun configuracion del robot:

```bash
roslaunch scout_bringup scout_minimal.launch
# o, si aplica UART:
roslaunch scout_bringup scout_minimal_uart.launch
```

Teleoperacion por teclado:

```bash
roslaunch scout_bringup scout_teleop_keyboard.launch
```

Navegacion / gmapping incluidos en el repo:

```bash
roslaunch scout_bringup gmapping.launch
roslaunch scout_bringup navigation_4wd.launch
```

### Visualizacion RViz

RViz del acumulador:

```bash
rviz -d /home/agilex/Documents/PhDAlex/AGV-Mapping/catkin_ws/src/scout_pointcloud_accumulator/rviz/accum.rviz
```

El `Fixed Frame` debe ser `map`.

Tópicos principales:

```text
/registered_cloud             LiDAR instantaneo
/accumulated_lidar_points     LiDAR acumulado
/camera/colored_points        RealSense instantanea con color
/accumulated_camera_points    RealSense acumulada
/accumulated_points           nube fusionada
/agv_trajectory_path          trayectoria tipo nav_msgs/Path
/agv_trajectory_marker        trayectoria tipo Marker
```

Ver ultimo PCD con PCL:

```bash
./catkin_ws/scripts/view_latest_pcd.sh /home/agilex/Documents/PhDAlex/AGV-Mapping/maps
```

## LilyGO T-Echo / GPS por Bluetooth y TCP

### Probe BLE directo en PC

En el PC que tiene Bluetooth y Python 3:

```bash
python3 -m pip install bleak
python3 catkin_ws/scripts/lilygo_ble_probe.py --name LilyGO,T-Echo --listen-seconds 30 --output lilygo_ble_probe.jsonl
```

Si hay varios dispositivos, repetir con `--address <MAC_O_ID>`.

### Puente BLE -> AGV por TCP

El logger del AGV escucha por defecto en TCP `29500`. Desde el PC del LilyGO:

```bash
python3 catkin_ws/scripts/lilygo_ble_tcp_bridge.py \
  --address CE:BA:33:E1:3A:39 \
  --agv-host 100.123.78.14 \
  --agv-port 29500
```

Ajustar `--agv-host` a la IP real del Xavier/AGV. En el AGV se puede cambiar la lista de hosts permitidos con:

```bash
GPS_ALLOWED_HOSTS=IP_DEL_PC,127.0.0.1 ./catkin_ws/scripts/start_lidar_mapping.sh
```

## Variables utiles del script principal

Se pasan antes del comando:

```bash
RVIZ=false ./catkin_ws/scripts/start_lidar_mapping.sh
```

| Variable | Uso | Valor por defecto |
| --- | --- | --- |
| `OUTPUT_DIR` | Carpeta de mapas PCD | `maps/` |
| `PCD_FILE` | Archivo base del mapa | `maps/map_YYYYMMDD_HHMMSS.pcd` |
| `RUN_DIR` | Carpeta de PIDs/logs | `agv_mapping/` |
| `METADATA_DIR` | Carpeta CSV/JSON de la sesion | `datos/metadata_YYYYMMDD_HHMMSS/` |
| `RVIZ` | Abrir RViz automaticamente | `true` |
| `ENABLE_LIDAR` | Acumular LiDAR | `true` |
| `ENABLE_CAMERA` | Acumular RealSense | `true` |
| `SAVE_LIDAR` | Guardar PCD LiDAR | `true` |
| `SAVE_CAMERA` | Guardar PCD camara | `true` |
| `TARGET_FRAME` | Frame de acumulacion | `map` |
| `VOXEL_SIZE` | Voxel general | `0.05` |
| `CAMERA_MIN_RANGE` / `CAMERA_MAX_RANGE` | Rango valido RealSense | `0.20` / `5.0` |
| `GPS_TCP_PORT` | Puerto TCP de GPS | `29500` |
| `GPS_ALLOWED_HOSTS` | Hosts permitidos para GPS TCP | `100.93.178.118,127.0.0.1,::1` |
| `DOBACK_ENABLE` | Leer DOBACK serial | `false` |
| `DOBACK_PORT` | Puerto DOBACK | `auto` |
| `TRAJECTORY_PUBLISH_RATE` | Frecuencia trayectoria RViz | `2.0` |

## Comprobaciones y diagnostico

Estado de ROS:

```bash
rosnode list
rostopic list
rostopic hz /registered_cloud
rostopic echo -n 1 /agv_trajectory_path
rosrun tf tf_echo map base_link
```

Guardar manualmente desde ROS:

```bash
rosservice call /accumulator_node/save_accumulated "{}"
rosservice call /mapping_metadata_logger/save_metadata "{}"
```

Si RViz dice que no existe `map`, revisar LeGO-LOAM:

```bash
tail -n 100 /home/agilex/Documents/PhDAlex/AGV-Mapping/agv_mapping/logs/lego_loam.log
rosnode list | grep camera_init_to_map
```

Si no aparecen CSV, revisar el logger:

```bash
tail -n 100 /home/agilex/Documents/PhDAlex/AGV-Mapping/agv_mapping/logs/metadata.log
ls -lt /home/agilex/Documents/PhDAlex/AGV-Mapping/datos
```

## Desarrollo

Recompilar despues de cambiar C++ o mensajes:

```bash
cd /home/agilex/Documents/PhDAlex/AGV-Mapping/catkin_ws
source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash
```

Los scripts Python instalables estan en:

```text
catkin_ws/src/scout_pointcloud_accumulator/scripts/
catkin_ws/scripts/
```

El nodo C++ principal esta en:

```text
catkin_ws/src/scout_pointcloud_accumulator/src/accumulator.cpp
```
