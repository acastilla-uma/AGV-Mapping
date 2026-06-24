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

Al llamar al servicio de guardado se generan:

```text
*_lidar.pcd   LiDAR solo, PointXYZI
*_camera.pcd  RealSense sola, PointXYZRGB
*_fused.pcd   LiDAR + RealSense, PointXYZRGB
*_doback_raw.csv
*_doback_stability.csv
*_gps.csv
*_map_track.csv
*_session_manifest.json
```

Directorio por defecto:

```text
/media/agilex/0123-4567/ros/maps
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

## GPS LilyGO + DOBACK

El logger de metadatos corre en el AGV junto al acumulador. Recibe:

- DOBACK por serie local del AGV, formato `esp_datareceiver`:
  ```text
  ax;ay;az;gx;gy;gz;roll;pitch;yaw;timeantwifi;usciclo1;usciclo2;usciclo3;usciclo4;usciclo5;si;accmag;microsds;k3
  ```
- GPS LilyGO T-Echo por TCP desde el PC hacia el AGV.

Arranque en el AGV:

```bash
cd /media/agilex/0123-4567/ros/catkin_ws
DOBACK_ENABLE=true DOBACK_PORT=/dev/ttyACM0 GPS_TCP_ENABLE=true GPS_TCP_PORT=29500 ./scripts/start_lidar_mapping.sh
```

Bridge en el PC conectado por Bluetooth al LilyGO:

```bash
python scripts/gps_lilygo_tcp_bridge.py --serial-port COM5 --agv-host 100.123.78.14 --agv-port 29500
```

En Linux el puerto Bluetooth suele ser algo como `/dev/rfcomm0`:

```bash
python /media/agilex/0123-4567/ros/catkin_ws/scripts/gps_lilygo_tcp_bridge.py --serial-port /dev/rfcomm0 --agv-host 100.123.78.14
```

El GPS TCP acepta JSON, CSV simple o NMEA `GGA/RMC`. Ejemplos validos:

```text
{"lat":36.716,"lon":-4.478,"alt":45.2,"sats":10,"hdop":0.9}
36.716,-4.478,45.2,fix,10,0.9
$GNGGA,...
```

El archivo principal para postproceso es:

```text
*_map_track.csv
```

Incluye `map_x,map_y,map_yaw`, GPS cercano, estabilidad DOBACK cercana y flags `tf_ok,gps_ok,doback_ok`.

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

## Compilar

```bash
cd /media/agilex/0123-4567/ros/catkin_ws
source /opt/ros/melodic/setup.bash
catkin_make --pkg lego_loam scout_pointcloud_accumulator
source devel/setup.bash
```
