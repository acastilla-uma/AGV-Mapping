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
cd /mnt/ros/catkin_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
```

Arrancar todo:

```bash
/mnt/ros/catkin_ws/scripts/start_lidar_mapping.sh
```

Arrancar sin RViz:

```bash
RVIZ=false /mnt/ros/catkin_ws/scripts/start_lidar_mapping.sh
```

Abrir RViz:

```bash
rviz -d /mnt/ros/catkin_ws/src/scout_pointcloud_accumulator/rviz/accum.rviz
```

Guardar mapa:

```bash
/mnt/ros/catkin_ws/scripts/save_accumulated_map.sh
```

Parar:

```bash
/mnt/ros/catkin_ws/scripts/stop_lidar_mapping.sh
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
```

Directorio por defecto:

```text
/mnt/ros/maps
```

## Flags de guardado

Solo LiDAR:

```bash
SAVE_LIDAR=true SAVE_CAMERA=false /mnt/ros/catkin_ws/scripts/start_lidar_mapping.sh
```

Solo RealSense:

```bash
SAVE_LIDAR=false SAVE_CAMERA=true /mnt/ros/catkin_ws/scripts/start_lidar_mapping.sh
```

LiDAR + RealSense:

```bash
SAVE_LIDAR=true SAVE_CAMERA=true /mnt/ros/catkin_ws/scripts/start_lidar_mapping.sh
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
/mnt/ros/catkin_ws/scripts/check_lego_roll_pitch.py --duration 60 --warn-deg 1.0
```

Debe devolver `OK` para `/aft_mapped_to_init`, `/integrated_to_init`, `map -> base_link` y `map -> camera_link`. Si roll/pitch crecen mientras el robot esta quieto, las nubes acumuladas volveran a formar abanico vertical.

Logs:

```bash
tail -f /mnt/ros/agv_mapping/logs/realsense.log
tail -f /mnt/ros/agv_mapping/logs/lego_loam.log
tail -f /mnt/ros/agv_mapping/logs/accumulator.log
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
cd /mnt/ros/catkin_ws
source /opt/ros/melodic/setup.bash
catkin_make --pkg lego_loam scout_pointcloud_accumulator
source devel/setup.bash
```
