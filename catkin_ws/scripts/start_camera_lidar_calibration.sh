#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-$(cd "$SCRIPT_DIR/.." && pwd)}"
CATKIN_WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_ROOT_DIR="$(cd "$WORKSPACE/.." && pwd)"
RUN_DIR="${CALIBRATION_RUN_DIR:-$ROS_ROOT_DIR/agv_calibration}"
LOG_DIR="${CALIBRATION_LOG_DIR:-$RUN_DIR/logs}"
PID_FILE="${CALIBRATION_PID_FILE:-$RUN_DIR/pids}"

CALIBRATION_DIR="${CALIBRATION_DIR:-$CATKIN_WS_DIR/calibration}"
CALIBRATION_FILE="${CALIBRATION_FILE:-$CALIBRATION_DIR/camera_lidar_calibration.yaml}"
if [ -d "$CALIBRATION_FILE" ]; then
  CALIBRATION_FILE="${CALIBRATION_FILE%/}/camera_lidar_calibration.yaml"
fi
CALIBRATION_PARENT_FRAME="${CALIBRATION_PARENT_FRAME:-base_link}"
CALIBRATION_CHILD_FRAME="${CALIBRATION_CHILD_FRAME:-camera_link}"
CALIBRATION_TARGET_FRAME="${CALIBRATION_TARGET_FRAME:-base_link}"
CALIBRATION_LIDAR_TOPIC="${CALIBRATION_LIDAR_TOPIC:-/velodyne_points}"
CALIBRATION_CAMERA_TOPIC="${CALIBRATION_CAMERA_TOPIC:-/camera/depth/color/points}"
CALIBRATION_XYZ="${CALIBRATION_XYZ:-0.16 0.0 0.20}"
CALIBRATION_RPY="${CALIBRATION_RPY:-0 0 0}"
CALIBRATION_STEP_TRANSLATION="${CALIBRATION_STEP_TRANSLATION:-0.01}"
CALIBRATION_STEP_ROTATION_DEG="${CALIBRATION_STEP_ROTATION_DEG:-0.5}"
RVIZ="${RVIZ:-true}"
CALIBRATION_LIDAR_FRAME="${CALIBRATION_LIDAR_FRAME:-velodyne}"
CALIBRATION_PUBLISH_LIDAR_TF="${CALIBRATION_PUBLISH_LIDAR_TF:-true}"
CALIBRATION_LIDAR_XYZ="${CALIBRATION_LIDAR_XYZ:-0 0 0}"
CALIBRATION_LIDAR_RPY="${CALIBRATION_LIDAR_RPY:-0 0 0}"

REALSENSE_DEPTH_WIDTH="${REALSENSE_DEPTH_WIDTH:-640}"
REALSENSE_DEPTH_HEIGHT="${REALSENSE_DEPTH_HEIGHT:-480}"
REALSENSE_COLOR_WIDTH="${REALSENSE_COLOR_WIDTH:-640}"
REALSENSE_COLOR_HEIGHT="${REALSENSE_COLOR_HEIGHT:-480}"
REALSENSE_DEPTH_FPS="${REALSENSE_DEPTH_FPS:-6}"
REALSENSE_COLOR_FPS="${REALSENSE_COLOR_FPS:-6}"
REALSENSE_INITIAL_RESET="${REALSENSE_INITIAL_RESET:-true}"

mkdir -p "$LOG_DIR" "$CALIBRATION_DIR" "$(dirname "$CALIBRATION_FILE")"
rm -f "$PID_FILE"

source /opt/ros/melodic/setup.bash
if [ -f "$WORKSPACE/devel/setup.bash" ]; then
  source "$WORKSPACE/devel/setup.bash"
fi

if rosnode list >/tmp/agv_calibration_rosnodes.$$ 2>/dev/null; then
  if grep -qx "/base_link_to_laser" /tmp/agv_calibration_rosnodes.$$ || grep -qx "/velodyne_nodelet_manager" /tmp/agv_calibration_rosnodes.$$ || grep -qx "/camera/realsense2_camera_manager" /tmp/agv_calibration_rosnodes.$$ || grep -qx "/camera/realsense2_camera" /tmp/agv_calibration_rosnodes.$$; then
    echo "ERROR: ya hay un LiDAR/RealSense/mapeo ROS en marcha."
    echo "Para evitar que RViz quede sin TF/nubes, para primero el mapeo/calibracion anterior:"
    echo "  ./scripts/stop_lidar_mapping.sh"
    echo "o cierra la calibracion anterior y repite este comando."
    rm -f /tmp/agv_calibration_rosnodes.$$
    exit 1
  fi
  rm -f /tmp/agv_calibration_rosnodes.$$
fi

start_process() {
  local name="$1"
  shift
  local log_file="$LOG_DIR/${name}.log"
  local command=""
  printf -v command "%q " "$@"
  nohup bash -lc "
    source /opt/ros/melodic/setup.bash
    if [ -f '$WORKSPACE/devel/setup.bash' ]; then source '$WORKSPACE/devel/setup.bash'; fi
    cd '$WORKSPACE'
    exec $command
  " > "$log_file" 2>&1 &
  local pid="$!"
  echo "$pid $name $log_file" >> "$PID_FILE"
  echo "Started $name: pid=$pid log=$log_file"
}

cleanup() {
  echo "Stopping calibration processes (no map/metadata save)."
  if [ -f "$PID_FILE" ]; then
    tac "$PID_FILE" | while read -r pid name _; do
      if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
        echo "Stopping $name pid=$pid"
        kill "$pid" 2>/dev/null || true
      fi
    done
    sleep 2
    tac "$PID_FILE" | while read -r pid name _; do
      if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
        echo "Force stopping $name pid=$pid"
        kill -9 "$pid" 2>/dev/null || true
      fi
    done
    rm -f "$PID_FILE"
  fi
}
trap cleanup EXIT INT TERM

start_process lidar roslaunch scout_bringup open_rslidar.launch \
  enable_laserscan:=false \
  enable_rf2o:=false \
  publish_robot_description:=false
sleep 2

# Publicador TF con nombre unico para que el viewer pueda transformar /velodyne_points.
# open_rslidar tambien publica base_link->velodyne, pero si ese nodo muere o cambia de nombre
# este fallback mantiene visible la nube LiDAR en calibracion.
if [ "$CALIBRATION_PUBLISH_LIDAR_TF" = "true" ]; then
  start_process lidar_tf rosrun tf static_transform_publisher \
    $CALIBRATION_LIDAR_XYZ $CALIBRATION_LIDAR_RPY \
    "$CALIBRATION_PARENT_FRAME" "$CALIBRATION_LIDAR_FRAME" 100
fi

start_process realsense roslaunch scout_pointcloud_accumulator realsense_mapping.launch \
  camera:=camera \
  enable_pointcloud:=true \
  depth_width:="$REALSENSE_DEPTH_WIDTH" \
  depth_height:="$REALSENSE_DEPTH_HEIGHT" \
  color_width:="$REALSENSE_COLOR_WIDTH" \
  color_height:="$REALSENSE_COLOR_HEIGHT" \
  depth_fps:="$REALSENSE_DEPTH_FPS" \
  color_fps:="$REALSENSE_COLOR_FPS" \
  initial_reset:="$REALSENSE_INITIAL_RESET"
sleep 2

start_process viewer rosrun scout_pointcloud_accumulator camera_lidar_calibration_viewer.py \
  _target_frame:="$CALIBRATION_TARGET_FRAME" \
  _lidar_topic:="$CALIBRATION_LIDAR_TOPIC" \
  _camera_topic:="$CALIBRATION_CAMERA_TOPIC"

if [ "$RVIZ" = "true" ]; then
  start_process rviz rviz -d "$WORKSPACE/src/scout_pointcloud_accumulator/rviz/calibration.rviz"
fi

cat <<EOF

Camera/LiDAR calibration mode started.

Non-persistent mode:
  - no accumulator_node
  - no metadata logger
  - no automatic PCD save

Calibration file:
  $CALIBRATION_FILE

Topics expected:
  $CALIBRATION_LIDAR_TOPIC -> /calibration/lidar_points
  LiDAR frame: $CALIBRATION_LIDAR_FRAME, target frame: $CALIBRATION_TARGET_FRAME
  $CALIBRATION_CAMERA_TOPIC -> /calibration/camera_points

Foreground calibrator commands: x+/x-/y+/y-/z+/z-/roll+/pitch+/yaw+/step/rstep/show/save/quit
EOF

rosrun scout_pointcloud_accumulator camera_lidar_calibrator.py \
  _calibration_file:="$CALIBRATION_FILE" \
  _parent_frame:="$CALIBRATION_PARENT_FRAME" \
  _child_frame:="$CALIBRATION_CHILD_FRAME" \
  _xyz:="$CALIBRATION_XYZ" \
  _rpy:="$CALIBRATION_RPY" \
  _step:="$CALIBRATION_STEP_TRANSLATION" \
  _rstep_deg:="$CALIBRATION_STEP_ROTATION_DEG"
