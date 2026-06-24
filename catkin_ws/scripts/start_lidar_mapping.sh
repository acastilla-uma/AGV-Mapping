#!/usr/bin/env bash
set -euo pipefail

# Starts the LiDAR driver, RealSense D435, LEGO-LOAM, and the accumulated cloud
# saver without tmux. Processes are detached with nohup and their PIDs are stored
# for stopping.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-$(cd "$SCRIPT_DIR/.." && pwd)}"
ROS_ROOT_DIR="$(cd "$WORKSPACE/.." && pwd)"
OUTPUT_DIR="${1:-${OUTPUT_DIR:-$ROS_ROOT_DIR/maps}}"
RUN_DIR="${RUN_DIR:-$ROS_ROOT_DIR/agv_mapping}"
LOG_DIR="${LOG_DIR:-$RUN_DIR/logs}"
PID_FILE="${PID_FILE:-$RUN_DIR/pids}"

LIDAR_TOPIC="${LIDAR_TOPIC:-${INPUT_TOPIC:-/registered_cloud}}"
CAMERA_TOPIC="${CAMERA_TOPIC:-/camera/depth/color/points}"
CAMERA_DEPTH_TOPIC="${CAMERA_DEPTH_TOPIC:-/camera/aligned_depth_to_color/image_raw}"
USE_ALIGNED_DEPTH_FOR_CAMERA="${USE_ALIGNED_DEPTH_FOR_CAMERA:-true}"
CAMERA_COLOR_TOPIC="${CAMERA_COLOR_TOPIC:-/camera/color/image_raw}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera/color/camera_info}"
ENABLE_CAMERA_COLOR="${ENABLE_CAMERA_COLOR:-true}"
TARGET_FRAME="${TARGET_FRAME:-map}"
VOXEL_SIZE="${VOXEL_SIZE:-0.05}"
LIDAR_VOXEL_SIZE="${LIDAR_VOXEL_SIZE:-0.05}"
CAMERA_VOXEL_SIZE="${CAMERA_VOXEL_SIZE:-0.05}"
CAMERA_VISUALIZATION_VOXEL_SIZE="${CAMERA_VISUALIZATION_VOXEL_SIZE:-0.02}"
CAMERA_ACCUMULATE_RATE="${CAMERA_ACCUMULATE_RATE:-1.0}"
CAMERA_VISUALIZATION_RATE="${CAMERA_VISUALIZATION_RATE:-5.0}"
CAMERA_MIN_RANGE="${CAMERA_MIN_RANGE:-0.20}"
CAMERA_MAX_RANGE="${CAMERA_MAX_RANGE:-5.0}"
CAMERA_DEPTH_PIXEL_STEP="${CAMERA_DEPTH_PIXEL_STEP:-2}"
PCD_FILE="${PCD_FILE:-$OUTPUT_DIR/map_$(date +%Y%m%d_%H%M%S).pcd}"
RVIZ="${RVIZ:-true}"

ENABLE_LIDAR="${ENABLE_LIDAR:-true}"
ENABLE_CAMERA="${ENABLE_CAMERA:-true}"
SAVE_LIDAR="${SAVE_LIDAR:-true}"
SAVE_CAMERA="${SAVE_CAMERA:-true}"
CAMERA_NAME="${CAMERA_NAME:-camera}"
CAMERA_PARENT_FRAME="${CAMERA_PARENT_FRAME:-base_link}"
CAMERA_CHILD_FRAME="${CAMERA_CHILD_FRAME:-camera_link}"
CAMERA_XYZ="${CAMERA_XYZ:-0.16 0.0 0.20}"
CAMERA_RPY="${CAMERA_RPY:-0 0 0}"
CAMERA_INTENSITY="${CAMERA_INTENSITY:-0.0}"
TRANSFORM_TIMEOUT="${TRANSFORM_TIMEOUT:-0.5}"
USE_LATEST_TF_ON_FAILURE="${USE_LATEST_TF_ON_FAILURE:-false}"
LEGO_USE_IMU="${LEGO_USE_IMU:-false}"
LEGO_LOCK_ROLL_PITCH="${LEGO_LOCK_ROLL_PITCH:-true}"

ENABLE_METADATA_LOGGER="${ENABLE_METADATA_LOGGER:-true}"
DOBACK_ENABLE="${DOBACK_ENABLE:-false}"
DOBACK_REQUIRED="${DOBACK_REQUIRED:-false}"
DOBACK_PORT="${DOBACK_PORT:-/dev/ttyACM0}"
DOBACK_BAUD="${DOBACK_BAUD:-115200}"
GPS_TCP_ENABLE="${GPS_TCP_ENABLE:-true}"
GPS_TCP_BIND="${GPS_TCP_BIND:-0.0.0.0}"
GPS_TCP_PORT="${GPS_TCP_PORT:-29500}"
GPS_ALLOWED_HOSTS="${GPS_ALLOWED_HOSTS:-100.93.178.118,127.0.0.1,::1}"
GPS_REQUIRED="${GPS_REQUIRED:-false}"
METADATA_ROBOT_FRAME="${METADATA_ROBOT_FRAME:-base_link}"
METADATA_JOIN_SLOP_SEC="${METADATA_JOIN_SLOP_SEC:-2.0}"

REALSENSE_DEPTH_WIDTH="${REALSENSE_DEPTH_WIDTH:-640}"
REALSENSE_DEPTH_HEIGHT="${REALSENSE_DEPTH_HEIGHT:-480}"
REALSENSE_COLOR_WIDTH="${REALSENSE_COLOR_WIDTH:-640}"
REALSENSE_COLOR_HEIGHT="${REALSENSE_COLOR_HEIGHT:-480}"
REALSENSE_DEPTH_FPS="${REALSENSE_DEPTH_FPS:-6}"
REALSENSE_COLOR_FPS="${REALSENSE_COLOR_FPS:-6}"
REALSENSE_FILTERS="${REALSENSE_FILTERS:-}"
REALSENSE_CLIP_DISTANCE="${REALSENSE_CLIP_DISTANCE:--1}"
REALSENSE_POINTCLOUD_TEXTURE_STREAM="${REALSENSE_POINTCLOUD_TEXTURE_STREAM:-RS2_STREAM_COLOR}"
REALSENSE_POINTCLOUD_TEXTURE_INDEX="${REALSENSE_POINTCLOUD_TEXTURE_INDEX:-0}"
REALSENSE_ALLOW_NO_TEXTURE_POINTS="${REALSENSE_ALLOW_NO_TEXTURE_POINTS:-false}"
REALSENSE_INITIAL_RESET="${REALSENSE_INITIAL_RESET:-true}"

mkdir -p "$OUTPUT_DIR" "$LOG_DIR"

if [ -f "$PID_FILE" ]; then
  running=false
  while read -r pid _; do
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      running=true
      break
    fi
  done < "$PID_FILE"

  if [ "$running" = true ]; then
    echo "Mapping already seems to be running."
    echo "PID file: $PID_FILE"
    echo "Stop it with: $WORKSPACE/scripts/stop_lidar_mapping.sh"
    exit 1
  fi
fi

rm -f "$PID_FILE"

start_process() {
  local name="$1"
  shift
  local log_file="$LOG_DIR/${name}.log"
  local command=""

  printf -v command "%q " "$@"

  nohup bash -lc "
    source /opt/ros/melodic/setup.bash
    if [ -f \"$WORKSPACE/devel/setup.bash\" ]; then
      source \"$WORKSPACE/devel/setup.bash\"
    fi
    cd \"$WORKSPACE\"
    exec $command
  " > "$log_file" 2>&1 &

  local pid="$!"
  echo "$pid $name $log_file" >> "$PID_FILE"
  echo "Started $name: pid=$pid log=$log_file"
}

wait_for_ros_node() {
  local node_name="$1"
  local timeout_sec="$2"
  local elapsed=0

  while [ "$elapsed" -lt "$timeout_sec" ]; do
    if bash -lc "source /opt/ros/melodic/setup.bash; source '$WORKSPACE/devel/setup.bash'; rosnode list 2>/dev/null | grep -qx '$node_name'"; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done

  return 1
}

start_process lidar roslaunch scout_bringup open_rslidar.launch \
  enable_rf2o:=false \
  publish_robot_description:=false
sleep 2
start_process realsense roslaunch scout_pointcloud_accumulator realsense_mapping.launch \
  camera:="$CAMERA_NAME" \
  depth_width:="$REALSENSE_DEPTH_WIDTH" \
  depth_height:="$REALSENSE_DEPTH_HEIGHT" \
  color_width:="$REALSENSE_COLOR_WIDTH" \
  color_height:="$REALSENSE_COLOR_HEIGHT" \
  depth_fps:="$REALSENSE_DEPTH_FPS" \
  color_fps:="$REALSENSE_COLOR_FPS" \
  filters:="$REALSENSE_FILTERS" \
  clip_distance:="$REALSENSE_CLIP_DISTANCE" \
  pointcloud_texture_stream:="$REALSENSE_POINTCLOUD_TEXTURE_STREAM" \
  pointcloud_texture_index:="$REALSENSE_POINTCLOUD_TEXTURE_INDEX" \
  allow_no_texture_points:="$REALSENSE_ALLOW_NO_TEXTURE_POINTS" \
  initial_reset:="$REALSENSE_INITIAL_RESET"
sleep 2
start_process lego_loam roslaunch lego_loam run.launch rviz:=false use_imu:="$LEGO_USE_IMU" lock_roll_pitch:="$LEGO_LOCK_ROLL_PITCH"
if ! wait_for_ros_node /camera_init_to_map 20; then
  echo "WARNING: LeGO-LOAM did not publish /camera_init_to_map after 20 seconds."
  echo "RViz may show: Fixed Frame [map] does not exist. Check: $LOG_DIR/lego_loam.log"
fi
sleep 1
start_process accumulator roslaunch scout_pointcloud_accumulator accumulate.launch \
  lidar_topic:="$LIDAR_TOPIC" \
  camera_topic:="$CAMERA_TOPIC" \
  camera_depth_topic:="$CAMERA_DEPTH_TOPIC" \
  use_aligned_depth_for_camera:="$USE_ALIGNED_DEPTH_FOR_CAMERA" \
  camera_color_topic:="$CAMERA_COLOR_TOPIC" \
  camera_info_topic:="$CAMERA_INFO_TOPIC" \
  enable_camera_color:="$ENABLE_CAMERA_COLOR" \
  enable_lidar:="$ENABLE_LIDAR" \
  enable_camera:="$ENABLE_CAMERA" \
  target_frame:="$TARGET_FRAME" \
  voxel_size:="$VOXEL_SIZE" \
  lidar_voxel_size:="$LIDAR_VOXEL_SIZE" \
  camera_voxel_size:="$CAMERA_VOXEL_SIZE" \
  camera_visualization_voxel_size:="$CAMERA_VISUALIZATION_VOXEL_SIZE" \
  camera_accumulate_rate:="$CAMERA_ACCUMULATE_RATE" \
  camera_visualization_rate:="$CAMERA_VISUALIZATION_RATE" \
  camera_min_range:="$CAMERA_MIN_RANGE" \
  camera_max_range:="$CAMERA_MAX_RANGE" \
  camera_depth_pixel_step:="$CAMERA_DEPTH_PIXEL_STEP" \
  camera_intensity:="$CAMERA_INTENSITY" \
  transform_timeout:="$TRANSFORM_TIMEOUT" \
  use_latest_tf_on_failure:="$USE_LATEST_TF_ON_FAILURE" \
  output_pcd:="$PCD_FILE" \
  save_lidar:="$SAVE_LIDAR" \
  save_camera:="$SAVE_CAMERA" \
  camera_parent_frame:="$CAMERA_PARENT_FRAME" \
  camera_child_frame:="$CAMERA_CHILD_FRAME" \
  camera_xyz:="$CAMERA_XYZ" \
  camera_rpy:="$CAMERA_RPY" \
  rviz:="$RVIZ"

if [ "$ENABLE_METADATA_LOGGER" = "true" ]; then
  start_process metadata roslaunch scout_pointcloud_accumulator mapping_metadata.launch \
    output_pcd:="$PCD_FILE" \
    target_frame:="$TARGET_FRAME" \
    robot_frame:="$METADATA_ROBOT_FRAME" \
    doback_enable:="$DOBACK_ENABLE" \
    doback_required:="$DOBACK_REQUIRED" \
    doback_port:="$DOBACK_PORT" \
    doback_baud:="$DOBACK_BAUD" \
    gps_tcp_enable:="$GPS_TCP_ENABLE" \
    gps_tcp_bind:="$GPS_TCP_BIND" \
    gps_tcp_port:="$GPS_TCP_PORT" \
    gps_allowed_hosts:="$GPS_ALLOWED_HOSTS" \
    gps_required:="$GPS_REQUIRED" \
    join_slop_sec:="$METADATA_JOIN_SLOP_SEC"
  if ! wait_for_ros_node /mapping_metadata_logger 10; then
    echo "WARNING: mapping_metadata_logger did not stay alive after launch."
    echo "Metadata CSVs may be missing. Check: $LOG_DIR/metadata.log"
  fi
fi

if ! wait_for_ros_node /accumulator_node 15; then
  echo "ERROR: accumulator_node did not stay alive after launch."
  echo "Check: $LOG_DIR/accumulator.log"
  tail -n 80 "$LOG_DIR/accumulator.log" 2>/dev/null || true
  exit 1
fi

cat <<EOF

Mapping started without tmux.

Logs:
  $LOG_DIR

Accumulated PCD outputs:
  ${PCD_FILE%.pcd}_lidar.pcd
  ${PCD_FILE%.pcd}_camera.pcd
  ${PCD_FILE%.pcd}_fused.pcd

Metadata outputs:
  ${PCD_FILE%.pcd}_doback_raw.csv
  ${PCD_FILE%.pcd}_doback_stability.csv
  ${PCD_FILE%.pcd}_gps.csv
  ${PCD_FILE%.pcd}_map_track.csv
  ${PCD_FILE%.pcd}_session_manifest.json

PC LilyGO bridge:
  python $WORKSPACE/scripts/gps_lilygo_tcp_bridge.py --serial-port COM5 --agv-host 100.123.78.14 --agv-port $GPS_TCP_PORT

Save at any time:
  $WORKSPACE/scripts/save_accumulated_map.sh

Stop everything:
  $WORKSPACE/scripts/stop_lidar_mapping.sh

Watch logs:
  tail -f $LOG_DIR/realsense.log
  tail -f $LOG_DIR/lego_loam.log
  tail -f $LOG_DIR/accumulator.log
EOF
