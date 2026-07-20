#!/usr/bin/env bash
set -euo pipefail

# Starts the LiDAR driver, RealSense D435, LEGO-LOAM, and the accumulated cloud
# saver without tmux. Processes are detached with nohup and their PIDs are stored
# for stopping.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/mapping_startup_health.sh"
WORKSPACE="${WORKSPACE:-$(cd "$SCRIPT_DIR/.." && pwd)}"
CATKIN_WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_ROOT_DIR="$(cd "$WORKSPACE/.." && pwd)"
OUTPUT_DIR="${1:-${OUTPUT_DIR:-$ROS_ROOT_DIR/maps}}"
RUN_DIR="${RUN_DIR:-$ROS_ROOT_DIR/agv_mapping}"
LOG_DIR="${LOG_DIR:-$RUN_DIR/logs}"
PID_FILE="${PID_FILE:-$RUN_DIR/pids}"
SESSION_NAME="${SESSION_NAME:-map_$(date +%Y%m%d_%H%M%S)}"
SESSION_DIR="${SESSION_DIR:-$OUTPUT_DIR/$SESSION_NAME}"

MAPPING_PROFILE="${MAPPING_PROFILE:-quality}"
case "$MAPPING_PROFILE" in
  quality) ;;
  *)
    echo "ERROR: only MAPPING_PROFILE=quality is supported (got '$MAPPING_PROFILE')." >&2
    exit 1
    ;;
esac

DEFAULT_REALSENSE_DEPTH_WIDTH=848
DEFAULT_REALSENSE_DEPTH_HEIGHT=480
DEFAULT_REALSENSE_COLOR_WIDTH=848
DEFAULT_REALSENSE_COLOR_HEIGHT=480
DEFAULT_REALSENSE_DEPTH_FPS=15
DEFAULT_REALSENSE_COLOR_FPS=15
DEFAULT_REALSENSE_FILTERS="decimation,spatial"
DEFAULT_CAMERA_MAX_RANGE=3.0
DEFAULT_CAMERA_OUTLIER_FILTER=sor
DEFAULT_CAMERA_KEYFRAME_MIN_TRANSLATION=0.05
DEFAULT_CAMERA_KEYFRAME_MIN_ROTATION_DEG=3.0

LIDAR_TOPIC="${LIDAR_TOPIC:-${INPUT_TOPIC:-/registered_cloud}}"
LIDAR_RAW_TOPIC="${LIDAR_RAW_TOPIC:-/velodyne_points}"
LIDAR_DEVICE_IP="${LIDAR_DEVICE_IP:-192.168.8.201}"
LIDAR_HOST_IP="${LIDAR_HOST_IP:-192.168.8.174}"
LIDAR_DATA_PORT="${LIDAR_DATA_PORT:-2368}"
CAMERA_TOPIC="${CAMERA_TOPIC:-/camera/depth/color/points}"
CAMERA_DEPTH_TOPIC="${CAMERA_DEPTH_TOPIC:-/camera/aligned_depth_to_color/image_raw}"
USE_ALIGNED_DEPTH_FOR_CAMERA="${USE_ALIGNED_DEPTH_FOR_CAMERA:-false}"
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
CAMERA_MAX_RANGE="${CAMERA_MAX_RANGE:-$DEFAULT_CAMERA_MAX_RANGE}"
CAMERA_DEPTH_PIXEL_STEP="${CAMERA_DEPTH_PIXEL_STEP:-2}"
CAMERA_OUTLIER_FILTER="${CAMERA_OUTLIER_FILTER:-$DEFAULT_CAMERA_OUTLIER_FILTER}"
CAMERA_SOR_MEAN_K="${CAMERA_SOR_MEAN_K:-24}"
CAMERA_SOR_STDDEV_MUL="${CAMERA_SOR_STDDEV_MUL:-1.0}"
CAMERA_ROR_RADIUS="${CAMERA_ROR_RADIUS:-0.08}"
CAMERA_ROR_MIN_NEIGHBORS="${CAMERA_ROR_MIN_NEIGHBORS:-3}"
CAMERA_MIN_POINTS="${CAMERA_MIN_POINTS:-30}"
CAMERA_KEYFRAME_MIN_TRANSLATION="${CAMERA_KEYFRAME_MIN_TRANSLATION:-$DEFAULT_CAMERA_KEYFRAME_MIN_TRANSLATION}"
CAMERA_KEYFRAME_MIN_ROTATION_DEG="${CAMERA_KEYFRAME_MIN_ROTATION_DEG:-$DEFAULT_CAMERA_KEYFRAME_MIN_ROTATION_DEG}"
CAMERA_SYNC_TOLERANCE="${CAMERA_SYNC_TOLERANCE:-0.03}"
CAMERA_SYNC_QUEUE_SIZE="${CAMERA_SYNC_QUEUE_SIZE:-10}"
PCD_FILE="${PCD_FILE:-$SESSION_DIR/$SESSION_NAME.pcd}"
RVIZ="${RVIZ:-true}"

ENABLE_LIDAR="${ENABLE_LIDAR:-true}"
ENABLE_CAMERA="${ENABLE_CAMERA:-true}"
SAVE_LIDAR="${SAVE_LIDAR:-true}"
SAVE_CAMERA="${SAVE_CAMERA:-true}"
[ -n "${CAMERA_PARENT_FRAME+x}" ] && CAMERA_PARENT_FRAME_WAS_SET=true || true
[ -n "${CAMERA_CHILD_FRAME+x}" ] && CAMERA_CHILD_FRAME_WAS_SET=true || true
[ -n "${CAMERA_XYZ+x}" ] && CAMERA_XYZ_WAS_SET=true || true
[ -n "${CAMERA_RPY+x}" ] && CAMERA_RPY_WAS_SET=true || true
DEFAULT_CAMERA_CALIBRATION_FILE="$CATKIN_WS_DIR/calibration/camera_lidar_calibration.yaml"
CAMERA_CALIBRATION_FILE="${CAMERA_CALIBRATION_FILE:-$DEFAULT_CAMERA_CALIBRATION_FILE}"

# Accept a workspace/calibration directory as well as a complete YAML path.
if [ -d "$CAMERA_CALIBRATION_FILE" ]; then
  if [ -f "${CAMERA_CALIBRATION_FILE%/}/calibration/camera_lidar_calibration.yaml" ]; then
    CAMERA_CALIBRATION_FILE="${CAMERA_CALIBRATION_FILE%/}/calibration/camera_lidar_calibration.yaml"
  elif [ -f "${CAMERA_CALIBRATION_FILE%/}/camera_lidar_calibration.yaml" ]; then
    CAMERA_CALIBRATION_FILE="${CAMERA_CALIBRATION_FILE%/}/camera_lidar_calibration.yaml"
  else
    CAMERA_CALIBRATION_FILE="$DEFAULT_CAMERA_CALIBRATION_FILE"
  fi
elif [ ! -f "$CAMERA_CALIBRATION_FILE" ] && [ -f "$DEFAULT_CAMERA_CALIBRATION_FILE" ]; then
  echo "WARNING: requested calibration file not found: $CAMERA_CALIBRATION_FILE"
  echo "Using repository calibration: $DEFAULT_CAMERA_CALIBRATION_FILE"
  CAMERA_CALIBRATION_FILE="$DEFAULT_CAMERA_CALIBRATION_FILE"
fi
CAMERA_NAME="${CAMERA_NAME:-camera}"
CAMERA_PARENT_FRAME="${CAMERA_PARENT_FRAME:-base_link}"
CAMERA_CHILD_FRAME="${CAMERA_CHILD_FRAME:-camera_link}"
CAMERA_XYZ="${CAMERA_XYZ:-0.16 0.0 0.20}"
CAMERA_RPY="${CAMERA_RPY:-0 0 0}"
CAMERA_VISUALIZATION_FRAME="${CAMERA_VISUALIZATION_FRAME:-camera_depth_optical_frame}"
RVIZ_FIXED_FRAME="${RVIZ_FIXED_FRAME:-$CAMERA_VISUALIZATION_FRAME}"
CAMERA_INTENSITY="${CAMERA_INTENSITY:-0.0}"
TRANSFORM_TIMEOUT="${TRANSFORM_TIMEOUT:-0.5}"
USE_LATEST_TF_ON_FAILURE="${USE_LATEST_TF_ON_FAILURE:-false}"
LEGO_USE_IMU="${LEGO_USE_IMU:-false}"
LEGO_LOCK_ROLL_PITCH="${LEGO_LOCK_ROLL_PITCH:-true}"
ENABLE_GPS="${ENABLE_GPS:-true}"
GPS_METADATA_DIR="${GPS_METADATA_DIR:-$SESSION_DIR}"
GPS_TCP_BIND="${GPS_TCP_BIND:-0.0.0.0}"
GPS_TCP_PORT="${GPS_TCP_PORT:-29500}"
GPS_ALLOWED_HOSTS="${GPS_ALLOWED_HOSTS:-}"
GPS_FRAME="${GPS_FRAME:-gps_link}"
GPS_ROBOT_FRAME="${GPS_ROBOT_FRAME:-base_link}"
GPS_MIN_SATS="${GPS_MIN_SATS:-4}"
GPS_MAX_HDOP="${GPS_MAX_HDOP:-5.0}"
GPS_MAX_AGE_MS="${GPS_MAX_AGE_MS:-2000}"
GPS_REQUIRE_FIX="${GPS_REQUIRE_FIX:-true}"
GPS_REQUIRE_SATS="${GPS_REQUIRE_SATS:-true}"
GPS_REQUIRE_HDOP="${GPS_REQUIRE_HDOP:-true}"
GPS_REQUIRE_AGE="${GPS_REQUIRE_AGE:-true}"
GPS_ASSOCIATION_MAX_AGE_SEC="${GPS_ASSOCIATION_MAX_AGE_SEC:-2.0}"
GPS_TF_WAIT_TIMEOUT_SEC="${GPS_TF_WAIT_TIMEOUT_SEC:-0.5}"
GPS_MAX_LINE_BYTES="${GPS_MAX_LINE_BYTES:-8192}"
GPS_DATUM_MODE="${GPS_DATUM_MODE:-first_valid_fix}"
GPS_DATUM_LATITUDE="${GPS_DATUM_LATITUDE:-}"
GPS_DATUM_LONGITUDE="${GPS_DATUM_LONGITUDE:-}"
GPS_DATUM_ALTITUDE="${GPS_DATUM_ALTITUDE:-}"
GPS_TRAJECTORY_PATH_TOPIC="${GPS_TRAJECTORY_PATH_TOPIC:-/gps_map_trajectory_path}"
GPS_TRAJECTORY_MAX_POINTS="${GPS_TRAJECTORY_MAX_POINTS:-10000}"

if [ -f "$CAMERA_CALIBRATION_FILE" ]; then
  eval "$(python "$CATKIN_WS_DIR/src/scout_pointcloud_accumulator/scripts/camera_lidar_calibration_env.py" "$CAMERA_CALIBRATION_FILE")"
  [ -n "${CAMERA_PARENT_FRAME_WAS_SET:-}" ] || CAMERA_PARENT_FRAME="${YAML_CAMERA_PARENT_FRAME:-$CAMERA_PARENT_FRAME}"
  [ -n "${CAMERA_CHILD_FRAME_WAS_SET:-}" ] || CAMERA_CHILD_FRAME="${YAML_CAMERA_CHILD_FRAME:-$CAMERA_CHILD_FRAME}"
  [ -n "${CAMERA_XYZ_WAS_SET:-}" ] || CAMERA_XYZ="${YAML_CAMERA_XYZ:-$CAMERA_XYZ}"
  [ -n "${CAMERA_RPY_WAS_SET:-}" ] || CAMERA_RPY="${YAML_CAMERA_RPY:-$CAMERA_RPY}"
  echo "Loaded camera calibration: $CAMERA_CALIBRATION_FILE"
else
  echo "WARNING: camera calibration file not found: $CAMERA_CALIBRATION_FILE"
  echo "Script directory: $SCRIPT_DIR"
fi

REALSENSE_DEPTH_WIDTH="${REALSENSE_DEPTH_WIDTH:-$DEFAULT_REALSENSE_DEPTH_WIDTH}"
REALSENSE_DEPTH_HEIGHT="${REALSENSE_DEPTH_HEIGHT:-$DEFAULT_REALSENSE_DEPTH_HEIGHT}"
REALSENSE_COLOR_WIDTH="${REALSENSE_COLOR_WIDTH:-$DEFAULT_REALSENSE_COLOR_WIDTH}"
REALSENSE_COLOR_HEIGHT="${REALSENSE_COLOR_HEIGHT:-$DEFAULT_REALSENSE_COLOR_HEIGHT}"
REALSENSE_DEPTH_FPS="${REALSENSE_DEPTH_FPS:-$DEFAULT_REALSENSE_DEPTH_FPS}"
REALSENSE_COLOR_FPS="${REALSENSE_COLOR_FPS:-$DEFAULT_REALSENSE_COLOR_FPS}"
REALSENSE_FILTERS="${REALSENSE_FILTERS:-$DEFAULT_REALSENSE_FILTERS}"
REALSENSE_CLIP_DISTANCE="${REALSENSE_CLIP_DISTANCE:--1}"
REALSENSE_POINTCLOUD_TEXTURE_STREAM="${REALSENSE_POINTCLOUD_TEXTURE_STREAM:-RS2_STREAM_COLOR}"
REALSENSE_POINTCLOUD_TEXTURE_INDEX="${REALSENSE_POINTCLOUD_TEXTURE_INDEX:-0}"
REALSENSE_ALLOW_NO_TEXTURE_POINTS="${REALSENSE_ALLOW_NO_TEXTURE_POINTS:-false}"
REALSENSE_INITIAL_RESET="${REALSENSE_INITIAL_RESET:-false}"
REALSENSE_ENABLE_POINTCLOUD="${REALSENSE_ENABLE_POINTCLOUD:-true}"
REALSENSE_ALIGN_DEPTH="${REALSENSE_ALIGN_DEPTH:-false}"
REALSENSE_ENABLE_SYNC="${REALSENSE_ENABLE_SYNC:-false}"
REALSENSE_READY_TIMEOUT="${REALSENSE_READY_TIMEOUT:-30}"
CAMERA_OUTPUT_READY_TIMEOUT="${CAMERA_OUTPUT_READY_TIMEOUT:-30}"
LIDAR_READY_TIMEOUT="${LIDAR_READY_TIMEOUT:-15}"
RUN_GIT_COMMIT="${RUN_GIT_COMMIT:-$(git -C "$ROS_ROOT_DIR" rev-parse --short HEAD 2>/dev/null || echo unknown)}"
CAPTURE_MANIFEST="${CAPTURE_MANIFEST:-}"

mapping_validate_bool RVIZ "$RVIZ"
mapping_validate_bool ENABLE_LIDAR "$ENABLE_LIDAR"
mapping_validate_bool ENABLE_CAMERA "$ENABLE_CAMERA"
mapping_validate_bool SAVE_LIDAR "$SAVE_LIDAR"
mapping_validate_bool SAVE_CAMERA "$SAVE_CAMERA"
mapping_validate_bool ENABLE_CAMERA_COLOR "$ENABLE_CAMERA_COLOR"
mapping_validate_bool USE_ALIGNED_DEPTH_FOR_CAMERA "$USE_ALIGNED_DEPTH_FOR_CAMERA"
mapping_validate_bool USE_LATEST_TF_ON_FAILURE "$USE_LATEST_TF_ON_FAILURE"
mapping_validate_bool REALSENSE_ENABLE_POINTCLOUD "$REALSENSE_ENABLE_POINTCLOUD"
mapping_validate_bool REALSENSE_ALIGN_DEPTH "$REALSENSE_ALIGN_DEPTH"
mapping_validate_bool REALSENSE_ENABLE_SYNC "$REALSENSE_ENABLE_SYNC"
mapping_validate_bool REALSENSE_INITIAL_RESET "$REALSENSE_INITIAL_RESET"
mapping_validate_bool ENABLE_GPS "$ENABLE_GPS"
mapping_validate_bool GPS_REQUIRE_FIX "$GPS_REQUIRE_FIX"
mapping_validate_bool GPS_REQUIRE_SATS "$GPS_REQUIRE_SATS"
mapping_validate_bool GPS_REQUIRE_HDOP "$GPS_REQUIRE_HDOP"
mapping_validate_bool GPS_REQUIRE_AGE "$GPS_REQUIRE_AGE"
mapping_validate_choice CAMERA_OUTLIER_FILTER "$CAMERA_OUTLIER_FILTER" none sor ror
mapping_validate_number_min VOXEL_SIZE "$VOXEL_SIZE" 0 true
mapping_validate_number_min LIDAR_VOXEL_SIZE "$LIDAR_VOXEL_SIZE" 0 true
mapping_validate_number_min CAMERA_VOXEL_SIZE "$CAMERA_VOXEL_SIZE" 0 true
mapping_validate_number_min CAMERA_VISUALIZATION_VOXEL_SIZE "$CAMERA_VISUALIZATION_VOXEL_SIZE" 0 true
mapping_validate_number_min CAMERA_ACCUMULATE_RATE "$CAMERA_ACCUMULATE_RATE" 0 true
mapping_validate_number_min CAMERA_VISUALIZATION_RATE "$CAMERA_VISUALIZATION_RATE" 0 true
mapping_validate_number_min CAMERA_MIN_RANGE "$CAMERA_MIN_RANGE" 0 true
mapping_validate_number_min CAMERA_MAX_RANGE "$CAMERA_MAX_RANGE" 0 false
mapping_validate_less_than CAMERA_MIN_RANGE "$CAMERA_MIN_RANGE" CAMERA_MAX_RANGE "$CAMERA_MAX_RANGE"
mapping_validate_int_min CAMERA_DEPTH_PIXEL_STEP "$CAMERA_DEPTH_PIXEL_STEP" 1
mapping_validate_int_min CAMERA_SOR_MEAN_K "$CAMERA_SOR_MEAN_K" 2
mapping_validate_number_min CAMERA_SOR_STDDEV_MUL "$CAMERA_SOR_STDDEV_MUL" 0 false
mapping_validate_number_min CAMERA_ROR_RADIUS "$CAMERA_ROR_RADIUS" 0 false
mapping_validate_int_min CAMERA_ROR_MIN_NEIGHBORS "$CAMERA_ROR_MIN_NEIGHBORS" 1
mapping_validate_int_min CAMERA_MIN_POINTS "$CAMERA_MIN_POINTS" 0
mapping_validate_number_min CAMERA_KEYFRAME_MIN_TRANSLATION "$CAMERA_KEYFRAME_MIN_TRANSLATION" 0 true
mapping_validate_number_min CAMERA_KEYFRAME_MIN_ROTATION_DEG "$CAMERA_KEYFRAME_MIN_ROTATION_DEG" 0 true
mapping_validate_number_min CAMERA_SYNC_TOLERANCE "$CAMERA_SYNC_TOLERANCE" 0 true
mapping_validate_int_min CAMERA_SYNC_QUEUE_SIZE "$CAMERA_SYNC_QUEUE_SIZE" 1
mapping_validate_int_min REALSENSE_DEPTH_WIDTH "$REALSENSE_DEPTH_WIDTH" 1
mapping_validate_int_min REALSENSE_DEPTH_HEIGHT "$REALSENSE_DEPTH_HEIGHT" 1
mapping_validate_int_min REALSENSE_COLOR_WIDTH "$REALSENSE_COLOR_WIDTH" 1
mapping_validate_int_min REALSENSE_COLOR_HEIGHT "$REALSENSE_COLOR_HEIGHT" 1
mapping_validate_int_min REALSENSE_DEPTH_FPS "$REALSENSE_DEPTH_FPS" 1
mapping_validate_int_min REALSENSE_COLOR_FPS "$REALSENSE_COLOR_FPS" 1
mapping_validate_int_min GPS_TCP_PORT "$GPS_TCP_PORT" 1
mapping_validate_int_min GPS_MIN_SATS "$GPS_MIN_SATS" 0
mapping_validate_number_min GPS_MAX_HDOP "$GPS_MAX_HDOP" 0 false
mapping_validate_number_min GPS_MAX_AGE_MS "$GPS_MAX_AGE_MS" 0 false
mapping_validate_number_min GPS_ASSOCIATION_MAX_AGE_SEC "$GPS_ASSOCIATION_MAX_AGE_SEC" 0 false
mapping_validate_number_min GPS_TF_WAIT_TIMEOUT_SEC "$GPS_TF_WAIT_TIMEOUT_SEC" 0 true
mapping_validate_int_min GPS_MAX_LINE_BYTES "$GPS_MAX_LINE_BYTES" 1
mapping_validate_int_min GPS_TRAJECTORY_MAX_POINTS "$GPS_TRAJECTORY_MAX_POINTS" 0
if [ "$ENABLE_GPS" = "true" ] && [ "$GPS_TCP_BIND" != "127.0.0.1" ] && \
   [ "$GPS_TCP_BIND" != "localhost" ] && [ "$GPS_TCP_BIND" != "::1" ] && \
   [ -z "$GPS_ALLOWED_HOSTS" ]; then
  echo "ERROR: ENABLE_GPS=true with GPS_TCP_BIND=$GPS_TCP_BIND requires GPS_ALLOWED_HOSTS=IP_DE_TU_PC." >&2
  echo "Example: GPS_ALLOWED_HOSTS=192.168.8.10 ./scripts/start_lidar_mapping.sh" >&2
  echo "Or disable GPS startup with: ENABLE_GPS=false ./scripts/start_lidar_mapping.sh" >&2
  exit 1
fi

mkdir -p "$OUTPUT_DIR" "$SESSION_DIR" "$LOG_DIR"

if [ "$RVIZ" = "true" ]; then
  mapping_validate_rviz_environment
fi

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

STARTUP_COMMITTED=false
cleanup_partial_start() {
  local status="$?"
  if [ "$STARTUP_COMMITTED" != true ]; then
    echo "Mapping startup failed; stopping processes started by this session." >&2
    if [ -f "$PID_FILE" ]; then
      while read -r pid name _; do
        if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
          echo "Stopping session process $name pid=$pid" >&2
          kill -- "-$pid" 2>/dev/null || kill "$pid" 2>/dev/null || true
        fi
      done < <(tac "$PID_FILE")
      rm -f "$PID_FILE"
    fi
  fi
  return "$status"
}
trap cleanup_partial_start EXIT

start_process() {
  local name="$1"
  shift
  local log_file="$LOG_DIR/${name}.log"
  local command=""

  printf -v command "%q " "$@"

  nohup setsid bash -lc "
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
  device_ip:="$LIDAR_DEVICE_IP" \
  port:="$LIDAR_DATA_PORT" \
  enable_rf2o:=false \
  publish_robot_description:=false
sleep 2
if [ "$ENABLE_LIDAR" = "true" ] && ! ip -4 -o addr show | grep -Fq " $LIDAR_HOST_IP/"; then
  echo "ERROR: LiDAR sends to $LIDAR_HOST_IP, but this address is not assigned to the Xavier." >&2
  ip -br -4 addr >&2 || true
  exit 1
fi
if [ "$ENABLE_LIDAR" = "true" ] && \
   ! mapping_wait_for_topics_once "$LIDAR_READY_TIMEOUT" "$LIDAR_RAW_TOPIC"; then
  echo "ERROR: no LiDAR packets were received on $LIDAR_RAW_TOPIC within ${LIDAR_READY_TIMEOUT}s." >&2
  echo "Expected sensor=$LIDAR_DEVICE_IP destination=$LIDAR_HOST_IP UDP=$LIDAR_DATA_PORT." >&2
  echo "Verify LiDAR power, Ethernet cabling and that the saved sensor configuration matches these values." >&2
  echo "Current IPv4 addresses:" >&2
  ip -br -4 addr >&2 || true
  tail -n 40 "$LOG_DIR/lidar.log" >&2 2>/dev/null || true
  exit 1
fi
if [ "$ENABLE_CAMERA" = "true" ]; then
  mapping_camera_preflight
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
    enable_pointcloud:="$REALSENSE_ENABLE_POINTCLOUD" \
    align_depth:="$REALSENSE_ALIGN_DEPTH" \
    enable_sync:="$REALSENSE_ENABLE_SYNC" \
    initial_reset:="$REALSENSE_INITIAL_RESET"

  if [ "$USE_ALIGNED_DEPTH_FOR_CAMERA" = "true" ]; then
    if ! mapping_wait_for_topics_once "$REALSENSE_READY_TIMEOUT" \
        "$CAMERA_DEPTH_TOPIC" "$CAMERA_COLOR_TOPIC" "$CAMERA_INFO_TOPIC"; then
      mapping_print_camera_diagnostics "$LOG_DIR/realsense.log"
      exit 1
    fi
  elif ! wait_for_ros_node /camera/realsense2_camera "$REALSENSE_READY_TIMEOUT"; then
    mapping_print_camera_diagnostics "$LOG_DIR/realsense.log"
    exit 1
  fi
  echo "RealSense camera driver is ready."
fi

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
  camera_visualization_frame:="$CAMERA_VISUALIZATION_FRAME" \
  rviz_fixed_frame:="$RVIZ_FIXED_FRAME" \
  voxel_size:="$VOXEL_SIZE" \
  lidar_voxel_size:="$LIDAR_VOXEL_SIZE" \
  camera_voxel_size:="$CAMERA_VOXEL_SIZE" \
  camera_visualization_voxel_size:="$CAMERA_VISUALIZATION_VOXEL_SIZE" \
  camera_accumulate_rate:="$CAMERA_ACCUMULATE_RATE" \
  camera_visualization_rate:="$CAMERA_VISUALIZATION_RATE" \
  camera_min_range:="$CAMERA_MIN_RANGE" \
  camera_max_range:="$CAMERA_MAX_RANGE" \
  camera_depth_pixel_step:="$CAMERA_DEPTH_PIXEL_STEP" \
  camera_outlier_filter:="$CAMERA_OUTLIER_FILTER" \
  camera_sor_mean_k:="$CAMERA_SOR_MEAN_K" \
  camera_sor_stddev_mul:="$CAMERA_SOR_STDDEV_MUL" \
  camera_ror_radius:="$CAMERA_ROR_RADIUS" \
  camera_ror_min_neighbors:="$CAMERA_ROR_MIN_NEIGHBORS" \
  camera_min_points:="$CAMERA_MIN_POINTS" \
  camera_keyframe_min_translation:="$CAMERA_KEYFRAME_MIN_TRANSLATION" \
  camera_keyframe_min_rotation_deg:="$CAMERA_KEYFRAME_MIN_ROTATION_DEG" \
  camera_sync_tolerance:="$CAMERA_SYNC_TOLERANCE" \
  camera_sync_queue_size:="$CAMERA_SYNC_QUEUE_SIZE" \
  camera_intensity:="$CAMERA_INTENSITY" \
  transform_timeout:="$TRANSFORM_TIMEOUT" \
  use_latest_tf_on_failure:="$USE_LATEST_TF_ON_FAILURE" \
  output_pcd:="$PCD_FILE" \
  save_lidar:="$SAVE_LIDAR" \
  save_camera:="$SAVE_CAMERA" \
  mapping_profile:="$MAPPING_PROFILE" \
  run_git_commit:="$RUN_GIT_COMMIT" \
  capture_manifest:="$CAPTURE_MANIFEST" \
  camera_parent_frame:="$CAMERA_PARENT_FRAME" \
  camera_child_frame:="$CAMERA_CHILD_FRAME" \
  camera_xyz:="$CAMERA_XYZ" \
  camera_rpy:="$CAMERA_RPY" \
  rviz:="$RVIZ"

if ! wait_for_ros_node /accumulator_node 15; then
  echo "ERROR: accumulator_node did not stay alive after launch."
  echo "Check: $LOG_DIR/accumulator.log"
  tail -n 80 "$LOG_DIR/accumulator.log" 2>/dev/null || true
  exit 1
fi

if [ "$ENABLE_CAMERA" = "true" ] && [ "$USE_ALIGNED_DEPTH_FOR_CAMERA" != "true" ]; then
  if ! mapping_wait_for_topics_once "$REALSENSE_READY_TIMEOUT" "$CAMERA_TOPIC"; then
    mapping_print_camera_diagnostics "$LOG_DIR/realsense.log"
    exit 1
  fi
fi

if [ "$ENABLE_CAMERA" = "true" ] && \
   ! mapping_wait_for_camera_topics "$CAMERA_OUTPUT_READY_TIMEOUT" /camera/colored_points; then
  echo "ERROR: accumulator did not sustain /camera/colored_points." >&2
  echo "The RealSense input is running, but the accumulator could not publish its visualization output." >&2
  echo "Check camera TF and inspect: $LOG_DIR/accumulator.log" >&2
  tail -n 60 "$LOG_DIR/accumulator.log" 2>/dev/null || true
  exit 1
fi

if [ "$ENABLE_GPS" = "true" ]; then
  start_process gps_metadata roslaunch scout_pointcloud_accumulator mapping_gps_metadata.launch \
    output_pcd:="$PCD_FILE" \
    metadata_dir:="$GPS_METADATA_DIR" \
    target_frame:="$TARGET_FRAME" \
    robot_frame:="$GPS_ROBOT_FRAME" \
    gps_frame:="$GPS_FRAME" \
    gps_tcp_bind:="$GPS_TCP_BIND" \
    gps_tcp_port:="$GPS_TCP_PORT" \
    gps_allowed_hosts:="$GPS_ALLOWED_HOSTS" \
    gps_min_sats:="$GPS_MIN_SATS" \
    gps_max_hdop:="$GPS_MAX_HDOP" \
    gps_max_age_ms:="$GPS_MAX_AGE_MS" \
    gps_require_fix:="$GPS_REQUIRE_FIX" \
    gps_require_sats:="$GPS_REQUIRE_SATS" \
    gps_require_hdop:="$GPS_REQUIRE_HDOP" \
    gps_require_age:="$GPS_REQUIRE_AGE" \
    gps_association_max_age_sec:="$GPS_ASSOCIATION_MAX_AGE_SEC" \
    gps_tf_wait_timeout_sec:="$GPS_TF_WAIT_TIMEOUT_SEC" \
    gps_max_line_bytes:="$GPS_MAX_LINE_BYTES" \
    datum_mode:="$GPS_DATUM_MODE" \
    datum_latitude:="$GPS_DATUM_LATITUDE" \
    datum_longitude:="$GPS_DATUM_LONGITUDE" \
    datum_altitude:="$GPS_DATUM_ALTITUDE" \
    trajectory_path_topic:="$GPS_TRAJECTORY_PATH_TOPIC" \
    trajectory_max_points:="$GPS_TRAJECTORY_MAX_POINTS"

  if ! wait_for_ros_node /mapping_gps_metadata_logger 10; then
    echo "ERROR: GPS metadata sidecar did not stay alive after launch." >&2
    echo "Check: $LOG_DIR/gps_metadata.log" >&2
    tail -n 80 "$LOG_DIR/gps_metadata.log" 2>/dev/null || true
    exit 1
  fi
fi

if [ "$RVIZ" = "true" ] && ! wait_for_ros_node /rviz 10; then
  echo "ERROR: RViz did not stay alive. Check DISPLAY and $LOG_DIR/accumulator.log" >&2
  exit 1
fi

STARTUP_COMMITTED=true

cat <<EOF

Mapping started without tmux.

Profile:
  $MAPPING_PROFILE

Quality parameters:
  RealSense ${REALSENSE_DEPTH_WIDTH}x${REALSENSE_DEPTH_HEIGHT}@${REALSENSE_DEPTH_FPS} filters='${REALSENSE_FILTERS}'
  camera_range=${CAMERA_MIN_RANGE}-${CAMERA_MAX_RANGE}m outlier_filter=${CAMERA_OUTLIER_FILTER}
  keyframe_min_translation=${CAMERA_KEYFRAME_MIN_TRANSLATION}m keyframe_min_rotation=${CAMERA_KEYFRAME_MIN_ROTATION_DEG}deg

Logs:
  $LOG_DIR

Accumulated PCD outputs:
  ${PCD_FILE%.pcd}_lidar.pcd
  ${PCD_FILE%.pcd}_camera.pcd
  ${PCD_FILE%.pcd}_fused_quality.pcd
  ${PCD_FILE%.pcd}_manifest.json

GPS:
  enabled=${ENABLE_GPS}
  metadata_dir=${GPS_METADATA_DIR}
  tcp=${GPS_TCP_BIND}:${GPS_TCP_PORT}
  allowed_hosts=${GPS_ALLOWED_HOSTS:-none}

Save at any time:
  $WORKSPACE/scripts/save_accumulated_map.sh

Stop everything:
  $WORKSPACE/scripts/stop_lidar_mapping.sh

Watch logs:
  tail -f $LOG_DIR/realsense.log
  tail -f $LOG_DIR/lego_loam.log
  tail -f $LOG_DIR/accumulator.log
  tail -f $LOG_DIR/gps_metadata.log
EOF
