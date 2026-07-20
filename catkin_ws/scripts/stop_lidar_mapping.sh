#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-$(cd "$SCRIPT_DIR/.." && pwd)}"
ROS_ROOT_DIR="$(cd "$WORKSPACE/.." && pwd)"
RUN_DIR="${RUN_DIR:-$ROS_ROOT_DIR/agv_mapping}"
PID_FILE="${PID_FILE:-$RUN_DIR/pids}"


source /opt/ros/melodic/setup.bash >/dev/null 2>&1 || true
if [ -f "$WORKSPACE/devel/setup.bash" ]; then
  source "$WORKSPACE/devel/setup.bash" >/dev/null 2>&1 || true
fi

ROSNODE_CMD="${ROSNODE_CMD:-rosnode}"
ROSSERVICE_CMD="${ROSSERVICE_CMD:-rosservice}"
TIMEOUT_CMD="${TIMEOUT_CMD:-timeout}"
GPS_METADATA_NODE="${GPS_METADATA_NODE:-/mapping_gps_metadata_logger}"
GPS_METADATA_SAVE_SERVICE="${GPS_METADATA_SAVE_SERVICE:-${GPS_METADATA_NODE}/save_metadata}"
GPS_METADATA_SAVE_TIMEOUT_SEC="${GPS_METADATA_SAVE_TIMEOUT_SEC:-10}"

save_gps_metadata_if_available() {
  if "$ROSNODE_CMD" list 2>/dev/null | grep -qx "$GPS_METADATA_NODE"; then
    echo "Saving GPS metadata sidecar..."
    if ! "$TIMEOUT_CMD" "${GPS_METADATA_SAVE_TIMEOUT_SEC}s" "$ROSSERVICE_CMD" call "$GPS_METADATA_SAVE_SERVICE" "{}" >/dev/null 2>&1; then
      echo "ERROR: GPS metadata save did not complete before shutdown; continuing stop." >&2
      return 1
    fi
  fi
  return 0
}

stop_status=0

if "$ROSNODE_CMD" list 2>/dev/null | grep -qx "/accumulator_node"; then
  echo "Saving accumulated clouds before shutdown..."
  if ! "$TIMEOUT_CMD" 30s "$ROSSERVICE_CMD" call /accumulator_node/save_accumulated "{}" >/dev/null 2>&1; then
    echo "WARNING: save_accumulated did not complete before shutdown; continuing stop."
    stop_status=1
  fi
fi

save_gps_metadata_if_available || stop_status=1

if [ ! -f "$PID_FILE" ]; then
  echo "No PID file found. Will still try to stop known ROS mapping nodes."
else
  tac "$PID_FILE" | while read -r pid name _; do
    if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
      echo "Stopping $name pid=$pid"
      kill -- "-$pid" 2>/dev/null || kill "$pid" 2>/dev/null || true
    fi
  done

  sleep 3

  tac "$PID_FILE" | while read -r pid name _; do
    if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
      echo "Force stopping $name pid=$pid"
      kill -9 -- "-$pid" 2>/dev/null || kill -9 "$pid" 2>/dev/null || true
    fi
  done

  rm -f "$PID_FILE"
fi


# roslaunch may have exited while nodelets stayed alive. Kill known mapping nodes too.
for node in \
  /accumulator_node \
  /mapping_gps_metadata_logger \
  /base_link_to_realsense \
  /camera/realsense2_camera \
  /camera/realsense2_camera_manager \
  /imageProjection \
  /featureAssociation \
  /mapOptmization \
  /transformFusion \
  /camera_init_to_map \
  /base_link_to_camera \
  /base_link_to_laser \
  /rf2o_laser_odometry \
  /pointcloud_to_laserscan \
  /joint_state_publisher \
  /robot_state_publisher \
  /velodyne_nodelet_manager \
  /velodyne_nodelet_manager_driver \
  /velodyne_nodelet_manager_laserscan \
  /velodyne_nodelet_manager_transform; do
  if "$ROSNODE_CMD" list 2>/dev/null | grep -qx "$node"; then
    echo "Stopping ROS node $node"
    "$ROSNODE_CMD" kill "$node" >/dev/null 2>&1 || true
  fi
done


echo "Stopped mapping processes."
exit "$stop_status"
