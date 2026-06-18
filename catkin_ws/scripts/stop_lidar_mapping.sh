#!/usr/bin/env bash
set -euo pipefail

RUN_DIR="${RUN_DIR:-/mnt/ros/agv_mapping}"
PID_FILE="${PID_FILE:-$RUN_DIR/pids}"


source /opt/ros/melodic/setup.bash >/dev/null 2>&1 || true
WORKSPACE="${WORKSPACE:-/mnt/ros/catkin_ws}"
if [ -f "$WORKSPACE/devel/setup.bash" ]; then
  source "$WORKSPACE/devel/setup.bash" >/dev/null 2>&1 || true
fi

if rosnode list 2>/dev/null | grep -qx "/accumulator_node"; then
  echo "Saving accumulated clouds before shutdown..."
  if ! timeout 30s rosservice call /accumulator_node/save_accumulated "{}" >/dev/null 2>&1; then
    echo "WARNING: save_accumulated did not complete before shutdown; continuing stop."
  fi
fi

if [ ! -f "$PID_FILE" ]; then
  echo "No PID file found. Will still try to stop known ROS mapping nodes."
else
  tac "$PID_FILE" | while read -r pid name _; do
    if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
      echo "Stopping $name pid=$pid"
      kill "$pid" 2>/dev/null || true
    fi
  done

  sleep 3

  tac "$PID_FILE" | while read -r pid name _; do
    if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
      echo "Force stopping $name pid=$pid"
      kill -9 "$pid" 2>/dev/null || true
    fi
  done

  rm -f "$PID_FILE"
fi


# roslaunch may have exited while nodelets stayed alive. Kill known mapping nodes too.
for node in \
  /accumulator_node \
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
  if rosnode list 2>/dev/null | grep -qx "$node"; then
    echo "Stopping ROS node $node"
    rosnode kill "$node" >/dev/null 2>&1 || true
  fi
done


echo "Stopped mapping processes."

