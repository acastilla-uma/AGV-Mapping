#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LIB="$ROOT_DIR/catkin_ws/scripts/mapping_startup_health.sh"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

assert_eq() {
  [ "$1" = "$2" ] || fail "expected '$2', got '$1'"
}

source "$LIB"

DISPLAY=:0 mapping_validate_rviz_environment || fail "graphical RViz environment rejected"
if DISPLAY= mapping_validate_rviz_environment >"$TMP_DIR/rviz.out" 2>&1; then
  fail "headless RViz environment accepted"
fi
grep -q 'RVIZ=false' "$TMP_DIR/rviz.out" || fail "headless diagnostic missing"

cat > "$TMP_DIR/rostopic-ok" <<'EOF'
#!/usr/bin/env bash
exit 0
EOF
cat > "$TMP_DIR/rostopic-fail" <<'EOF'
#!/usr/bin/env bash
exit 1
EOF
cat > "$TMP_DIR/rostopic-slow" <<'EOF'
#!/usr/bin/env bash
sleep 2
exit 0
EOF
cat > "$TMP_DIR/rosnode-empty" <<'EOF'
#!/usr/bin/env bash
[ "${1:-}" = list ] && exit 0
exit 1
EOF
cat > "$TMP_DIR/rosnode-camera" <<'EOF'
#!/usr/bin/env bash
[ "${1:-}" = list ] || exit 1
echo /camera/realsense2_camera
echo /camera/realsense2_camera_manager
EOF
cat > "$TMP_DIR/rosnode-fail" <<'EOF'
#!/usr/bin/env bash
exit 1
EOF
cat > "$TMP_DIR/pgrep-empty" <<'EOF'
#!/usr/bin/env bash
exit 1
EOF
cat > "$TMP_DIR/pgrep-camera" <<'EOF'
#!/usr/bin/env bash
echo '14997 /opt/ros/melodic/lib/nodelet/nodelet manager __name:=realsense2_camera_manager'
EOF
chmod +x "$TMP_DIR"/*

ROSTOPIC_CMD="$TMP_DIR/rostopic-ok"
ROSNODE_CMD="$TMP_DIR/rosnode-empty"
PGREP_CMD="$TMP_DIR/pgrep-empty"
CAMERA_READY_SAMPLE_MESSAGES=1
CAMERA_READY_SAMPLE_TIMEOUT_SEC=1
mapping_wait_for_camera_topics 1 /depth /color /info || fail "healthy topics rejected"
mapping_wait_for_topics_once 1 /points || fail "healthy lazy topic rejected"
mapping_camera_preflight || fail "empty ROS graph rejected"

ROSNODE_CMD="$TMP_DIR/rosnode-fail"
if mapping_camera_preflight >"$TMP_DIR/master.out" 2>&1; then
  fail "unreachable ROS master accepted"
fi
grep -q 'ROS graph' "$TMP_DIR/master.out" || fail "ROS graph failure diagnostic missing"

ROSNODE_CMD="$TMP_DIR/rosnode-empty"
ROSTOPIC_CMD="$TMP_DIR/rostopic-slow"
CAMERA_READY_SAMPLE_TIMEOUT_SEC=1
if mapping_wait_for_camera_topics 1 /depth; then
  fail "camera stream below the readiness rate accepted"
fi

ROSTOPIC_CMD="$TMP_DIR/rostopic-fail"
if mapping_wait_for_camera_topics 1 /depth /color /info; then
  fail "missing topics accepted"
fi
if mapping_wait_for_topics_once 1 /points; then
  fail "missing lazy topic accepted"
fi

ROSNODE_CMD="$TMP_DIR/rosnode-camera"
if mapping_camera_preflight >"$TMP_DIR/preflight.out" 2>&1; then
  fail "pre-existing RealSense nodes accepted"
fi
grep -q '/camera/realsense2_camera' "$TMP_DIR/preflight.out" || fail "owner diagnostic missing"

ROSNODE_CMD="$TMP_DIR/rosnode-empty"
PGREP_CMD="$TMP_DIR/pgrep-camera"
if mapping_camera_preflight >"$TMP_DIR/process.out" 2>&1; then
  fail "orphan RealSense process accepted"
fi
grep -q '14997' "$TMP_DIR/process.out" || fail "orphan process diagnostic missing"

cat > "$TMP_DIR/realsense.log" <<'EOF'
failed to claim usb interface: 0, error: RS2_USB_STATUS_BUSY
EOF
assert_eq "$(mapping_realsense_failure_reason "$TMP_DIR/realsense.log")" "RealSense USB interface is busy (RS2_USB_STATUS_BUSY)."

cat > "$TMP_DIR/realsense.log" <<'EOF'
Hardware Notification:Right MIPI error,Error,Hardware Error
EOF
assert_eq "$(mapping_realsense_failure_reason "$TMP_DIR/realsense.log")" "RealSense reported an internal MIPI/hardware error and stopped streaming."

: > "$TMP_DIR/realsense.log"
assert_eq "$(mapping_realsense_failure_reason "$TMP_DIR/realsense.log")" "RealSense did not publish the required camera streams."

if grep -q 'mapping_metadata' "$ROOT_DIR/catkin_ws/scripts/start_lidar_mapping.sh" \
    "$ROOT_DIR/catkin_ws/scripts/stop_lidar_mapping.sh" \
    "$ROOT_DIR/catkin_ws/src/scout_pointcloud_accumulator/CMakeLists.txt"; then
  fail "stale mapping-metadata integration is still referenced"
fi

grep -q 'REALSENSE_INITIAL_RESET="${REALSENSE_INITIAL_RESET:-false}"' \
  "$ROOT_DIR/catkin_ws/scripts/start_lidar_mapping.sh" || fail "unsafe RealSense reset default restored"
grep -q 'USE_ALIGNED_DEPTH_FOR_CAMERA="${USE_ALIGNED_DEPTH_FOR_CAMERA:-false}"' \
  "$ROOT_DIR/catkin_ws/scripts/start_lidar_mapping.sh" || fail "unstable aligned-depth path restored"
grep -q 'REALSENSE_ENABLE_POINTCLOUD="${REALSENSE_ENABLE_POINTCLOUD:-true}"' \
  "$ROOT_DIR/catkin_ws/scripts/start_lidar_mapping.sh" || fail "native pointcloud path disabled"
grep -q 'CAMERA_VISUALIZATION_FRAME="${CAMERA_VISUALIZATION_FRAME:-camera_depth_optical_frame}"' \
  "$ROOT_DIR/catkin_ws/scripts/start_lidar_mapping.sh" || fail "camera visualization is not isolated from the map frame"
grep -q 'camera_visualization_frame:="$CAMERA_VISUALIZATION_FRAME"' \
  "$ROOT_DIR/catkin_ws/scripts/start_lidar_mapping.sh" || fail "camera visualization frame is not passed to the accumulator"
grep -q 'rviz_fixed_frame:="$RVIZ_FIXED_FRAME"' \
  "$ROOT_DIR/catkin_ws/scripts/start_lidar_mapping.sh" || fail "RViz fallback frame is not passed to launch"
grep -q 'LIDAR_RAW_TOPIC="${LIDAR_RAW_TOPIC:-/velodyne_points}"' \
  "$ROOT_DIR/catkin_ws/scripts/start_lidar_mapping.sh" || fail "raw LiDAR readiness topic is not configured"
grep -q 'LIDAR_DEVICE_IP="${LIDAR_DEVICE_IP:-192.168.8.201}"' \
  "$ROOT_DIR/catkin_ws/scripts/start_lidar_mapping.sh" || fail "Velodyne source IP is not configured"
grep -q 'LIDAR_HOST_IP="${LIDAR_HOST_IP:-192.168.8.174}"' \
  "$ROOT_DIR/catkin_ws/scripts/start_lidar_mapping.sh" || fail "Velodyne destination host IP is not configured"
grep -q 'device_ip:="$LIDAR_DEVICE_IP"' \
  "$ROOT_DIR/catkin_ws/scripts/start_lidar_mapping.sh" || fail "Velodyne source IP is not passed to roslaunch"
grep -q 'mapping_wait_for_topics_once "$LIDAR_READY_TIMEOUT" "$LIDAR_RAW_TOPIC"' \
  "$ROOT_DIR/catkin_ws/scripts/start_lidar_mapping.sh" || fail "startup does not verify raw LiDAR packets"
grep -q 'Topic: /velodyne_points' \
  "$ROOT_DIR/catkin_ws/src/scout_pointcloud_accumulator/rviz/accum.rviz" || fail "RViz instant LiDAR topic is incorrect"

if grep -A12 '^cleanup_partial_start()' "$ROOT_DIR/catkin_ws/scripts/start_lidar_mapping.sh" | grep -q 'stop_lidar_mapping'; then
  fail "startup rollback delegates to global stop"
fi

echo "mapping startup health tests: PASS"
