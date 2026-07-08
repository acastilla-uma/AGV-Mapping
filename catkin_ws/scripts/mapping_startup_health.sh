#!/usr/bin/env bash

ROSTOPIC_CMD="${ROSTOPIC_CMD:-rostopic}"
ROSNODE_CMD="${ROSNODE_CMD:-rosnode}"
PGREP_CMD="${PGREP_CMD:-pgrep}"

mapping_validate_rviz_environment() {
  if [ -z "${DISPLAY:-}" ]; then
    echo "ERROR: RVIZ=true but DISPLAY is not set. Start from a graphical session or use RVIZ=false." >&2
    return 1
  fi
}

mapping_camera_preflight() {
  local nodes
  if ! nodes="$($ROSNODE_CMD list 2>/dev/null)"; then
    echo "ERROR: unable to query the ROS graph; verify that roscore is reachable." >&2
    return 1
  fi
  if grep -Eq '^/camera/realsense2_camera(_manager)?$' <<<"$nodes"; then
    echo "ERROR: a RealSense ROS node is already running:" >&2
    grep -E '^/camera/realsense2_camera(_manager)?$' <<<"$nodes" >&2
    echo "Stop the existing camera session before starting mapping." >&2
    return 1
  fi

  local processes
  processes="$($PGREP_CMD -af '/opt/ros/.*/nodelet/nodelet (manager|load).*__name:=realsense2_camera' 2>/dev/null || true)"
  if [ -n "$processes" ]; then
    echo "ERROR: a RealSense nodelet is already running outside the current ROS graph:" >&2
    echo "$processes" >&2
    echo "Stop the stale camera process before starting mapping." >&2
    return 1
  fi
}

mapping_wait_for_camera_topics() {
  local timeout_sec="$1"
  shift
  local deadline=$((SECONDS + timeout_sec))
  local sample_messages="${CAMERA_READY_SAMPLE_MESSAGES:-4}"
  local sample_timeout="${CAMERA_READY_SAMPLE_TIMEOUT_SEC:-5}"
  local topic

  for topic in "$@"; do
    local remaining=$((deadline - SECONDS))
    [ "$remaining" -gt 0 ] || return 1
    local topic_timeout="$sample_timeout"
    [ "$topic_timeout" -le "$remaining" ] || topic_timeout="$remaining"
    if ! timeout "${topic_timeout}s" "$ROSTOPIC_CMD" echo -n "$sample_messages" "$topic" >/dev/null 2>&1; then
      return 1
    fi
  done
}

mapping_wait_for_topics_once() {
  local timeout_sec="$1"
  shift
  local deadline=$((SECONDS + timeout_sec))
  local topic
  for topic in "$@"; do
    local remaining=$((deadline - SECONDS))
    [ "$remaining" -gt 0 ] || return 1
    if ! timeout "${remaining}s" "$ROSTOPIC_CMD" echo -n 1 "$topic" >/dev/null 2>&1; then
      return 1
    fi
  done
}

mapping_realsense_failure_reason() {
  local log_file="$1"
  if grep -q 'RS2_USB_STATUS_BUSY\|failed to claim usb interface' "$log_file" 2>/dev/null; then
    echo "RealSense USB interface is busy (RS2_USB_STATUS_BUSY)."
  elif grep -q 'MIPI error\|HW not ready' "$log_file" 2>/dev/null; then
    echo "RealSense reported an internal MIPI/hardware error and stopped streaming."
  elif grep -q 'No device connected\|device with .* is NOT found' "$log_file" 2>/dev/null; then
    echo "RealSense device was not found."
  else
    echo "RealSense did not publish the required camera streams."
  fi
}

mapping_print_camera_diagnostics() {
  local log_file="$1"
  echo "ERROR: $(mapping_realsense_failure_reason "$log_file")" >&2
  echo "Check for another camera process (for example realsense-viewer) and inspect: $log_file" >&2
  "$PGREP_CMD" -af 'realsense2_camera|realsense-viewer' >&2 || true
  tail -n 40 "$log_file" >&2 2>/dev/null || true
}
