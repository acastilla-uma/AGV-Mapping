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

mapping_validate_bool() {
  local name="$1"
  local value="$2"
  case "$value" in
    true|false) return 0 ;;
    *)
      echo "ERROR: $name must be 'true' or 'false' (got '$value')." >&2
      return 1
      ;;
  esac
}

mapping_validate_choice() {
  local name="$1"
  local value="$2"
  shift 2
  local choice
  for choice in "$@"; do
    if [ "$value" = "$choice" ]; then
      return 0
    fi
  done
  echo "ERROR: $name must be one of: $* (got '$value')." >&2
  return 1
}

mapping_validate_number() {
  local name="$1"
  local value="$2"
  python3 - "$name" "$value" <<'PY'
import math
import sys
name, value = sys.argv[1], sys.argv[2]
try:
    parsed = float(value)
except ValueError:
    print("ERROR: %s must be numeric (got '%s')." % (name, value), file=sys.stderr)
    sys.exit(1)
if not math.isfinite(parsed):
    print("ERROR: %s must be finite (got '%s')." % (name, value), file=sys.stderr)
    sys.exit(1)
PY
}

mapping_validate_number_min() {
  local name="$1"
  local value="$2"
  local min_value="$3"
  local inclusive="$4"
  python3 - "$name" "$value" "$min_value" "$inclusive" <<'PY'
import math
import sys
name, value, min_value, inclusive = sys.argv[1], sys.argv[2], float(sys.argv[3]), sys.argv[4] == "true"
try:
    parsed = float(value)
except ValueError:
    print("ERROR: %s must be numeric (got '%s')." % (name, value), file=sys.stderr)
    sys.exit(1)
if not math.isfinite(parsed):
    print("ERROR: %s must be finite (got '%s')." % (name, value), file=sys.stderr)
    sys.exit(1)
ok = parsed >= min_value if inclusive else parsed > min_value
if not ok:
    op = ">=" if inclusive else ">"
    print("ERROR: %s must be %s %s (got '%s')." % (name, op, min_value, value), file=sys.stderr)
    sys.exit(1)
PY
}

mapping_validate_int_min() {
  local name="$1"
  local value="$2"
  local min_value="$3"
  python3 - "$name" "$value" "$min_value" <<'PY'
import sys
name, value, min_value = sys.argv[1], sys.argv[2], int(sys.argv[3])
try:
    parsed = int(value)
except ValueError:
    print("ERROR: %s must be an integer (got '%s')." % (name, value), file=sys.stderr)
    sys.exit(1)
if str(parsed) != value and not (value.startswith("+") and str(parsed) == value[1:]):
    print("ERROR: %s must be an integer (got '%s')." % (name, value), file=sys.stderr)
    sys.exit(1)
if parsed < min_value:
    print("ERROR: %s must be >= %d (got '%s')." % (name, min_value, value), file=sys.stderr)
    sys.exit(1)
PY
}

mapping_validate_less_than() {
  local lesser_name="$1"
  local lesser_value="$2"
  local greater_name="$3"
  local greater_value="$4"
  python3 - "$lesser_name" "$lesser_value" "$greater_name" "$greater_value" <<'PY'
import sys
lesser_name, lesser_value, greater_name, greater_value = sys.argv[1:5]
lesser = float(lesser_value)
greater = float(greater_value)
if lesser >= greater:
    print("ERROR: %s (%s) must be smaller than %s (%s)." % (lesser_name, lesser_value, greater_name, greater_value), file=sys.stderr)
    sys.exit(1)
PY
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
