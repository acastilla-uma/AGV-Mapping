#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/melodic/setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-$(cd "$SCRIPT_DIR/.." && pwd)}"
if [ -f "$WORKSPACE/devel/setup.bash" ]; then
  source "$WORKSPACE/devel/setup.bash"
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
      echo "ERROR: GPS metadata save did not complete." >&2
      return 1
    fi
  fi
  return 0
}

save_status=0
"$ROSSERVICE_CMD" call /accumulator_node/save_accumulated "{}" || save_status=$?
gps_status=0
save_gps_metadata_if_available || gps_status=$?
if [ "$save_status" -ne 0 ]; then
  exit "$save_status"
fi
if [ "$gps_status" -ne 0 ]; then
  exit "$gps_status"
fi
exit "$save_status"
