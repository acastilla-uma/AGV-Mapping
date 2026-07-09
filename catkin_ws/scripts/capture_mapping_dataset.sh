#!/usr/bin/env bash
set -euo pipefail

# Capture a reproducible mapping rosbag plus a JSON manifest.
#
# Typical use:
#   SCENARIO=S4_recta_pared SPLIT=training DURATION_SEC=60 \
#     ./scripts/capture_mapping_dataset.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CATKIN_WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_DIR="$(cd "$CATKIN_WS_DIR/.." && pwd)"

SCENARIO="${SCENARIO:-manual}"
SPLIT="${SPLIT:-training}"
DURATION_SEC="${DURATION_SEC:-60}"
OUTPUT_ROOT="${OUTPUT_ROOT:-$REPO_DIR/datasets/mapping_quality}"
RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)_${SCENARIO}_${SPLIT}}"
OUTPUT_DIR="${OUTPUT_DIR:-$OUTPUT_ROOT/$RUN_ID}"
BAG_PATH="${BAG_PATH:-$OUTPUT_DIR/${RUN_ID}.bag}"
MANIFEST_PATH="${MANIFEST_PATH:-$OUTPUT_DIR/${RUN_ID}.manifest.json}"
RUN_GIT_COMMIT="${RUN_GIT_COMMIT:-$(git -C "$REPO_DIR" rev-parse --short HEAD 2>/dev/null || echo unknown)}"
CAMERA_CALIBRATION_FILE="${CAMERA_CALIBRATION_FILE:-$CATKIN_WS_DIR/calibration/camera_lidar_calibration.yaml}"

TOPICS="${TOPICS:-/registered_cloud /camera/depth/color/points /camera/color/image_raw /camera/color/camera_info /camera/aligned_depth_to_color/image_raw /tf /tf_static /aft_mapped_to_init /integrated_to_init /diagnostics}"

export SCENARIO SPLIT DURATION_SEC OUTPUT_ROOT RUN_ID OUTPUT_DIR BAG_PATH MANIFEST_PATH
export RUN_GIT_COMMIT CAMERA_CALIBRATION_FILE TOPICS

python3 - "$DURATION_SEC" <<'PY'
import math
import sys
value = float(sys.argv[1])
if not math.isfinite(value) or value <= 0:
    print("ERROR: DURATION_SEC must be > 0.", file=sys.stderr)
    sys.exit(1)
PY

mkdir -p "$OUTPUT_DIR"

write_manifest() {
  local status="$1"
  local bag_hash=""
  local bag_size=0
  if [ -f "$BAG_PATH" ]; then
    bag_hash="$(sha256sum "$BAG_PATH" | awk '{print $1}')"
    bag_size="$(stat -c '%s' "$BAG_PATH")"
  fi

  STATUS="$status" \
  BAG_HASH="$bag_hash" \
  BAG_SIZE="$bag_size" \
  python3 - "$MANIFEST_PATH" <<'PY'
import json
import os
import sys

manifest_path = sys.argv[1]
topics = [topic for topic in os.environ["TOPICS"].split() if topic]
payload = {
    "schema": "scout_mapping_capture_manifest_v1",
    "status": os.environ["STATUS"],
    "run_id": os.environ["RUN_ID"],
    "scenario": os.environ["SCENARIO"],
    "split": os.environ["SPLIT"],
    "duration_sec": float(os.environ["DURATION_SEC"]),
    "bag_path": os.environ["BAG_PATH"],
    "bag_hash_algorithm": "sha256",
    "bag_hash": os.environ["BAG_HASH"],
    "bag_size_bytes": int(os.environ["BAG_SIZE"]),
    "git_commit": os.environ["RUN_GIT_COMMIT"],
    "camera_calibration_file": os.environ["CAMERA_CALIBRATION_FILE"],
    "topics": topics,
    "resolved_config": {
        key: os.environ.get(key, "")
        for key in [
            "MAPPING_PROFILE",
            "REALSENSE_DEPTH_WIDTH",
            "REALSENSE_DEPTH_HEIGHT",
            "REALSENSE_DEPTH_FPS",
            "REALSENSE_COLOR_WIDTH",
            "REALSENSE_COLOR_HEIGHT",
            "REALSENSE_COLOR_FPS",
            "REALSENSE_FILTERS",
            "CAMERA_MIN_RANGE",
            "CAMERA_MAX_RANGE",
            "CAMERA_OUTLIER_FILTER",
            "CAMERA_VOXEL_SIZE",
            "CAMERA_ACCUMULATE_RATE",
            "USE_ALIGNED_DEPTH_FOR_CAMERA",
            "USE_LATEST_TF_ON_FAILURE",
        ]
    },
}
tmp_path = manifest_path + ".tmp"
with open(tmp_path, "w") as stream:
    json.dump(payload, stream, indent=2, sort_keys=True)
    stream.write("\n")
os.replace(tmp_path, manifest_path)
PY
}

write_manifest started

echo "Recording mapping dataset:"
echo "  scenario: $SCENARIO"
echo "  split:    $SPLIT"
echo "  duration: ${DURATION_SEC}s"
echo "  bag:      $BAG_PATH"
echo "  manifest: $MANIFEST_PATH"
echo "  topics:   $TOPICS"

set +e
timeout --signal=INT "${DURATION_SEC}s" rosbag record -O "$BAG_PATH" $TOPICS
status="$?"
set -e

if [ "$status" -ne 0 ] && [ "$status" -ne 124 ]; then
  write_manifest failed
  echo "ERROR: rosbag record failed with status $status" >&2
  exit "$status"
fi

write_manifest complete
echo "Capture complete: $MANIFEST_PATH"
