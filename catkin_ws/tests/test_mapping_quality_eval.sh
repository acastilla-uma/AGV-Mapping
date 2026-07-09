#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
EVAL="$ROOT_DIR/catkin_ws/src/scout_pointcloud_accumulator/scripts/mapping_quality_eval.py"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

PCD="$TMP_DIR/plane.pcd"
cat > "$PCD" <<'PCD'
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH 4
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 4
DATA ascii
0.0 0.0 1.00 0
1.0 0.0 1.02 0
0.0 1.0 0.98 0
1.0 1.0 1.01 0
PCD

python3 "$EVAL" "$PCD" --plane 0 0 1 -1 --sensor-origin camera --output-json "$TMP_DIR/out.json"

python3 - "$TMP_DIR/out.json" <<'PY'
import json
import sys
with open(sys.argv[1]) as stream:
    data = json.load(stream)
assert data["schema"] == "scout_mapping_quality_eval_v1"
assert data["sensor_origin"] == "camera"
assert data["finite_points"] == 4
assert abs(data["plane_thickness_m"] - 0.037) < 0.002, data["plane_thickness_m"]
assert data["plane_p95_m"] > 0.0
PY

python3 "$EVAL" "$PCD" --output-csv "$TMP_DIR/out.csv" >/dev/null
grep -q 'finite_points' "$TMP_DIR/out.csv" || fail "CSV header missing finite_points"

echo "mapping quality evaluator tests: PASS"
