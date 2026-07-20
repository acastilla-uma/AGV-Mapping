#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
EXPORTER="$ROOT_DIR/catkin_ws/src/scout_pointcloud_accumulator/scripts/georeference_lidar_map.py"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

cat > "$TMP_DIR/trajectory_gps_map.csv" <<'CSV'
sample_seq,accepted,tf_ok,latitude,longitude,altitude,map_x,map_y,map_z,hdop,sats,fix_ok
1,1,1,38.000000,-4.000000,100.0,0.0,0.0,0.0,1.0,8,1
2,1,1,38.000000,-3.999900,100.0,10.0,0.0,0.0,1.0,8,1
3,1,1,38.000090,-4.000000,100.0,0.0,10.0,0.0,1.0,8,1
4,1,0,38.000045,-3.999950,100.0,,,,1.0,8,1
CSV

cat > "$TMP_DIR/map_points.csv" <<'CSV'
x,y,z,intensity
0,0,0,1
5,5,0,2
10,0,0,3
CSV

cat > "$TMP_DIR/map_points_ascii.pcd" <<'PCD'
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH 3
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 3
DATA ascii
0 0 0 1
5 5 0 2
10 0 0 3
PCD

python3 - "$TMP_DIR/map_points_binary.pcd" <<'PY'
import sys

header = (
    "# .PCD v0.7 - Point Cloud Data file format\n"
    "VERSION 0.7\n"
    "FIELDS x y z intensity\n"
    "SIZE 4 4 4 4\n"
    "TYPE F F F F\n"
    "COUNT 1 1 1 1\n"
    "WIDTH 1\n"
    "HEIGHT 1\n"
    "VIEWPOINT 0 0 0 1 0 0 0\n"
    "POINTS 1\n"
    "DATA binary\n"
).encode("ascii")
with open(sys.argv[1], "wb") as handle:
    handle.write(header)
    handle.write(b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x00")
PY

cat > "$TMP_DIR/bad_trajectory_nan.csv" <<'CSV'
sample_seq,accepted,tf_ok,latitude,longitude,altitude,map_x,map_y,map_z,hdop,sats,fix_ok
1,1,1,nan,-4.000000,100.0,0.0,0.0,0.0,1.0,8,1
2,1,1,inf,-3.999900,100.0,10.0,0.0,0.0,1.0,8,1
3,1,1,,,-,0.0,10.0,0.0,1.0,8,1
CSV

cat > "$TMP_DIR/bad_trajectory_z.csv" <<'CSV'
sample_seq,accepted,tf_ok,latitude,longitude,altitude,map_x,map_y,map_z,hdop,sats,fix_ok
1,1,1,38.000000,-4.000000,100.0,0.0,0.0,nan,1.0,8,1
2,1,1,38.000000,-3.999900,100.0,10.0,0.0,0.0,1.0,8,1
3,1,1,38.000090,-4.000000,100.0,0.0,10.0,0.0,1.0,8,1
CSV

cat > "$TMP_DIR/bad_points_z.csv" <<'CSV'
x,y,z,intensity
0,0,0,1
5,5,nan,2
10,0,0,3
CSV

python3 -m py_compile "$EXPORTER"
python3 "$EXPORTER" \
  --trajectory "$TMP_DIR/trajectory_gps_map.csv" \
  --points-csv "$TMP_DIR/map_points.csv" \
  --pcd "$TMP_DIR/map_points_ascii.pcd" \
  --allow-first-fix-datum \
  --output-dir "$TMP_DIR/out" \
  --output-prefix fixture >/dev/null

python3 "$EXPORTER" \
  --trajectory "$TMP_DIR/trajectory_gps_map.csv" \
  --pcd "$TMP_DIR/map_points_binary.pcd" \
  --allow-first-fix-datum \
  --output-dir "$TMP_DIR/out_binary" \
  --output-prefix fixture_binary >/dev/null

if python3 "$EXPORTER" \
  --trajectory "$TMP_DIR/trajectory_gps_map.csv" \
  --output-dir "$TMP_DIR/out_no_datum" \
  --output-prefix no_datum >/dev/null 2>&1; then
  fail "exploratory first-fix datum was accepted without explicit opt-in"
fi

if python3 "$EXPORTER" \
  --trajectory "$TMP_DIR/trajectory_gps_map.csv" \
  --datum-latitude 38.0 \
  --output-dir "$TMP_DIR/out_bad" \
  --output-prefix bad >/dev/null 2>&1; then
  fail "manual datum missing longitude was accepted"
fi

if python3 "$EXPORTER" \
  --trajectory "$TMP_DIR/trajectory_gps_map.csv" \
  --datum-latitude nan \
  --datum-longitude -4.0 \
  --output-dir "$TMP_DIR/out_bad_datum_nan" \
  --output-prefix bad_datum_nan >/dev/null 2>&1; then
  fail "non-finite manual datum was accepted"
fi

if python3 "$EXPORTER" \
  --trajectory "$TMP_DIR/trajectory_gps_map.csv" \
  --datum-latitude 91.0 \
  --datum-longitude -4.0 \
  --output-dir "$TMP_DIR/out_bad_datum_range" \
  --output-prefix bad_datum_range >/dev/null 2>&1; then
  fail "out-of-range manual datum was accepted"
fi

if python3 "$EXPORTER" \
  --trajectory "$TMP_DIR/bad_trajectory_nan.csv" \
  --allow-first-fix-datum \
  --output-dir "$TMP_DIR/out_bad_nan" \
  --output-prefix bad_nan >/dev/null 2>&1; then
  fail "non-finite trajectory coordinates were accepted"
fi

if python3 "$EXPORTER" \
  --trajectory "$TMP_DIR/bad_trajectory_z.csv" \
  --allow-first-fix-datum \
  --output-dir "$TMP_DIR/out_bad_z" \
  --output-prefix bad_z >/dev/null 2>&1; then
  fail "non-finite map z was accepted"
fi

if python3 "$EXPORTER" \
  --trajectory "$TMP_DIR/trajectory_gps_map.csv" \
  --points-csv "$TMP_DIR/bad_points_z.csv" \
  --allow-first-fix-datum \
  --output-dir "$TMP_DIR/out_bad_points_z" \
  --output-prefix bad_points_z >/dev/null 2>&1; then
  fail "non-finite point z was accepted"
fi

[ -s "$TMP_DIR/out/fixture_trajectory_enu.csv" ] || fail "trajectory ENU CSV missing"
[ -s "$TMP_DIR/out/fixture_points_enu.csv" ] || fail "points ENU CSV missing"
[ -s "$TMP_DIR/out/fixture_lidar_enu_ascii.pcd" ] || fail "ASCII PCD export missing"

python3 - "$TMP_DIR/out/fixture_georef_manifest.json" "$TMP_DIR/out/fixture_points_enu.csv" "$TMP_DIR/out/fixture_lidar_enu_ascii.pcd" <<'PY'
import csv
import json
import sys

with open(sys.argv[1]) as handle:
    manifest = json.load(handle)
assert manifest["schema"] == "agv_mapping_georef_export_v1"
assert manifest["datum"]["mode"] == "first_valid_fix"
assert manifest["transform"]["type"] == "2d_similarity_plus_vertical_offset"
assert manifest["transform"]["pair_count"] == 3
assert manifest["transform"]["rms_residual_m"] < 1.0
assert manifest["warnings"] == []

with open(sys.argv[2], newline="") as handle:
    rows = list(csv.DictReader(handle))
assert len(rows) == 3
assert {"enu_e", "enu_n", "enu_u"}.issubset(rows[0].keys())

with open(sys.argv[3]) as handle:
    text = handle.read()
assert "DATA ascii" in text
assert "-0.301613 0.301615" in text
PY

python3 - "$TMP_DIR/out_binary/fixture_binary_georef_manifest.json" <<'PY'
import json
import sys

with open(sys.argv[1]) as handle:
    manifest = json.load(handle)
assert manifest["warnings"] == ["pcd_not_exported: Only ASCII PCD is supported by this exporter"]
assert "lidar_enu_ascii_pcd" not in manifest["products"]
PY

echo "georeference exporter tests: PASS"
