#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
VISUALIZER="$ROOT_DIR/catkin_ws/src/scout_pointcloud_accumulator/scripts/visualize_gps_lidar_route.py"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

cat > "$TMP_DIR/trajectory_gps_map.csv" <<'CSV'
sample_seq,recv_time_utc,accepted,tf_ok,latitude,longitude,altitude,map_x,map_y,map_z,hdop,sats,measurement_age_ms,doback_ok,doback_association_mode,doback_sample_count,doback_association_age_sec,doback_roll,doback_pitch,doback_si,doback_accmag
1,2026-07-20T10:00:00.000Z,1,1,38.000000,-4.000000,100.0,0.0,0.0,0.0,1.0,8,120,1,window_mean,10,0.05,1.2,-0.4,0.82,1001.0
2,2026-07-20T10:00:01.000Z,1,1,38.000010,-3.999990,100.1,5.0,0.0,0.0,1.1,9,130,1,window_mean,10,0.04,1.4,-0.5,0.78,1002.0
3,2026-07-20T10:00:02.000Z,1,1,38.000020,-3.999980,100.2,10.0,5.0,0.0,1.2,10,140,1,window_mean,10,0.03,1.6,-0.6,0.75,1003.0
4,2026-07-20T10:00:03.000Z,1,0,38.000030,-3.999970,100.3,,,,1.3,11,150,0,missing,0,,,,,
CSV

cat > "$TMP_DIR/map_ascii.pcd" <<'PCD'
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH 5
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 5
DATA ascii
0 0 0 1
2 1 0 2
4 2 0 3
8 4 0 4
10 5 0 5
PCD

python3 -m py_compile "$VISUALIZER"
python3 "$VISUALIZER" \
  --trajectory "$TMP_DIR/trajectory_gps_map.csv" \
  --pcd "$TMP_DIR/map_ascii.pcd" \
  --output "$TMP_DIR/view.html" \
  --max-points 3 >/dev/null

[ -s "$TMP_DIR/view.html" ] || fail "HTML viewer was not created"
grep -q 'AGV LiDAR GPS Route Viewer' "$TMP_DIR/view.html" || fail "viewer title missing"
grep -q '38.00002000' "$TMP_DIR/view.html" || fail "GPS latitude table missing"
grep -q '"route"' "$TMP_DIR/view.html" || fail "route payload missing"
grep -q '"cloud"' "$TMP_DIR/view.html" || fail "cloud payload missing"
grep -q 'Click a route point' "$TMP_DIR/view.html" || fail "interaction hint missing"
grep -q 'Doback (' "$TMP_DIR/view.html" || fail "Doback detail panel missing"
grep -q '"si": 0.75' "$TMP_DIR/view.html" || fail "Doback stability index missing"
grep -q '<th>SI</th>' "$TMP_DIR/view.html" || fail "Doback columns missing from trajectory table"

echo "gps lidar route visualizer tests: PASS"
