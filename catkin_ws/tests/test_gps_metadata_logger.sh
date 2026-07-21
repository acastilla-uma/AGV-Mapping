#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MODULE="$ROOT_DIR/catkin_ws/src/scout_pointcloud_accumulator/scripts/mapping_gps_metadata_logger.py"
PROBE="$ROOT_DIR/catkin_ws/scripts/lilygo_ble_probe.py"
BRIDGE="$ROOT_DIR/catkin_ws/scripts/lilygo_ble_tcp_bridge.py"
LAUNCH="$ROOT_DIR/catkin_ws/src/scout_pointcloud_accumulator/launch/mapping_gps_metadata.launch"
SAVE_SCRIPT="$ROOT_DIR/catkin_ws/scripts/save_accumulated_map.sh"
STOP_SCRIPT="$ROOT_DIR/catkin_ws/scripts/stop_lidar_mapping.sh"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

python3 - "$MODULE" <<'PY'
import importlib.util
import math
import sys

spec = importlib.util.spec_from_file_location("gps_logger", sys.argv[1])
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

_, gps = mod.parse_payload_line("sats=7 hdop=4.3 lat=38.047221 lon=-4.041935 age_ms=362 alt_m=13.1 fix_ok=1")
assert gps["latitude"] == 38.047221
assert gps["longitude"] == -4.041935
assert gps["sats"] == 7
assert gps["hdop"] == 4.3
assert gps["measurement_age_ms"] == 362.0
accepted, reason = mod.evaluate_quality(gps, min_sats=4, max_hdop=5.0, max_age_ms=2000.0, require_fix=True)
assert accepted and reason == ""

_, stale = mod.parse_payload_line('{"lat": 38.0, "lon": -4.0, "sats": 8, "hdop": 1.0, "age_ms": 3000, "fix_ok": true}')
accepted, reason = mod.evaluate_quality(stale, min_sats=4, max_hdop=5.0, max_age_ms=2000.0, require_fix=True)
assert not accepted and reason == "stale_age"

_, weak = mod.parse_payload_line("lat=38 lon=-4 sats=2 hdop=7.5 age_ms=100 fix_ok=0")
accepted, reason = mod.evaluate_quality(weak, min_sats=4, max_hdop=5.0, max_age_ms=2000.0, require_fix=True)
assert not accepted
assert set(reason.split(";")) == {"fix_not_valid", "low_sats", "high_hdop"}

_, missing_quality = mod.parse_payload_line("lat=38 lon=-4 fix_ok=1")
accepted, reason = mod.evaluate_quality(missing_quality, min_sats=4, max_hdop=5.0, max_age_ms=2000.0, require_fix=True)
assert not accepted
assert set(reason.split(";")) == {"missing_sats", "missing_hdop", "missing_age"}

_, malformed = mod.parse_payload_line(b"\xff\xfe lat=38 lon=-4 fix_ok=1")
accepted, reason = mod.evaluate_quality(malformed, min_sats=4, max_hdop=5.0, max_age_ms=2000.0, require_fix=True)
assert not accepted
assert "missing_sats" in reason

_, nmea = mod.parse_payload_line("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47")
assert math.isclose(nmea["latitude"], 48.1173, abs_tol=1e-6)
assert math.isclose(nmea["longitude"], 11.5166666667, abs_tol=1e-6)
assert nmea["fix_ok"] is True
accepted, reason = mod.evaluate_quality(nmea, min_sats=4, max_hdop=5.0, max_age_ms=2000.0, require_fix=True, require_age=False)
assert accepted and reason == ""

_, bad_checksum = mod.parse_payload_line("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00")
accepted, reason = mod.evaluate_quality(bad_checksum, min_sats=4, max_hdop=5.0, max_age_ms=2000.0, require_fix=True, require_age=False)
assert not accepted
assert "invalid_nmea_checksum" in reason

_, bridge_no_age = mod.parse_payload_line('{"time_utc": "2026-07-09T11:49:00.123+00:00", "text": "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"}')
accepted, reason = mod.evaluate_quality(bridge_no_age, min_sats=4, max_hdop=5.0, max_age_ms=2000.0, require_fix=True)
assert not accepted and reason == "missing_age"

_, bridge_with_age = mod.parse_payload_line('{"time_utc": "2026-07-09T11:49:00.123+00:00", "text": "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47", "age_ms": 100}')
accepted, reason = mod.evaluate_quality(bridge_with_age, min_sats=4, max_hdop=5.0, max_age_ms=2000.0, require_fix=True)
assert accepted and reason == ""

for text, expected in [
    ("lat=nan lon=-4 sats=8 hdop=1 age_ms=100 fix_ok=1", "missing_lat_lon"),
    ("lat=91 lon=-4 sats=8 hdop=1 age_ms=100 fix_ok=1", "latitude_out_of_range"),
    ("lat=38 lon=-181 sats=8 hdop=1 age_ms=100 fix_ok=1", "longitude_out_of_range"),
    ("lat=38 lon=-4 sats=8 hdop=nan age_ms=100 fix_ok=1", "missing_hdop"),
    ("lat=38 lon=-4 sats=8 hdop=-0.1 age_ms=100 fix_ok=1", "negative_hdop"),
    ("lat=38 lon=-4 sats=8 hdop=1 age_ms=-1 fix_ok=1", "negative_age"),
    ("lat=38 lon=-4 sats=8 hdop=1 age_ms=inf fix_ok=1", "missing_age"),
    ("lat=38 lon=-4 sats=inf hdop=1 age_ms=100 fix_ok=1", "missing_sats"),
]:
    _, bad = mod.parse_payload_line(text)
    accepted, reason = mod.evaluate_quality(bad, min_sats=4, max_hdop=5.0, max_age_ms=2000.0, require_fix=True)
    assert not accepted
    assert expected in reason, (text, reason)

assert mod.bool_param(True, False) is True
assert mod.bool_param(False, True) is False
assert mod.bool_param("true", False) is True
assert mod.bool_param("false", True) is False
assert mod.bool_param("", True) is True
assert mod.tcp_policy_error("127.0.0.1", set()) == ""
assert mod.tcp_policy_error("0.0.0.0", set()) != ""
assert mod.tcp_policy_error("0.0.0.0", {"192.168.8.10"}) == ""
assert mod.tcp_policy_error("192.168.8.174", set()) != ""
assert mod.tcp_policy_error("192.168.8.174", {"192.168.8.10"}) == ""
assert "Córdoba" in mod.to_text("Córdoba".encode("utf-8"))

ok, reason, age, lookup_time = mod.association_status(100.0, 98.5, 2.0)
assert ok and reason == "" and age == 1.5 and lookup_time == 98.5
ok, reason, age, lookup_time = mod.association_status(100.0, 97.0, 2.0)
assert not ok and reason == "association_age_exceeded"
ok, reason, age, lookup_time = mod.association_status(100.0, "", 2.0)
assert not ok and reason == "missing_estimated_measurement_time"
ok, reason, age, lookup_time = mod.association_status(100.0, 100.5, 2.0)
assert not ok and reason == "measurement_time_in_future"
assert mod.row_has_valid_fix({"fix_ok": "1"}) is True
assert mod.row_has_valid_fix({"fix_ok": "0"}) is False
assert mod.row_has_valid_fix({"fix_ok": ""}) is False
assert mod.selected_fix_stamp_sec({"estimated_measurement_ros_time": "98.5"}, 100.0) == 98.5
assert mod.selected_fix_stamp_sec({"estimated_measurement_ros_time": ""}, 100.0) == 100.0
assert mod.require_float("5.0", "value", 0.0) == 5.0
for bad_value in ("nan", "inf", "-1"):
    try:
        mod.require_float(bad_value, "value", 0.0)
    except ValueError:
        pass
    else:
        raise AssertionError("bad float accepted: " + bad_value)
assert mod.require_int("4", "value", 0) == 4
for bad_value in ("nan", "inf", "-1"):
    try:
        mod.require_int(bad_value, "value", 0)
    except ValueError:
        pass
    else:
        raise AssertionError("bad int accepted: " + bad_value)

values = [float(index) for index in range(1, 20)]
doback = mod.parse_doback_line("; ".join(str(value) for value in values) + "; ")
assert doback is not None
assert doback["ax"] == 1.0
assert doback["si"] == 16.0
assert doback["k3"] == 19.0
assert mod.parse_doback_line("ax; ay; az; gx; gy; gz; roll; pitch; yaw; timeantwifi; usciclo1; usciclo2; usciclo3; usciclo4; usciclo5; si; accmag; microsds; k3") is None
assert mod.parse_doback_line("12:34:56") is None

buffer = mod.DobackSampleBuffer()
for seq, stamp, offset in [(1, 100.1, 0.0), (2, 100.4, 2.0), (3, 100.8, 4.0)]:
    sample = {"doback_seq": seq, "recv_ros_time": stamp, "estimated_measurement_ros_time": stamp}
    sample.update({name: float(index) + offset for index, name in enumerate(mod.DOBACK_VALUE_FIELDS)})
    buffer.add(sample)
associated = buffer.associate(101.0, 2.0)
assert associated["doback_ok"] == "1"
assert associated["doback_association_mode"] == "window_mean"
assert associated["doback_sample_count"] == 3
assert math.isclose(associated["doback_ax"], 2.0)
reused = buffer.associate(101.5, 2.0)
assert reused["doback_association_mode"] == "latest"
assert reused["doback_sample_count"] == 1
assert reused["doback_ax"] == 4.0
stale = buffer.associate(103.0, 2.0)
assert stale["doback_ok"] == "0"
assert stale["doback_association_mode"] == "stale"
assert abs(abs(mod.circular_mean_deg([179.0, -179.0])) - 180.0) < 1e-6

late_buffer = mod.DobackSampleBuffer()
assert late_buffer.associate(100.5, 2.0)["doback_ok"] == "0"
for seq, stamp in [(1, 100.2), (2, 100.8)]:
    sample = {"doback_seq": seq, "recv_ros_time": 101.0, "estimated_measurement_ros_time": stamp}
    sample.update({name: float(seq) for name in mod.DOBACK_VALUE_FIELDS})
    late_buffer.add(sample)
late = late_buffer.associate(101.0, 2.0)
assert late["doback_sample_count"] == 1
assert late["doback_ax"] == 2.0

node = object.__new__(mod.GpsMetadataLogger)
node.lock = __import__("threading").Lock()
node.doback_seq = 0
node.doback_buffer = mod.DobackSampleBuffer()
node.doback_writer = None
node.doback_csv = None
row_a = mod.parse_doback_line("1;2;3;4;5;6;7;8;179;9;20000;20000;20000;20000;20000;0.8;1000;10;0.75;")
row_b = mod.parse_doback_line("2;3;4;5;6;7;8;9;-179;10;20000;20000;20000;20000;20000;0.7;1001;11;0.75;")
node.record_doback_batch([(row_a, 100.99), (row_b, 101.0)], "/dev/test", 101.0)
assert len(node.doback_buffer.samples) == 2
assert math.isclose(node.doback_buffer.samples[0]["estimated_measurement_ros_time"], 100.9)
assert math.isclose(node.doback_buffer.samples[1]["estimated_measurement_ros_time"], 101.0)
burst = node.doback_buffer.associate(101.0, 2.0)
assert burst["doback_sample_count"] == 2
assert abs(abs(burst["doback_yaw"]) - 180.0) < 1e-6
PY

python3 - "$PROBE" <<'PY'
import importlib.util
import sys

spec = importlib.util.spec_from_file_location("lilygo_probe", sys.argv[1])
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
assert mod.looks_like_gps_payload({"text": "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"})
assert mod.looks_like_gps_payload({"text": "lat=38 lon=-4 sats=8 hdop=1.2 fix_ok=1 age_ms=100"})
assert not mod.looks_like_gps_payload({"text": "hello from device"})
seen = {"any": False, "gps": False}
mod.mark_payload(seen, {"text": "hello from device"})
assert seen == {"any": True, "gps": False}
mod.mark_payload(seen, {"text": "lat=38 lon=-4 sats=8 hdop=1.2"})
assert seen == {"any": True, "gps": True}
PY

python3 - "$BRIDGE" <<'PY'
import asyncio
import importlib.util
import sys

spec = importlib.util.spec_from_file_location("lilygo_bridge", sys.argv[1])
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


class Forwarder(object):
    def __init__(self):
        self.sent = []
        self.closed = False

    def send_json(self, payload):
        self.sent.append(payload)
        return True

    def close(self):
        self.closed = True


async def check_sender_shutdown():
    forwarder = Forwarder()
    queue = asyncio.Queue()
    task = asyncio.ensure_future(mod.tcp_sender(queue, forwarder, None))
    await queue.put({"text": "ok"})
    await mod.stop_tcp_sender(queue, task, forwarder)
    assert forwarder.sent == [{"text": "ok"}]
    assert forwarder.closed
    assert task.done()


if hasattr(asyncio, "run"):
    asyncio.run(check_sender_shutdown())
else:
    loop = asyncio.get_event_loop()
    loop.run_until_complete(check_sender_shutdown())
PY

python3 - "$LAUNCH" <<'PY'
import sys
import xml.etree.ElementTree as ET

root = ET.parse(sys.argv[1]).getroot()
params = {item.attrib["name"]: item.attrib.get("value", item.attrib.get("default", "")) for item in root.iter("param")}
args = {item.attrib["name"]: item.attrib.get("default", "") for item in root.iter("arg")}
assert args["gps_tcp_bind"] == "127.0.0.1"
assert params["gps_min_sats"] == "$(arg gps_min_sats)"
assert params["gps_max_hdop"] == "$(arg gps_max_hdop)"
assert params["gps_max_age_ms"] == "$(arg gps_max_age_ms)"
assert params["gps_require_fix"] == "$(arg gps_require_fix)"
assert params["gps_require_sats"] == "$(arg gps_require_sats)"
assert params["gps_require_hdop"] == "$(arg gps_require_hdop)"
assert params["gps_require_age"] == "$(arg gps_require_age)"
assert params["gps_association_max_age_sec"] == "$(arg gps_association_max_age_sec)"
assert params["gps_tf_wait_timeout_sec"] == "$(arg gps_tf_wait_timeout_sec)"
assert params["gps_max_line_bytes"] == "$(arg gps_max_line_bytes)"
assert params["datum_mode"] == "$(arg datum_mode)"
assert args["doback_enabled"] == "true"
assert args["doback_port"] == "auto"
assert args["doback_baud"] == "115200"
assert params["doback_enabled"] == "$(arg doback_enabled)"
assert params["doback_port"] == "$(arg doback_port)"
assert params["doback_association_max_age_sec"] == "$(arg doback_association_max_age_sec)"
assert params["doback_probe_timeout_sec"] == "$(arg doback_probe_timeout_sec)"
PY

grep -q 'rospy.get_param("~gps_tcp_bind", "127.0.0.1")' "$MODULE" || fail "node tcp bind default is not loopback-safe"
grep -q 'rospy.get_param("~gps_tf_wait_timeout_sec", 0.5)' "$MODULE" || fail "TF wait timeout parameter missing"

cat > "$TMP_DIR/rosnode" <<'EOF'
#!/usr/bin/env bash
if [ "${1:-}" = list ]; then
  echo /accumulator_node
  echo /mapping_gps_metadata_logger
  exit 0
fi
if [ "${1:-}" = kill ]; then
  echo "kill $2" >> "$GPS_TEST_CALLS"
  exit 0
fi
exit 1
EOF
cat > "$TMP_DIR/rosservice" <<'EOF'
#!/usr/bin/env bash
echo "$*" >> "$GPS_TEST_CALLS"
exit 0
EOF
cat > "$TMP_DIR/timeout" <<'EOF'
#!/usr/bin/env bash
shift
"$@"
EOF
chmod +x "$TMP_DIR/rosnode" "$TMP_DIR/rosservice" "$TMP_DIR/timeout"

GPS_TEST_CALLS="$TMP_DIR/save.calls" \
ROSNODE_CMD="$TMP_DIR/rosnode" \
ROSSERVICE_CMD="$TMP_DIR/rosservice" \
TIMEOUT_CMD="$TMP_DIR/timeout" \
WORKSPACE="$ROOT_DIR/catkin_ws" \
bash "$SAVE_SCRIPT" >/dev/null
grep -q 'call /accumulator_node/save_accumulated' "$TMP_DIR/save.calls" || fail "accumulator save was not requested"
grep -q 'call /mapping_gps_metadata_logger/save_metadata' "$TMP_DIR/save.calls" || fail "GPS metadata save was not requested"

GPS_TEST_CALLS="$TMP_DIR/stop.calls" \
ROSNODE_CMD="$TMP_DIR/rosnode" \
ROSSERVICE_CMD="$TMP_DIR/rosservice" \
TIMEOUT_CMD="$TMP_DIR/timeout" \
WORKSPACE="$ROOT_DIR/catkin_ws" \
PID_FILE="$TMP_DIR/missing-pids" \
bash "$STOP_SCRIPT" >/dev/null
grep -q 'call /accumulator_node/save_accumulated' "$TMP_DIR/stop.calls" || fail "stop did not request accumulator save"
grep -q 'call /mapping_gps_metadata_logger/save_metadata' "$TMP_DIR/stop.calls" || fail "stop did not request GPS metadata save"
grep -q 'kill /mapping_gps_metadata_logger' "$TMP_DIR/stop.calls" || fail "stop did not gracefully kill GPS metadata node"

cat > "$TMP_DIR/rosservice_fail_gps" <<'EOF'
#!/usr/bin/env bash
echo "$*" >> "$GPS_TEST_CALLS"
if [ "$1" = call ] && [ "$2" = /mapping_gps_metadata_logger/save_metadata ]; then
  exit 7
fi
exit 0
EOF
chmod +x "$TMP_DIR/rosservice_fail_gps"

if GPS_TEST_CALLS="$TMP_DIR/save-fail.calls" \
  ROSNODE_CMD="$TMP_DIR/rosnode" \
  ROSSERVICE_CMD="$TMP_DIR/rosservice_fail_gps" \
  TIMEOUT_CMD="$TMP_DIR/timeout" \
  WORKSPACE="$ROOT_DIR/catkin_ws" \
  bash "$SAVE_SCRIPT" >/dev/null 2>&1; then
  fail "save script hid GPS metadata save failure"
fi
grep -q 'call /accumulator_node/save_accumulated' "$TMP_DIR/save-fail.calls" || fail "save failure test did not request accumulator save"
grep -q 'call /mapping_gps_metadata_logger/save_metadata' "$TMP_DIR/save-fail.calls" || fail "save failure test did not request GPS metadata save"

if GPS_TEST_CALLS="$TMP_DIR/stop-fail.calls" \
  ROSNODE_CMD="$TMP_DIR/rosnode" \
  ROSSERVICE_CMD="$TMP_DIR/rosservice_fail_gps" \
  TIMEOUT_CMD="$TMP_DIR/timeout" \
  WORKSPACE="$ROOT_DIR/catkin_ws" \
  PID_FILE="$TMP_DIR/missing-pids-fail" \
  bash "$STOP_SCRIPT" >/dev/null 2>&1; then
  fail "stop script hid GPS metadata save failure"
fi
grep -q 'call /mapping_gps_metadata_logger/save_metadata' "$TMP_DIR/stop-fail.calls" || fail "stop failure test did not request GPS metadata save"
grep -q 'kill /mapping_gps_metadata_logger' "$TMP_DIR/stop-fail.calls" || fail "stop failure test did not continue to stop GPS node"

echo "gps metadata logger tests: PASS"
