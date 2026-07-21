#!/usr/bin/env python
from __future__ import print_function

import csv
import glob
import json
import math
import os
import socket
import sys
import threading
import time
from datetime import datetime

rospy = None
tf = None
RosPath = None
NavSatFix = None
NavSatStatus = None
PoseStamped = None
Empty = None
EmptyResponse = None
PY2 = sys.version_info[0] == 2
try:
    text_type = unicode
except NameError:
    text_type = str


def load_ros_modules():
    global rospy, tf, RosPath, NavSatFix, NavSatStatus, PoseStamped, Empty, EmptyResponse
    if rospy is not None:
        return
    import rospy as rospy_module
    import tf as tf_module
    from nav_msgs.msg import Path as RosPathMsg
    from sensor_msgs.msg import NavSatFix as NavSatFixMsg
    from sensor_msgs.msg import NavSatStatus as NavSatStatusMsg
    from geometry_msgs.msg import PoseStamped as PoseStampedMsg
    from std_srvs.srv import Empty as EmptySrv
    from std_srvs.srv import EmptyResponse as EmptyResponseSrv
    rospy = rospy_module
    tf = tf_module
    RosPath = RosPathMsg
    NavSatFix = NavSatFixMsg
    NavSatStatus = NavSatStatusMsg
    PoseStamped = PoseStampedMsg
    Empty = EmptySrv
    EmptyResponse = EmptyResponseSrv


GPS_FIELDS = [
    "sample_seq",
    "recv_ros_time",
    "recv_time_utc",
    "estimated_measurement_ros_time",
    "measurement_age_ms",
    "remote_host",
    "latitude",
    "longitude",
    "altitude",
    "sats",
    "hdop",
    "fix_ok",
    "accepted",
    "rejection_reason",
    "raw_text",
    "raw_hex",
]

TRAJECTORY_FIELDS = GPS_FIELDS + [
    "association_age_sec",
    "association_rejection_reason",
    "tf_lookup_ros_time",
    "tf_ok",
    "map_x",
    "map_y",
    "map_z",
    "map_roll",
    "map_pitch",
    "map_yaw",
]

DOBACK_VALUE_FIELDS = [
    "ax",
    "ay",
    "az",
    "gx",
    "gy",
    "gz",
    "roll",
    "pitch",
    "yaw",
    "timeantwifi",
    "usciclo1",
    "usciclo2",
    "usciclo3",
    "usciclo4",
    "usciclo5",
    "si",
    "accmag",
    "microsds",
    "k3",
]

DOBACK_FIELDS = [
    "doback_seq",
    "recv_ros_time",
    "estimated_measurement_ros_time",
    "batch_recv_ros_time",
    "recv_time_utc",
    "serial_port",
] + DOBACK_VALUE_FIELDS + ["raw_text"]

DOBACK_ASSOCIATION_FIELDS = [
    "doback_ok",
    "doback_association_mode",
    "doback_sample_count",
    "doback_first_measurement_ros_time",
    "doback_last_measurement_ros_time",
    "doback_association_age_sec",
] + ["doback_{}".format(name) for name in DOBACK_VALUE_FIELDS]

TRAJECTORY_FIELDS += DOBACK_ASSOCIATION_FIELDS


def utc_now():
    now = datetime.utcnow()
    return now.strftime("%Y-%m-%dT%H:%M:%S") + ".%03dZ" % int(now.microsecond / 1000)


def timestamp_slug():
    return datetime.utcnow().strftime("%Y%m%d_%H%M%S")


def safe_float(value):
    if value in (None, "", "?"):
        return None
    try:
        parsed = float(value)
        if math.isnan(parsed) or math.isinf(parsed):
            return None
        return parsed
    except (TypeError, ValueError):
        return None


def safe_int(value):
    if value in (None, "", "?"):
        return None
    try:
        parsed = safe_float(value)
        if parsed is None:
            return None
        return int(parsed)
    except (TypeError, ValueError, OverflowError):
        return None


def require_float(value, name, min_value=None, max_value=None):
    parsed = safe_float(value)
    if parsed is None:
        raise ValueError("{} must be a finite number".format(name))
    if min_value is not None and parsed < min_value:
        raise ValueError("{} must be >= {}".format(name, min_value))
    if max_value is not None and parsed > max_value:
        raise ValueError("{} must be <= {}".format(name, max_value))
    return parsed


def require_int(value, name, min_value=None, max_value=None):
    parsed = safe_int(value)
    if parsed is None:
        raise ValueError("{} must be a finite integer".format(name))
    if min_value is not None and parsed < min_value:
        raise ValueError("{} must be >= {}".format(name, min_value))
    if max_value is not None and parsed > max_value:
        raise ValueError("{} must be <= {}".format(name, max_value))
    return parsed


def to_text(value):
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", "replace")
    try:
        return text_type(value)
    except UnicodeDecodeError:
        raw = str(value)
        if isinstance(raw, bytes):
            return raw.decode("utf-8", "replace")
        return raw


def csv_row(row):
    out = {}
    for key, value in row.items():
        text = to_text(value)
        if PY2 and isinstance(text, text_type):
            out[key] = text.encode("utf-8")
        else:
            out[key] = text
    return out


def truthy(value):
    if value in (None, "", "?"):
        return None
    text = to_text(value).strip().lower()
    if text in ("1", "true", "yes", "ok", "fix", "valid", "a"):
        return True
    if text in ("0", "false", "no", "none", "invalid", "v"):
        return False
    return None


def bool_param(value, default=False):
    parsed = truthy(value)
    if parsed is None:
        return default
    return parsed


def parse_doback_line(line):
    """Parse one firmware data row; non-data/status lines return None."""
    text = to_text(line).strip()
    if ";" not in text:
        return None
    values = [part.strip() for part in text.split(";")]
    while values and values[-1] == "":
        values.pop()
    if len(values) != len(DOBACK_VALUE_FIELDS):
        return None
    parsed = {}
    for name, value in zip(DOBACK_VALUE_FIELDS, values):
        number = safe_float(value)
        if number is None:
            return None
        parsed[name] = number
    parsed["raw_text"] = text
    return parsed


def find_doback_ports():
    ports = []
    devices = set()
    for pattern in ("/dev/serial/by-id/*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        for path in sorted(glob.glob(pattern)):
            device = os.path.realpath(path)
            if device not in devices:
                devices.add(device)
                ports.append(path)
    return ports


def doback_row_duration_sec(sample):
    duration = sum(sample[name] for name in ("usciclo1", "usciclo2", "usciclo3", "usciclo4", "usciclo5")) / 1000000.0
    if duration <= 0.0 or duration > 5.0:
        return 0.1
    return duration


def circular_mean_deg(values):
    radians = [math.radians(value) for value in values]
    sine = sum(math.sin(value) for value in radians) / float(len(radians))
    cosine = sum(math.cos(value) for value in radians) / float(len(radians))
    return math.degrees(math.atan2(sine, cosine))


def validate_csv_header(path, fieldnames):
    if not os.path.isfile(path) or os.path.getsize(path) == 0:
        return
    with open(path, "r") as handle:
        reader = csv.reader(handle)
        existing = next(reader, [])
    if existing != list(fieldnames):
        raise RuntimeError("CSV schema mismatch in {}; use a new SESSION_DIR".format(path))


def empty_doback_association(mode="missing"):
    row = dict((field, "") for field in DOBACK_ASSOCIATION_FIELDS)
    row.update({
        "doback_ok": "0",
        "doback_association_mode": mode,
        "doback_sample_count": 0,
    })
    return row


class DobackSampleBuffer(object):
    """Associate received Doback samples with GPS receive-time windows."""

    def __init__(self, max_samples=10000):
        self.max_samples = max_samples
        self.samples = []
        self.latest = None
        self.last_consumed_seq = 0
        self.last_association_time = None
        self.lock = threading.Lock()

    def add(self, sample):
        with self.lock:
            item = dict(sample)
            self.samples.append(item)
            self.latest = item
            if self.max_samples > 0 and len(self.samples) > self.max_samples:
                self.samples = self.samples[-self.max_samples:]

    def associate(self, gps_recv_ros_time, max_age_sec):
        current = float(gps_recv_ros_time)
        cutoff = current - float(max_age_sec)
        with self.lock:
            previous_association_time = self.last_association_time
            self.last_association_time = current
            received = [
                sample for sample in self.samples
                if sample["doback_seq"] > self.last_consumed_seq
                and sample["estimated_measurement_ros_time"] <= current
            ]
            if received:
                self.last_consumed_seq = max(sample["doback_seq"] for sample in received)
            selected = [
                sample for sample in received
                if sample["estimated_measurement_ros_time"] >= cutoff
                and (previous_association_time is None or sample["estimated_measurement_ros_time"] > previous_association_time)
            ]
            latest = dict(self.latest) if self.latest is not None else None

        if selected:
            mode = "window_mean" if len(selected) > 1 else "window_sample"
            result = {
                "doback_ok": "1",
                "doback_association_mode": mode,
                "doback_sample_count": len(selected),
                "doback_first_measurement_ros_time": selected[0]["estimated_measurement_ros_time"],
                "doback_last_measurement_ros_time": selected[-1]["estimated_measurement_ros_time"],
                "doback_association_age_sec": current - selected[-1]["estimated_measurement_ros_time"],
            }
            for name in DOBACK_VALUE_FIELDS:
                values = [sample[name] for sample in selected]
                result["doback_{}".format(name)] = circular_mean_deg(values) if name == "yaw" else sum(values) / float(len(values))
            return result

        if latest is None or latest["estimated_measurement_ros_time"] > current:
            return empty_doback_association("missing")
        age_sec = current - latest["estimated_measurement_ros_time"]
        if age_sec > float(max_age_sec):
            return empty_doback_association("stale")
        result = {
            "doback_ok": "1",
            "doback_association_mode": "latest",
            "doback_sample_count": 1,
            "doback_first_measurement_ros_time": latest["estimated_measurement_ros_time"],
            "doback_last_measurement_ros_time": latest["estimated_measurement_ros_time"],
            "doback_association_age_sec": age_sec,
        }
        for name in DOBACK_VALUE_FIELDS:
            result["doback_{}".format(name)] = latest[name]
        return result


def parse_key_value_status(text):
    parsed = {}
    for token in to_text(text).replace("\r", " ").replace("\n", " ").split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        parsed[key.strip()] = value.strip()
    return parsed


def parse_nmea_latlon(raw_value, hemi):
    if raw_value in (None, "", "?") or hemi in (None, "", "?"):
        return None
    try:
        value = float(raw_value)
        degrees = int(value / 100)
        minutes = value - degrees * 100
        decimal = degrees + minutes / 60.0
        if to_text(hemi).upper() in ("S", "W"):
            decimal *= -1.0
        return decimal
    except (TypeError, ValueError):
        return None


def nmea_checksum_valid(text):
    text = to_text(text).strip()
    if "*" not in text:
        return True
    body, checksum = text[1:].split("*", 1) if text.startswith("$") else text.split("*", 1)
    checksum = checksum[:2]
    try:
        expected = int(checksum, 16)
    except ValueError:
        return False
    actual = 0
    for char in body:
        actual ^= ord(char)
    return actual == expected


def parse_nmea(text):
    text = to_text(text).strip()
    if not text.startswith("$"):
        return {}
    if not nmea_checksum_valid(text):
        return {"invalid_nmea_checksum": True}
    text = text.split("*", 1)[0]
    fields = text.split(",")
    sentence = fields[0][-3:]
    out = {}
    if sentence == "GGA" and len(fields) >= 10:
        out["latitude"] = parse_nmea_latlon(fields[2], fields[3])
        out["longitude"] = parse_nmea_latlon(fields[4], fields[5])
        out["fix_ok"] = bool(safe_int(fields[6]))
        out["sats"] = safe_int(fields[7])
        out["hdop"] = safe_float(fields[8])
        out["altitude"] = safe_float(fields[9])
    elif sentence == "RMC" and len(fields) >= 10:
        out["latitude"] = parse_nmea_latlon(fields[3], fields[4])
        out["longitude"] = parse_nmea_latlon(fields[5], fields[6])
        out["fix_ok"] = fields[2] == "A"
    return out


def parse_payload_line(line):
    text = to_text(line).strip()
    payload = {}
    try:
        decoded = json.loads(text)
        if isinstance(decoded, dict):
            payload = decoded
        else:
            payload = {"text": text}
    except ValueError:
        payload = {"text": text}

    raw_text = payload.get("text", text) or ""
    parsed = payload.get("parsed", {})
    if not isinstance(parsed, dict):
        parsed = {}
    kv = parse_key_value_status(raw_text)
    data = {}
    data.update(parse_nmea(raw_text))
    invalid_nmea_checksum = data.pop("invalid_nmea_checksum", False)
    data.update(extract_aliases(payload))
    data.update(extract_aliases(parsed))
    data.update(extract_aliases(kv))
    if "fix_ok" not in data:
        waiting = truthy(parsed.get("waiting_for_fix", kv.get("waiting_for_fix")))
        if waiting is True:
            data["fix_ok"] = False
        elif "sentences_fix" in parsed or "sentences_fix" in kv:
            data["fix_ok"] = bool(safe_int(parsed.get("sentences_fix", kv.get("sentences_fix"))))
    data.setdefault("raw_text", raw_text)
    data.setdefault("raw_hex", payload.get("raw_hex", ""))
    if invalid_nmea_checksum:
        data["invalid_nmea_checksum"] = True
    return payload, normalize_gps(data)


def extract_aliases(source):
    out = {}
    for key in ("lat", "latitude"):
        if key in source:
            out["latitude"] = safe_float(source.get(key))
    for key in ("lon", "lng", "longitude"):
        if key in source:
            out["longitude"] = safe_float(source.get(key))
    for key in ("alt", "alt_m", "altitude"):
        if key in source:
            out["altitude"] = safe_float(source.get(key))
    for key in ("sats", "satellites", "satelites"):
        if key in source:
            out["sats"] = safe_int(source.get(key))
    if "hdop" in source:
        out["hdop"] = safe_float(source.get("hdop"))
    for key in ("fix_ok", "fix", "fix_valido"):
        if key in source:
            out["fix_ok"] = truthy(source.get(key))
    for key in ("age_ms", "measurement_age_ms"):
        if key in source:
            out["measurement_age_ms"] = safe_float(source.get(key))
    return out


def normalize_gps(data):
    return {
        "latitude": data.get("latitude"),
        "longitude": data.get("longitude"),
        "altitude": data.get("altitude"),
        "sats": data.get("sats"),
        "hdop": data.get("hdop"),
        "fix_ok": data.get("fix_ok"),
        "measurement_age_ms": data.get("measurement_age_ms"),
        "invalid_nmea_checksum": data.get("invalid_nmea_checksum", False),
        "raw_text": data.get("raw_text", ""),
        "raw_hex": data.get("raw_hex", ""),
    }


def evaluate_quality(gps, min_sats, max_hdop, max_age_ms, require_fix, require_sats=True, require_hdop=True, require_age=True):
    reasons = []
    lat = gps.get("latitude")
    lon = gps.get("longitude")
    if lat is None or lon is None:
        reasons.append("missing_lat_lon")
    else:
        if lat < -90.0 or lat > 90.0:
            reasons.append("latitude_out_of_range")
        if lon < -180.0 or lon > 180.0:
            reasons.append("longitude_out_of_range")
    if require_fix and gps.get("fix_ok") is not True:
        reasons.append("fix_not_valid")
    if gps.get("invalid_nmea_checksum"):
        reasons.append("invalid_nmea_checksum")
    sats = gps.get("sats")
    if require_sats and sats is None:
        reasons.append("missing_sats")
    elif sats is not None and sats < min_sats:
        reasons.append("low_sats")
    hdop = gps.get("hdop")
    if require_hdop and hdop is None:
        reasons.append("missing_hdop")
    elif hdop is not None:
        if hdop < 0:
            reasons.append("negative_hdop")
        elif hdop > max_hdop:
            reasons.append("high_hdop")
    age_ms = gps.get("measurement_age_ms")
    if require_age and age_ms is None:
        reasons.append("missing_age")
    elif age_ms is not None:
        if age_ms < 0:
            reasons.append("negative_age")
        elif age_ms > max_age_ms:
            reasons.append("stale_age")
    return len(reasons) == 0, ";".join(reasons)


def bind_exposes_network(bind):
    text = to_text(bind).strip()
    return text in ("", "0.0.0.0", "::", "*")


def bind_is_loopback(bind):
    text = to_text(bind).strip().lower()
    return text in ("127.0.0.1", "localhost", "::1")


def tcp_policy_error(bind, allowed_hosts):
    if not bind_is_loopback(bind) and not allowed_hosts:
        return "gps_allowed_hosts must be non-empty when gps_tcp_bind is not loopback"
    return ""


def association_status(recv_ros_sec, estimated_measurement_ros_time, max_age_sec):
    measurement_ros_sec = safe_float(estimated_measurement_ros_time)
    if measurement_ros_sec is None:
        return False, "missing_estimated_measurement_time", "", ""
    age_sec = float(recv_ros_sec) - measurement_ros_sec
    if age_sec < -0.001:
        return False, "measurement_time_in_future", age_sec, measurement_ros_sec
    if age_sec > float(max_age_sec):
        return False, "association_age_exceeded", age_sec, measurement_ros_sec
    return True, "", age_sec, measurement_ros_sec


def row_has_valid_fix(row):
    return truthy(row.get("fix_ok")) is True


def hdop_covariance(hdop):
    if hdop is None:
        return [0.0] * 9, NavSatFix.COVARIANCE_TYPE_UNKNOWN if NavSatFix else 0
    variance = float(hdop) * float(hdop)
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance * 4.0], (
        NavSatFix.COVARIANCE_TYPE_APPROXIMATED if NavSatFix else 1
    )


class GpsMetadataLogger(object):
    def __init__(self):
        load_ros_modules()
        self.output_pcd = rospy.get_param("~output_pcd", "/tmp/accumulated_cloud.pcd")
        self.metadata_dir = rospy.get_param("~metadata_dir", "")
        self.target_frame = rospy.get_param("~target_frame", "map").lstrip("/")
        self.robot_frame = rospy.get_param("~robot_frame", "base_link").lstrip("/")
        self.gps_frame = rospy.get_param("~gps_frame", "gps_link").lstrip("/")
        self.tcp_bind = rospy.get_param("~gps_tcp_bind", "127.0.0.1")
        self.tcp_port = require_int(rospy.get_param("~gps_tcp_port", 29500), "~gps_tcp_port", 1, 65535)
        self.allowed_hosts = self.parse_hosts(rospy.get_param("~gps_allowed_hosts", ""))
        self.min_sats = require_int(rospy.get_param("~gps_min_sats", 4), "~gps_min_sats", 0)
        self.max_hdop = require_float(rospy.get_param("~gps_max_hdop", 5.0), "~gps_max_hdop", 0.0)
        self.max_age_ms = require_float(rospy.get_param("~gps_max_age_ms", 2000.0), "~gps_max_age_ms", 0.0)
        self.require_fix = bool_param(rospy.get_param("~gps_require_fix", True), True)
        self.require_sats = bool_param(rospy.get_param("~gps_require_sats", True), True)
        self.require_hdop = bool_param(rospy.get_param("~gps_require_hdop", True), True)
        self.require_age = bool_param(rospy.get_param("~gps_require_age", True), True)
        self.association_max_age_sec = require_float(
            rospy.get_param("~gps_association_max_age_sec", 2.0), "~gps_association_max_age_sec", 0.0
        )
        self.tf_wait_timeout_sec = require_float(rospy.get_param("~gps_tf_wait_timeout_sec", 0.5), "~gps_tf_wait_timeout_sec", 0.0)
        self.max_line_bytes = require_int(rospy.get_param("~gps_max_line_bytes", 8192), "~gps_max_line_bytes", 1)
        self.path_topic = rospy.get_param("~trajectory_path_topic", "/gps_map_trajectory_path")
        self.path_max_poses = require_int(rospy.get_param("~trajectory_max_points", 10000), "~trajectory_max_points", 0)
        self.doback_enabled = bool_param(rospy.get_param("~doback_enabled", True), True)
        self.doback_port = to_text(rospy.get_param("~doback_port", "auto")).strip() or "auto"
        self.doback_baud = require_int(rospy.get_param("~doback_baud", 115200), "~doback_baud", 1)
        self.doback_association_max_age_sec = require_float(
            rospy.get_param("~doback_association_max_age_sec", 2.0), "~doback_association_max_age_sec", 0.0
        )
        self.doback_reconnect_sec = require_float(
            rospy.get_param("~doback_reconnect_sec", 2.0), "~doback_reconnect_sec", 0.1
        )
        self.doback_probe_timeout_sec = require_float(
            rospy.get_param("~doback_probe_timeout_sec", 5.0), "~doback_probe_timeout_sec", 0.5
        )
        self.doback_max_line_bytes = require_int(
            rospy.get_param("~doback_max_line_bytes", 65536), "~doback_max_line_bytes", 128
        )
        self.datum_mode = rospy.get_param("~datum_mode", "first_valid_fix")
        self.datum_latitude = self.optional_float_param("~datum_latitude")
        self.datum_longitude = self.optional_float_param("~datum_longitude")
        self.datum_altitude = self.optional_float_param("~datum_altitude")

        self.lock = threading.Lock()
        self.gps_record_lock = threading.Lock()
        self.shutdown_event = threading.Event()
        self.server_socket = None
        self.doback_serial = None
        self.doback_serial_thread = None
        self.client_sockets = set()
        self.client_threads = []
        self.sample_seq = 0
        self.accepted_count = 0
        self.rejected_count = 0
        self.parse_error_count = 0
        self.doback_seq = 0
        self.doback_line_count = 0
        self.doback_ignored_line_count = 0
        self.doback_reconnect_count = 0
        self.doback_active_port = ""
        self.doback_buffer = DobackSampleBuffer()
        self.path_poses = []
        policy_error = tcp_policy_error(self.tcp_bind, self.allowed_hosts)
        if policy_error:
            rospy.logerr(policy_error)
            raise RuntimeError(policy_error)

        self.session_dir = self.resolve_metadata_dir()
        self.gps_csv_path = os.path.join(self.session_dir, "gps.csv")
        self.gps_raw_path = os.path.join(self.session_dir, "gps_raw.jsonl")
        self.trajectory_path = os.path.join(self.session_dir, "trajectory_gps_map.csv")
        self.doback_csv_path = os.path.join(self.session_dir, "doback.csv")
        self.manifest_path = os.path.join(self.session_dir, "manifest.json")
        self.ensure_parent(self.gps_csv_path)
        validate_csv_header(self.gps_csv_path, GPS_FIELDS)
        validate_csv_header(self.trajectory_path, TRAJECTORY_FIELDS)
        self.gps_csv = open(self.gps_csv_path, "a")
        self.gps_writer = csv.DictWriter(self.gps_csv, fieldnames=GPS_FIELDS)
        if os.path.getsize(self.gps_csv_path) == 0:
            self.gps_writer.writeheader()
        self.trajectory_csv = open(self.trajectory_path, "a")
        self.trajectory_writer = csv.DictWriter(self.trajectory_csv, fieldnames=TRAJECTORY_FIELDS)
        if os.path.getsize(self.trajectory_path) == 0:
            self.trajectory_writer.writeheader()
        self.gps_raw = open(self.gps_raw_path, "a")
        self.doback_csv = None
        self.doback_writer = None
        if self.doback_enabled:
            validate_csv_header(self.doback_csv_path, DOBACK_FIELDS)
            self.doback_csv = open(self.doback_csv_path, "a")
            self.doback_writer = csv.DictWriter(self.doback_csv, fieldnames=DOBACK_FIELDS)
            if os.path.getsize(self.doback_csv_path) == 0:
                self.doback_writer.writeheader()

        self.fix_pub = rospy.Publisher("/gps/fix", NavSatFix, queue_size=10)
        self.path_pub = rospy.Publisher(self.path_topic, RosPath, queue_size=1, latch=True)
        self.listener = tf.TransformListener()
        self.save_srv = rospy.Service("~save_metadata", Empty, self.save_service)
        self.server_thread = threading.Thread(target=self.tcp_server)
        self.server_thread.daemon = True
        self.server_thread.start()
        rospy.loginfo("GPS metadata sidecar listening on %s:%d", self.tcp_bind, self.tcp_port)
        if self.doback_enabled:
            self.doback_serial_thread = threading.Thread(target=self.doback_serial_loop)
            self.doback_serial_thread.daemon = True
            self.doback_serial_thread.start()
            rospy.loginfo("Doback serial integration enabled: port=%s baud=%d", self.doback_port, self.doback_baud)

    @staticmethod
    def parse_hosts(value):
        if isinstance(value, list):
            return set(str(item).strip() for item in value if str(item).strip())
        return set(part.strip() for part in str(value).split(",") if part.strip())

    @staticmethod
    def ensure_parent(path):
        directory = os.path.dirname(path)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)

    def optional_float_param(self, name):
        value = rospy.get_param(name, "")
        return safe_float(value)

    def resolve_metadata_dir(self):
        if self.metadata_dir:
            path = self.metadata_dir
        else:
            workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
            session_name = "gps_metadata_" + timestamp_slug()
            path = os.path.join(workspace_root, "maps", session_name)
        if not os.path.isdir(path):
            os.makedirs(path)
        return path

    def requested_doback_ports(self):
        if self.doback_port.lower() != "auto":
            return [self.doback_port]
        return find_doback_ports()

    def doback_serial_loop(self):
        try:
            import serial
        except ImportError as exc:
            rospy.logerr("Doback serial integration needs pyserial/python-serial: %s", exc)
            return

        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            ports = self.requested_doback_ports()
            if not ports:
                rospy.logwarn_throttle(15.0, "Doback enabled but no /dev/serial/by-id, ttyACM or ttyUSB port is available")
                self.shutdown_event.wait(self.doback_reconnect_sec)
                continue

            validated_any = False
            for port in ports:
                if self.shutdown_event.is_set() or rospy.is_shutdown():
                    break
                try:
                    device = serial.Serial(port=port, baudrate=self.doback_baud, timeout=1.0)
                except Exception as exc:
                    rospy.logwarn_throttle(15.0, "Cannot open Doback serial port %s: %s", port, exc)
                    continue

                with self.lock:
                    self.doback_serial = device
                auto_probe = self.doback_port.lower() == "auto"
                probe_deadline = time.time() + self.doback_probe_timeout_sec
                validated = False
                pending = []
                try:
                    while not rospy.is_shutdown() and not self.shutdown_event.is_set():
                        raw = device.readline(self.doback_max_line_bytes + 1)
                        if not raw:
                            if auto_probe and not validated and time.time() >= probe_deadline:
                                break
                            if not auto_probe and not validated:
                                rospy.logwarn_throttle(15.0, "Doback port %s is open but has not emitted a valid 19-field row", port)
                            continue
                        if len(raw) > self.doback_max_line_bytes:
                            with self.lock:
                                self.doback_ignored_line_count += 1
                            rospy.logwarn_throttle(10.0, "Oversized Doback serial line dropped")
                            continue
                        text = raw.decode("utf-8", "replace") if isinstance(raw, bytes) else raw
                        recv_ros_sec = rospy.Time.now().to_sec()
                        parsed = parse_doback_line(text)
                        with self.lock:
                            self.doback_line_count += 1
                            if parsed is None:
                                self.doback_ignored_line_count += 1
                        if parsed is not None:
                            pending.append((parsed, recv_ros_sec))
                            if not validated:
                                validated = True
                                validated_any = True
                                with self.lock:
                                    self.doback_active_port = port
                                    self.doback_reconnect_count += 1
                                rospy.loginfo("Doback serial validated: %s at %d baud", port, self.doback_baud)
                            if len(pending) >= 10:
                                self.record_doback_batch(pending, port, recv_ros_sec)
                                pending = []
                        elif pending:
                            self.record_doback_batch(pending, port, pending[-1][1])
                            pending = []
                        if auto_probe and not validated and time.time() >= probe_deadline:
                            break
                except Exception as exc:
                    if not self.shutdown_event.is_set() and not rospy.is_shutdown():
                        rospy.logwarn("Doback serial port %s disconnected: %s", port, exc)
                finally:
                    if pending and validated:
                        self.record_doback_batch(pending, port, pending[-1][1])
                    try:
                        device.close()
                    except Exception:
                        pass
                    with self.lock:
                        if self.doback_serial is device:
                            self.doback_serial = None
                            self.doback_active_port = ""
                if validated:
                    break
                rospy.logwarn_throttle(
                    15.0,
                    "Serial candidate %s did not emit a valid Doback row within %.1f s",
                    port,
                    self.doback_probe_timeout_sec,
                )

            if not validated_any:
                rospy.logwarn_throttle(15.0, "No serial candidate emitted valid Doback data: %s", ", ".join(ports))
            self.shutdown_event.wait(self.doback_reconnect_sec)

    def record_doback_line(self, line, port):
        parsed = parse_doback_line(line)
        with self.lock:
            self.doback_line_count += 1
            if parsed is None:
                self.doback_ignored_line_count += 1
                return False
        recv_ros_sec = rospy.Time.now().to_sec()
        self.record_doback_batch([(parsed, recv_ros_sec)], port, recv_ros_sec)
        return True

    def record_doback_batch(self, pending, port, batch_recv_ros_time):
        measurement_times = [0.0] * len(pending)
        measurement_time = float(batch_recv_ros_time)
        for index in range(len(pending) - 1, -1, -1):
            measurement_times[index] = measurement_time
            measurement_time -= doback_row_duration_sec(pending[index][0])

        samples = []
        for index, (parsed, recv_ros_sec) in enumerate(pending):
            with self.lock:
                self.doback_seq += 1
                doback_seq = self.doback_seq
            sample = {
                "doback_seq": doback_seq,
                "recv_ros_time": recv_ros_sec,
                "estimated_measurement_ros_time": measurement_times[index],
                "batch_recv_ros_time": batch_recv_ros_time,
                "recv_time_utc": utc_now(),
                "serial_port": port,
            }
            sample.update(parsed)
            samples.append(sample)
            self.doback_buffer.add(sample)

        with self.lock:
            if self.doback_writer is not None:
                for sample in samples:
                    self.doback_writer.writerow(csv_row(sample))
                self.doback_csv.flush()

    def tcp_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket = server
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.tcp_bind, self.tcp_port))
        server.listen(5)
        server.settimeout(1.0)
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            except Exception as exc:
                if self.shutdown_event.is_set() or rospy.is_shutdown():
                    break
                rospy.logwarn("GPS TCP accept failed: %s", exc)
                continue
            thread = threading.Thread(target=self.handle_client, args=(conn, addr))
            thread.daemon = True
            with self.lock:
                self.client_threads.append(thread)
            thread.start()
        try:
            server.close()
        except Exception:
            pass

    def host_allowed(self, host):
        return not self.allowed_hosts or host in self.allowed_hosts

    def handle_client(self, conn, addr):
        host = addr[0]
        if not self.host_allowed(host):
            rospy.logwarn("Rejected GPS TCP client from %s", host)
            conn.close()
            return
        rospy.loginfo("Accepted GPS TCP client from %s", host)
        conn.settimeout(1.0)
        with self.lock:
            self.client_sockets.add(conn)
        buf = b""
        try:
            while not rospy.is_shutdown() and not self.shutdown_event.is_set():
                try:
                    chunk = conn.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buf += chunk
                if len(buf) > self.max_line_bytes:
                    with self.lock:
                        self.parse_error_count += 1
                    rospy.logwarn("GPS TCP payload from %s exceeded %d bytes without newline; closing client", host, self.max_line_bytes)
                    break
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if len(line) > self.max_line_bytes:
                        with self.lock:
                            self.parse_error_count += 1
                        rospy.logwarn("GPS TCP line from %s exceeded %d bytes; dropped", host, self.max_line_bytes)
                        continue
                    text = line.decode("utf-8", errors="replace").strip()
                    if text:
                        self.record_line(text, host)
        except Exception as exc:
            rospy.logwarn("GPS TCP client %s error: %s", host, exc)
        finally:
            with self.lock:
                self.client_sockets.discard(conn)
            try:
                conn.close()
            except Exception:
                pass
            rospy.loginfo("GPS TCP client %s disconnected", host)

    def record_line(self, line, remote_host):
        with self.gps_record_lock:
            self._record_line_serialized(line, remote_host)

    def _record_line_serialized(self, line, remote_host):
        recv_ros_time = rospy.Time.now()
        recv_ros_sec = recv_ros_time.to_sec()
        recv_time_utc = utc_now()
        with self.lock:
            self.sample_seq += 1
            sample_seq = self.sample_seq

        try:
            payload, gps = parse_payload_line(line)
        except Exception as exc:
            payload = {"text": line}
            gps = normalize_gps({"raw_text": line})
            self.parse_error_count += 1
            rospy.logwarn("GPS parse error: %s", exc)
        accepted, reason = evaluate_quality(
            gps,
            self.min_sats,
            self.max_hdop,
            self.max_age_ms,
            self.require_fix,
            self.require_sats,
            self.require_hdop,
            self.require_age,
        )
        estimated = ""
        if gps.get("measurement_age_ms") is not None:
            estimated = recv_ros_sec - gps.get("measurement_age_ms") / 1000.0

        row = {
            "sample_seq": sample_seq,
            "recv_ros_time": recv_ros_sec,
            "recv_time_utc": recv_time_utc,
            "estimated_measurement_ros_time": estimated,
            "measurement_age_ms": value_or_empty(gps.get("measurement_age_ms")),
            "remote_host": remote_host,
            "latitude": value_or_empty(gps.get("latitude")),
            "longitude": value_or_empty(gps.get("longitude")),
            "altitude": value_or_empty(gps.get("altitude")),
            "sats": value_or_empty(gps.get("sats")),
            "hdop": value_or_empty(gps.get("hdop")),
            "fix_ok": "1" if gps.get("fix_ok") is True else "0",
            "accepted": "1" if accepted else "0",
            "rejection_reason": reason,
            "raw_text": gps.get("raw_text", ""),
            "raw_hex": gps.get("raw_hex", ""),
        }
        raw_record = dict(payload)
        raw_record.update(row)

        with self.lock:
            self.gps_raw.write(json.dumps(raw_record, sort_keys=True) + "\n")
            self.gps_raw.flush()
            self.gps_writer.writerow(csv_row(row))
            self.gps_csv.flush()
            if accepted:
                self.accepted_count += 1
            else:
                self.rejected_count += 1

        if accepted:
            fix_stamp = rospy.Time.from_sec(selected_fix_stamp_sec(row, recv_ros_sec))
            self.publish_fix(row, fix_stamp)
            self.record_trajectory(row, recv_ros_time)
        else:
            rospy.logwarn_throttle(5.0, "Rejected GPS sample %s: %s", sample_seq, reason)

    def publish_fix(self, row, stamp):
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = self.gps_frame
        msg.status.status = NavSatStatus.STATUS_FIX if row_has_valid_fix(row) else NavSatStatus.STATUS_NO_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = float(row["latitude"])
        msg.longitude = float(row["longitude"])
        msg.altitude = float(row["altitude"]) if row["altitude"] != "" else float("nan")
        covariance, cov_type = hdop_covariance(safe_float(row["hdop"]))
        msg.position_covariance = covariance
        msg.position_covariance_type = cov_type
        self.fix_pub.publish(msg)

    def lookup_pose(self, stamp):
        self.listener.waitForTransform(
            self.target_frame,
            self.robot_frame,
            stamp,
            rospy.Duration(max(0.0, self.tf_wait_timeout_sec)),
        )
        trans, rot = self.listener.lookupTransform(self.target_frame, self.robot_frame, stamp)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
        return trans, rot, roll, pitch, yaw

    def record_trajectory(self, row, stamp):
        trajectory_row = dict(row)
        trajectory_row.update({
            "association_age_sec": "",
            "association_rejection_reason": "",
            "tf_lookup_ros_time": "",
            "tf_ok": "0",
            "map_x": "",
            "map_y": "",
            "map_z": "",
            "map_roll": "",
            "map_pitch": "",
            "map_yaw": "",
        })
        if self.doback_enabled:
            trajectory_row.update(self.doback_buffer.associate(stamp.to_sec(), self.doback_association_max_age_sec))
        else:
            trajectory_row.update(empty_doback_association("disabled"))
        association_ok, association_reason, association_age_sec, measurement_ros_sec = association_status(
            stamp.to_sec(), row.get("estimated_measurement_ros_time"), self.association_max_age_sec
        )
        trajectory_row["association_age_sec"] = value_or_empty(association_age_sec)
        trajectory_row["tf_lookup_ros_time"] = value_or_empty(measurement_ros_sec)
        if not association_ok:
            trajectory_row["association_rejection_reason"] = association_reason
            rospy.logwarn_throttle(5.0, "GPS sample %s not associated to TF: %s", row.get("sample_seq"), association_reason)
            with self.lock:
                self.trajectory_writer.writerow(csv_row(trajectory_row))
                self.trajectory_csv.flush()
            return
        try:
            lookup_stamp = rospy.Time.from_sec(measurement_ros_sec)
            trans, rot, roll, pitch, yaw = self.lookup_pose(lookup_stamp)
            trajectory_row.update({
                "tf_ok": "1",
                "map_x": trans[0],
                "map_y": trans[1],
                "map_z": trans[2],
                "map_roll": roll,
                "map_pitch": pitch,
                "map_yaw": yaw,
            })
            self.append_path_pose(lookup_stamp, trans, rot)
        except Exception as exc:
            trajectory_row["association_rejection_reason"] = "tf_lookup_failed"
            rospy.logwarn_throttle(10.0, "GPS metadata waiting for TF %s -> %s: %s", self.target_frame, self.robot_frame, exc)
        with self.lock:
            self.trajectory_writer.writerow(csv_row(trajectory_row))
            self.trajectory_csv.flush()

    def append_path_pose(self, stamp, trans, rot):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.target_frame
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        pose.pose.orientation.x = rot[0]
        pose.pose.orientation.y = rot[1]
        pose.pose.orientation.z = rot[2]
        pose.pose.orientation.w = rot[3]
        self.path_poses.append(pose)
        if self.path_max_poses > 0 and len(self.path_poses) > self.path_max_poses:
            self.path_poses = self.path_poses[-self.path_max_poses:]
        path = RosPath()
        path.header.stamp = stamp
        path.header.frame_id = self.target_frame
        path.poses = list(self.path_poses)
        self.path_pub.publish(path)

    def save_service(self, _request):
        self.write_manifest()
        return EmptyResponse()

    def build_manifest(self):
        return {
            "schema": "agv_mapping_gps_doback_metadata_v2",
            "time_utc": utc_now(),
            "output_pcd": self.output_pcd,
            "metadata_dir": self.session_dir,
            "target_frame": self.target_frame,
            "robot_frame": self.robot_frame,
            "gps_frame": self.gps_frame,
            "gps_tcp_bind": self.tcp_bind,
            "gps_tcp_port": self.tcp_port,
            "gps_allowed_hosts": sorted(self.allowed_hosts),
            "gps_quality_gates": {
                "min_sats": self.min_sats,
                "max_hdop": self.max_hdop,
                "max_age_ms": self.max_age_ms,
                "require_fix": self.require_fix,
                "require_sats": self.require_sats,
                "require_hdop": self.require_hdop,
                "require_age": self.require_age,
                "association_max_age_sec": self.association_max_age_sec,
                "tf_wait_timeout_sec": self.tf_wait_timeout_sec,
                "max_line_bytes": self.max_line_bytes,
            },
            "datum_policy": {
                "datum_mode": self.datum_mode,
                "datum_latitude": self.datum_latitude,
                "datum_longitude": self.datum_longitude,
                "datum_altitude": self.datum_altitude,
            },
            "association_policy": {
                "tf_lookup_time": "estimated_measurement_ros_time",
                "max_age_sec": self.association_max_age_sec,
                "tf_wait_timeout_sec": self.tf_wait_timeout_sec,
                "rejection_field": "association_rejection_reason",
            },
            "doback": {
                "enabled": self.doback_enabled,
                "configured_port": self.doback_port,
                "active_port": self.doback_active_port,
                "baud": self.doback_baud,
                "association_clock": "estimated_doback_measurement_ros_time_to_gps_recv_ros_time",
                "batch_time_reconstruction": "anchor_last_row_at_batch_receive_and_backdate_with_usciclo1_to_usciclo5",
                "association_max_age_sec": self.doback_association_max_age_sec,
                "faster_source_policy": "mean_new_samples_since_previous_accepted_gps_fix",
                "slower_source_policy": "reuse_latest_within_max_age",
                "value_fields": list(DOBACK_VALUE_FIELDS),
                "max_line_bytes": self.doback_max_line_bytes,
                "reconnect_sec": self.doback_reconnect_sec,
                "probe_timeout_sec": self.doback_probe_timeout_sec,
            },
            "counts": {
                "samples": self.sample_seq,
                "accepted": self.accepted_count,
                "rejected": self.rejected_count,
                "parse_errors": self.parse_error_count,
                "doback_lines": self.doback_line_count,
                "doback_samples": self.doback_seq,
                "doback_ignored_lines": self.doback_ignored_line_count,
                "doback_connections": self.doback_reconnect_count,
            },
            "files": {
                "gps_csv": self.gps_csv_path,
                "gps_raw_jsonl": self.gps_raw_path,
                "trajectory_gps_map_csv": self.trajectory_path,
                "doback_csv": self.doback_csv_path if self.doback_enabled else "",
                "manifest_json": self.manifest_path,
            },
        }

    def _write_manifest_unlocked(self):
        manifest = self.build_manifest()
        with open(self.manifest_path, "w") as handle:
            json.dump(manifest, handle, indent=2, sort_keys=True)
        rospy.loginfo("GPS metadata manifest saved: %s", self.manifest_path)

    def write_manifest(self):
        with self.lock:
            self._write_manifest_unlocked()

    def close_socket(self, sock):
        try:
            sock.close()
        except Exception:
            pass

    def close(self):
        self.shutdown_event.set()
        if self.server_socket is not None:
            self.close_socket(self.server_socket)
        if self.doback_serial is not None:
            self.close_socket(self.doback_serial)
        with self.lock:
            self.doback_writer = None
            sockets = list(self.client_sockets)
            threads = list(self.client_threads)
        for sock in sockets:
            self.close_socket(sock)
        current = threading.current_thread()
        if self.server_thread is not current:
            self.server_thread.join(1.0)
        if self.doback_serial_thread is not None and self.doback_serial_thread is not current:
            self.doback_serial_thread.join(max(1.0, self.doback_reconnect_sec + 0.5))
        for thread in threads:
            if thread is not current:
                thread.join(1.0)
        with self.lock:
            self._write_manifest_unlocked()
            self.gps_csv.close()
            self.trajectory_csv.close()
            self.gps_raw.close()
            if self.doback_csv is not None:
                self.doback_csv.close()


def selected_fix_stamp_sec(row, recv_ros_sec):
    estimated = safe_float(row.get("estimated_measurement_ros_time"))
    return estimated if estimated is not None else recv_ros_sec


def value_or_empty(value):
    return "" if value is None else value


def main():
    load_ros_modules()
    rospy.init_node("mapping_gps_metadata_logger")
    node = GpsMetadataLogger()
    rospy.on_shutdown(node.close)
    rospy.spin()


if __name__ == "__main__":
    main()
