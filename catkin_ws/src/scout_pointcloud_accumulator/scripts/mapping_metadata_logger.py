#!/usr/bin/env python
from __future__ import print_function

import csv
import glob
import json
import math
import os
import socket
import threading
import time
from datetime import datetime

import rospy
import tf
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as RosPath
from std_srvs.srv import Empty, EmptyResponse
from visualization_msgs.msg import Marker

try:
    import serial
except ImportError:
    serial = None


GPS_FIELDS = [
    "ros_time",
    "recv_time_utc",
    "remote_host",
    "latitud",
    "longitud",
    "altitud_m",
    "satelites",
    "hdop",
    "fix_valido",
    "esperando_fix",
    "mensaje_gps",
    "raw_hex",
    # Backward-compatible English aliases for scripts already reading the old CSV.
    "latitude",
    "longitude",
    "altitude",
    "sats",
    "fix_ok",
    "waiting_for_fix",
]

DOBACK_COLUMNS = [
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

DOBACK_RAW_FIELDS = [
    "ros_time",
    "recv_time_utc",
    "raw_line",
    "parse_ok",
    "error",
]

DOBACK_STABILITY_FIELDS = ["ros_time", "recv_time_utc"] + DOBACK_COLUMNS

MAP_FIELDS = [
    "ros_time",
    "recv_time_utc",
    "map_x",
    "map_y",
    "map_z",
    "map_roll",
    "map_pitch",
    "map_yaw",
    "latitud",
    "longitud",
    "altitud_m",
    "satelites",
    "hdop",
    "fix_valido",
    "esperando_fix",
    "mensaje_gps",
    # Backward-compatible English aliases.
    "latitude",
    "longitude",
    "altitude",
    "sats",
    "fix_ok",
    "waiting_for_fix",
    "gps_text",
    "tf_ok",
    "doback_ok",
    "doback_age_sec",
    "doback_ax",
    "doback_ay",
    "doback_az",
    "doback_gx",
    "doback_gy",
    "doback_gz",
    "doback_roll",
    "doback_pitch",
    "doback_yaw",
    "doback_si",
    "doback_accmag",
]

AGV_TRAJECTORY_FIELDS = [
    "ros_time",
    "recv_time_utc",
    "map_x",
    "map_y",
    "map_z",
    "map_roll",
    "map_pitch",
    "map_yaw",
    "distance_from_previous_m",
    "distance_total_m",
]



def utc_now():
    now = datetime.utcnow()
    return now.strftime("%Y-%m-%dT%H:%M:%S") + ".%03dZ" % int(now.microsecond / 1000)


def append_suffix(path, suffix, ext):
    root, _ = os.path.splitext(path)
    return root + suffix + ext


def timestamp_slug():
    return datetime.utcnow().strftime("%Y%m%d_%H%M%S")


def safe_float(value):
    if value in (None, "", "?"):
        return ""
    try:
        return float(value)
    except (TypeError, ValueError):
        return ""


def safe_int(value):
    if value in (None, "", "?"):
        return ""
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return ""


def truthy(value):
    if value in (None, "", "?"):
        return ""
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "ok", "fix"):
        return "1"
    if text in ("0", "false", "no", "none"):
        return "0"
    return text


def parse_nmea_latlon(raw_value, hemi):
    if raw_value in (None, "", "?") or hemi in (None, "", "?"):
        return ""
    try:
        value = float(raw_value)
        degrees = int(value / 100)
        minutes = value - degrees * 100
        decimal = degrees + minutes / 60.0
        if hemi.upper() in ("S", "W"):
            decimal *= -1.0
        return decimal
    except (TypeError, ValueError):
        return ""


def parse_nmea(text):
    if not text.startswith("$"):
        return {}
    fields = text.strip().split(",")
    sentence = fields[0][-3:]
    out = {}
    if sentence == "GGA" and len(fields) >= 10:
        out["latitude"] = parse_nmea_latlon(fields[2], fields[3])
        out["longitude"] = parse_nmea_latlon(fields[4], fields[5])
        out["fix_ok"] = "1" if safe_int(fields[6]) else "0"
        out["sats"] = safe_int(fields[7])
        out["hdop"] = safe_float(fields[8])
        out["altitude"] = safe_float(fields[9])
    elif sentence == "RMC" and len(fields) >= 10:
        out["latitude"] = parse_nmea_latlon(fields[3], fields[4])
        out["longitude"] = parse_nmea_latlon(fields[5], fields[6])
        out["fix_ok"] = "1" if fields[2] == "A" else "0"
    return out


def extract_gps(payload):
    text = payload.get("text", "") or ""
    parsed = payload.get("parsed", {}) or {}
    if not isinstance(parsed, dict):
        parsed = {}

    data = {}
    data.update(parse_nmea(text))

    for key in ("lat", "latitude"):
        if key in payload:
            data["latitude"] = safe_float(payload.get(key))
        if key in parsed:
            data["latitude"] = safe_float(parsed.get(key))
    for key in ("lon", "lng", "longitude"):
        if key in payload:
            data["longitude"] = safe_float(payload.get(key))
        if key in parsed:
            data["longitude"] = safe_float(parsed.get(key))
    for key in ("alt", "altitude"):
        if key in payload:
            data["altitude"] = safe_float(payload.get(key))
        if key in parsed:
            data["altitude"] = safe_float(parsed.get(key))
    for key in ("sats", "satellites"):
        if key in payload:
            data["sats"] = safe_int(payload.get(key))
        if key in parsed:
            data["sats"] = safe_int(parsed.get(key))
    if "hdop" in payload:
        data["hdop"] = safe_float(payload.get("hdop"))
    if "hdop" in parsed:
        data["hdop"] = safe_float(parsed.get("hdop"))
    if "waiting_for_fix" in parsed:
        data["waiting_for_fix"] = truthy(parsed.get("waiting_for_fix"))
        data["fix_ok"] = "0" if data["waiting_for_fix"] == "1" else data.get("fix_ok", "")
    if "sentences_fix" in parsed and "fix_ok" not in data:
        data["fix_ok"] = "1" if safe_int(parsed.get("sentences_fix")) else "0"

    for key in ("latitude", "longitude", "altitude", "sats", "hdop", "fix_ok", "waiting_for_fix"):
        data.setdefault(key, "")
    return data


def parse_doback_line(line):
    parts = [part.strip() for part in line.strip().split(";")]
    if len(parts) == 1 and "," in line:
        parts = [part.strip() for part in line.strip().split(",")]
    if not parts or parts[0].lower() == "ax":
        return None, "header"
    if len(parts) < len(DOBACK_COLUMNS):
        return None, "expected_%d_fields_got_%d" % (len(DOBACK_COLUMNS), len(parts))
    data = {}
    for index, name in enumerate(DOBACK_COLUMNS):
        data[name] = parts[index]
    return data, ""


class MetadataLogger(object):
    def __init__(self):
        self.output_pcd = rospy.get_param("~output_pcd", "/tmp/accumulated_cloud.pcd")
        self.metadata_dir_param = rospy.get_param("~metadata_dir", "")
        self.target_frame = rospy.get_param("~target_frame", "map").lstrip("/")
        self.robot_frame = rospy.get_param("~robot_frame", "base_link").lstrip("/")
        self.gps_tcp_enable = rospy.get_param("~gps_tcp_enable", True)
        self.gps_tcp_bind = rospy.get_param("~gps_tcp_bind", "0.0.0.0")
        self.gps_tcp_port = int(rospy.get_param("~gps_tcp_port", 29500))
        self.gps_allowed_hosts = self.parse_hosts(rospy.get_param("~gps_allowed_hosts", ""))
        self.gps_required = rospy.get_param("~gps_required", False)
        self.doback_enable = rospy.get_param("~doback_enable", False)
        self.doback_required = rospy.get_param("~doback_required", False)
        self.doback_port = rospy.get_param("~doback_port", "auto")
        self.doback_baud = int(rospy.get_param("~doback_baud", 115200))
        self.doback_test_file = rospy.get_param("~doback_test_file", "")
        self.join_slop_sec = float(rospy.get_param("~join_slop_sec", 2.0))
        self.trajectory_enable = rospy.get_param("~trajectory_enable", True)
        self.trajectory_publish_rate = float(rospy.get_param("~trajectory_publish_rate", 2.0))
        self.trajectory_min_distance = float(rospy.get_param("~trajectory_min_distance", 0.05))
        self.trajectory_max_points = int(rospy.get_param("~trajectory_max_points", 10000))
        self.trajectory_path_topic = rospy.get_param("~trajectory_path_topic", "/agv_trajectory_path")
        self.trajectory_marker_topic = rospy.get_param("~trajectory_marker_topic", "/agv_trajectory_marker")

        self.lock = threading.Lock()
        self.shutdown_event = threading.Event()
        self.closed = False
        self.server_socket = None
        self.listener = tf.TransformListener()
        self.gps_count = 0
        self.doback_count = 0
        self.latest_gps = None
        self.latest_doback = None
        self.active_doback_port = ""
        self.trajectory_points = []
        self.trajectory_poses = []
        self.trajectory_last_point = None
        self.trajectory_distance_total = 0.0
        self.trajectory_count = 0

        self.session_dir = self.resolve_metadata_dir()
        self.gps_csv_path = os.path.join(self.session_dir, "gps.csv")
        self.gps_raw_path = os.path.join(self.session_dir, "gps_raw.jsonl")
        self.map_track_path = os.path.join(self.session_dir, "trayectoria_gps_doback.csv")
        self.agv_trajectory_path = os.path.join(self.session_dir, "trayectoria_agv_mapa.csv")
        self.manifest_path = os.path.join(self.session_dir, "manifest.json")
        self.doback_raw_path = os.path.join(self.session_dir, "doback_raw.csv")
        self.doback_stability_path = os.path.join(self.session_dir, "doback.csv")
        self.legacy_paths = self.build_legacy_paths()

        self.ensure_parent(self.gps_csv_path)
        self.gps_csv = open(self.gps_csv_path, "a")
        self.gps_writer = csv.DictWriter(self.gps_csv, fieldnames=GPS_FIELDS)
        if os.path.getsize(self.gps_csv_path) == 0:
            self.gps_writer.writeheader()

        self.map_csv = open(self.map_track_path, "a")
        self.map_writer = csv.DictWriter(self.map_csv, fieldnames=MAP_FIELDS)
        if os.path.getsize(self.map_track_path) == 0:
            self.map_writer.writeheader()

        self.agv_trajectory_csv = open(self.agv_trajectory_path, "a")
        self.agv_trajectory_writer = csv.DictWriter(self.agv_trajectory_csv, fieldnames=AGV_TRAJECTORY_FIELDS)
        if os.path.getsize(self.agv_trajectory_path) == 0:
            self.agv_trajectory_writer.writeheader()

        self.gps_raw = open(self.gps_raw_path, "a")

        self.doback_raw = open(self.doback_raw_path, "a")
        self.doback_raw_writer = csv.DictWriter(self.doback_raw, fieldnames=DOBACK_RAW_FIELDS)
        if os.path.getsize(self.doback_raw_path) == 0:
            self.doback_raw_writer.writeheader()

        self.doback_csv = open(self.doback_stability_path, "a")
        self.doback_writer = csv.DictWriter(self.doback_csv, fieldnames=DOBACK_STABILITY_FIELDS)
        if os.path.getsize(self.doback_stability_path) == 0:
            self.doback_writer.writeheader()

        self.path_pub = rospy.Publisher(self.trajectory_path_topic, RosPath, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher(self.trajectory_marker_topic, Marker, queue_size=1, latch=True)
        self.trajectory_timer = None
        if self.trajectory_enable:
            period = 1.0 / self.trajectory_publish_rate if self.trajectory_publish_rate > 0.0 else 1.0
            self.trajectory_timer = rospy.Timer(rospy.Duration(period), self.trajectory_timer_cb)
            rospy.loginfo(
                "AGV trajectory enabled: path=%s marker=%s frame=%s robot=%s rate=%.2fHz min_distance=%.3fm",
                self.trajectory_path_topic,
                self.trajectory_marker_topic,
                self.target_frame,
                self.robot_frame,
                self.trajectory_publish_rate,
                self.trajectory_min_distance,
            )

        self.save_srv = rospy.Service("~save_metadata", Empty, self.save_service)

        if self.doback_enable:
            self.doback_thread = threading.Thread(target=self.doback_serial_loop)
            self.doback_thread.daemon = True
            self.doback_thread.start()
            rospy.loginfo("DOBACK serial logger enabled on %s @ %d baud", self.doback_port, self.doback_baud)
        if self.gps_tcp_enable:
            self.server_thread = threading.Thread(target=self.tcp_server)
            self.server_thread.daemon = True
            self.server_thread.start()
            rospy.loginfo("GPS TCP metadata server listening on %s:%d", self.gps_tcp_bind, self.gps_tcp_port)
        else:
            rospy.logwarn("GPS TCP metadata server disabled.")

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

    def resolve_metadata_dir(self):
        if self.metadata_dir_param:
            path = self.metadata_dir_param
        else:
            workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
            session_name = "sesion_" + timestamp_slug()
            path = os.path.join(workspace_root, "datos", session_name)
        if not os.path.isdir(path):
            os.makedirs(path)
        return path

    def build_legacy_paths(self):
        return {
            "gps_csv": append_suffix(self.output_pcd, "_gps", ".csv"),
            "gps_raw_jsonl": append_suffix(self.output_pcd, "_gps_raw", ".jsonl"),
            "map_track_csv": append_suffix(self.output_pcd, "_map_track", ".csv"),
            "agv_trajectory_csv": append_suffix(self.output_pcd, "_agv_trajectory", ".csv"),
            "doback_raw_csv": append_suffix(self.output_pcd, "_doback_raw", ".csv"),
            "doback_stability_csv": append_suffix(self.output_pcd, "_doback_stability", ".csv"),
            "manifest_json": append_suffix(self.output_pcd, "_session_manifest", ".json"),
        }

    def tcp_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket = server
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.gps_tcp_bind, self.gps_tcp_port))
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
            thread.start()
        try:
            server.close()
        except Exception:
            pass

    def host_allowed(self, host):
        if not self.gps_allowed_hosts:
            return True
        return host in self.gps_allowed_hosts

    def resolve_doback_port(self):
        if self.doback_port and self.doback_port.lower() != "auto":
            return self.doback_port
        candidates = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
        if not candidates:
            return ""
        return candidates[0]

    def doback_serial_loop(self):
        if self.doback_test_file:
            rospy.loginfo("DOBACK test file enabled: %s", self.doback_test_file)
            while not rospy.is_shutdown() and not self.shutdown_event.is_set():
                try:
                    with open(self.doback_test_file) as handle:
                        for line in handle:
                            if rospy.is_shutdown() or self.shutdown_event.is_set():
                                break
                            line = line.strip()
                            if line:
                                self.record_doback_line(line)
                            time.sleep(0.1)
                    time.sleep(1.0)
                except Exception as exc:
                    rospy.logwarn_throttle(10.0, "DOBACK test file error: %s", exc)
                    time.sleep(2.0)
            return

        if serial is None:
            rospy.logerr("DOBACK enabled but python-serial is not available. Install python-serial.")
            return

        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            try:
                port_name = self.resolve_doback_port()
                if not port_name:
                    rospy.logwarn_throttle(10.0, "DOBACK serial auto-detect found no /dev/ttyACM* or /dev/ttyUSB* devices.")
                    time.sleep(2.0)
                    continue
                port = serial.Serial(port_name, self.doback_baud, timeout=1.0)
                self.active_doback_port = port_name
                rospy.loginfo("DOBACK serial opened: %s", port_name)
                while not rospy.is_shutdown() and not self.shutdown_event.is_set():
                    raw = port.readline()
                    if not raw:
                        continue
                    if isinstance(raw, bytes):
                        line = raw.decode("utf-8", errors="replace").strip()
                    else:
                        line = raw.strip()
                    if line:
                        self.record_doback_line(line)
                port.close()
            except Exception as exc:
                rospy.logwarn_throttle(10.0, "DOBACK serial error on %s: %s", self.doback_port, exc)
                time.sleep(2.0)

    def record_doback_line(self, line):
        ros_time = rospy.Time.now().to_sec()
        recv_time_utc = utc_now()
        data, error = parse_doback_line(line)
        raw_row = {
            "ros_time": ros_time,
            "recv_time_utc": recv_time_utc,
            "raw_line": line,
            "parse_ok": "1" if data is not None else "0",
            "error": error,
        }
        with self.lock:
            self.doback_raw_writer.writerow(raw_row)
            self.doback_raw.flush()
            if data is not None:
                row = {"ros_time": ros_time, "recv_time_utc": recv_time_utc}
                row.update(data)
                self.doback_writer.writerow(row)
                self.doback_csv.flush()
                self.latest_doback = row
                self.doback_count += 1
        if data is not None:
            rospy.loginfo_throttle(
                5.0,
                "DOBACK samples received: %d latest roll=%s pitch=%s yaw=%s si=%s",
                self.doback_count,
                data.get("roll", ""),
                data.get("pitch", ""),
                data.get("yaw", ""),
                data.get("si", ""),
            )

    def handle_client(self, conn, addr):
        host = addr[0]
        if not self.host_allowed(host):
            rospy.logwarn("Rejected GPS TCP client from %s", host)
            conn.close()
            return
        rospy.loginfo("Accepted GPS TCP client from %s", host)
        buf = b""
        try:
            while not rospy.is_shutdown() and not self.shutdown_event.is_set():
                chunk = conn.recv(4096)
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode("utf-8", errors="replace").strip()
                    if text:
                        self.record_line(text, host)
        except Exception as exc:
            rospy.logwarn("GPS TCP client %s error: %s", host, exc)
        finally:
            conn.close()
            rospy.loginfo("GPS TCP client %s disconnected", host)

    def record_line(self, line, remote_host):
        ros_time = rospy.Time.now().to_sec()
        try:
            payload = json.loads(line)
            if not isinstance(payload, dict):
                payload = {"text": line}
        except ValueError:
            payload = {"text": line}
        payload.setdefault("text", "")
        payload.setdefault("raw_hex", "")
        payload["remote_host"] = remote_host
        payload["recv_time_utc"] = utc_now()
        payload["ros_time"] = ros_time
        gps = extract_gps(payload)

        gps_row = {
            "ros_time": ros_time,
            "recv_time_utc": payload["recv_time_utc"],
            "remote_host": remote_host,
            "latitud": gps.get("latitude", ""),
            "longitud": gps.get("longitude", ""),
            "altitud_m": gps.get("altitude", ""),
            "satelites": gps.get("sats", ""),
            "hdop": gps.get("hdop", ""),
            "fix_valido": gps.get("fix_ok", ""),
            "esperando_fix": gps.get("waiting_for_fix", ""),
            "mensaje_gps": payload.get("text", ""),
            "raw_hex": payload.get("raw_hex", ""),
            "latitude": gps.get("latitude", ""),
            "longitude": gps.get("longitude", ""),
            "altitude": gps.get("altitude", ""),
            "sats": gps.get("sats", ""),
            "fix_ok": gps.get("fix_ok", ""),
            "waiting_for_fix": gps.get("waiting_for_fix", ""),
        }

        with self.lock:
            latest_doback = dict(self.latest_doback) if self.latest_doback is not None else None

        map_row = self.build_map_row(ros_time, payload["recv_time_utc"], gps, payload.get("text", ""), latest_doback)
        with self.lock:
            self.gps_raw.write(json.dumps(payload, sort_keys=True) + "\n")
            self.gps_raw.flush()
            self.gps_writer.writerow(gps_row)
            self.gps_csv.flush()
            self.map_writer.writerow(map_row)
            self.map_csv.flush()
            self.latest_gps = gps_row
            self.gps_count += 1
        rospy.loginfo_throttle(5.0, "GPS TCP samples received: %d latest='%s'", self.gps_count, payload.get("text", ""))

    def build_map_row(self, ros_time, recv_time_utc, gps, gps_text, doback):
        row = {
            "ros_time": ros_time,
            "recv_time_utc": recv_time_utc,
            "map_x": "",
            "map_y": "",
            "map_z": "",
            "map_roll": "",
            "map_pitch": "",
            "map_yaw": "",
            "latitud": gps.get("latitude", ""),
            "longitud": gps.get("longitude", ""),
            "altitud_m": gps.get("altitude", ""),
            "satelites": gps.get("sats", ""),
            "hdop": gps.get("hdop", ""),
            "fix_valido": gps.get("fix_ok", ""),
            "esperando_fix": gps.get("waiting_for_fix", ""),
            "mensaje_gps": gps_text,
            "latitude": gps.get("latitude", ""),
            "longitude": gps.get("longitude", ""),
            "altitude": gps.get("altitude", ""),
            "sats": gps.get("sats", ""),
            "fix_ok": gps.get("fix_ok", ""),
            "waiting_for_fix": gps.get("waiting_for_fix", ""),
            "gps_text": gps_text,
            "tf_ok": "0",
            "doback_ok": "0",
            "doback_age_sec": "",
            "doback_ax": "",
            "doback_ay": "",
            "doback_az": "",
            "doback_gx": "",
            "doback_gy": "",
            "doback_gz": "",
            "doback_roll": "",
            "doback_pitch": "",
            "doback_yaw": "",
            "doback_si": "",
            "doback_accmag": "",
        }
        if doback is not None:
            try:
                age = abs(float(ros_time) - float(doback.get("ros_time", ros_time)))
            except (TypeError, ValueError):
                age = ""
            if age == "" or age <= self.join_slop_sec:
                row.update({
                    "doback_ok": "1",
                    "doback_age_sec": age,
                    "doback_ax": doback.get("ax", ""),
                    "doback_ay": doback.get("ay", ""),
                    "doback_az": doback.get("az", ""),
                    "doback_gx": doback.get("gx", ""),
                    "doback_gy": doback.get("gy", ""),
                    "doback_gz": doback.get("gz", ""),
                    "doback_roll": doback.get("roll", ""),
                    "doback_pitch": doback.get("pitch", ""),
                    "doback_yaw": doback.get("yaw", ""),
                    "doback_si": doback.get("si", ""),
                    "doback_accmag": doback.get("accmag", ""),
                })
        try:
            trans, _rot, roll, pitch, yaw = self.lookup_robot_pose()
            row.update({
                "map_x": trans[0],
                "map_y": trans[1],
                "map_z": trans[2],
                "map_roll": roll,
                "map_pitch": pitch,
                "map_yaw": yaw,
                "tf_ok": "1",
            })
        except Exception:
            pass
        return row


    def lookup_robot_pose(self):
        trans, rot = self.listener.lookupTransform(self.target_frame, self.robot_frame, rospy.Time(0))
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
        return trans, rot, roll, pitch, yaw

    def point_distance(self, a, b):
        dx = float(a[0]) - float(b[0])
        dy = float(a[1]) - float(b[1])
        dz = float(a[2]) - float(b[2])
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def trajectory_timer_cb(self, _event):
        try:
            trans, rot, roll, pitch, yaw = self.lookup_robot_pose()
        except Exception as exc:
            rospy.logwarn_throttle(10.0, "AGV trajectory waiting for TF %s -> %s: %s", self.target_frame, self.robot_frame, exc)
            return

        ros_time = rospy.Time.now()
        point = (float(trans[0]), float(trans[1]), float(trans[2]))
        with self.lock:
            if self.trajectory_last_point is not None:
                delta = self.point_distance(point, self.trajectory_last_point)
                if delta < self.trajectory_min_distance:
                    self.publish_trajectory_locked(ros_time)
                    return
            else:
                delta = 0.0

            self.trajectory_distance_total += delta
            self.trajectory_last_point = point
            self.trajectory_points.append(point)

            pose = PoseStamped()
            pose.header.stamp = ros_time
            pose.header.frame_id = self.target_frame
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            pose.pose.orientation.x = rot[0]
            pose.pose.orientation.y = rot[1]
            pose.pose.orientation.z = rot[2]
            pose.pose.orientation.w = rot[3]
            self.trajectory_poses.append(pose)

            if self.trajectory_max_points > 0 and len(self.trajectory_points) > self.trajectory_max_points:
                self.trajectory_points = self.trajectory_points[-self.trajectory_max_points:]
                self.trajectory_poses = self.trajectory_poses[-self.trajectory_max_points:]

            row = {
                "ros_time": ros_time.to_sec(),
                "recv_time_utc": utc_now(),
                "map_x": point[0],
                "map_y": point[1],
                "map_z": point[2],
                "map_roll": roll,
                "map_pitch": pitch,
                "map_yaw": yaw,
                "distance_from_previous_m": delta,
                "distance_total_m": self.trajectory_distance_total,
            }
            self.agv_trajectory_writer.writerow(row)
            self.agv_trajectory_csv.flush()
            self.trajectory_count += 1
            self.publish_trajectory_locked(ros_time)

        rospy.loginfo_throttle(5.0, "AGV trajectory points: %d total_distance=%.2fm", self.trajectory_count, self.trajectory_distance_total)

    def publish_trajectory_locked(self, stamp):
        path_msg = RosPath()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = self.target_frame
        path_msg.poses = list(self.trajectory_poses)
        self.path_pub.publish(path_msg)

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.target_frame
        marker.ns = "agv_trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.06
        marker.color.r = 1.0
        marker.color.g = 0.15
        marker.color.b = 0.05
        marker.color.a = 1.0
        marker.points = []
        for item in self.trajectory_points:
            point = Point()
            point.x = item[0]
            point.y = item[1]
            point.z = item[2] + 0.08
            marker.points.append(point)
        self.marker_pub.publish(marker)

    def save_service(self, _request):
        self.write_manifest()
        return EmptyResponse()

    def write_manifest(self):
        manifest = {
            "time_utc": utc_now(),
            "output_pcd": self.output_pcd,
            "metadata_dir": self.session_dir,
            "target_frame": self.target_frame,
            "robot_frame": self.robot_frame,
            "gps_tcp_bind": self.gps_tcp_bind,
            "gps_tcp_port": self.gps_tcp_port,
            "gps_allowed_hosts": sorted(self.gps_allowed_hosts),
            "gps_count": self.gps_count,
            "doback_count": self.doback_count,
            "files": {
                "gps_csv": self.gps_csv_path,
                "gps_raw_jsonl": self.gps_raw_path,
                "map_track_csv": self.map_track_path,
                "agv_trajectory_csv": self.agv_trajectory_path,
                "doback_raw_csv": self.doback_raw_path,
                "doback_stability_csv": self.doback_stability_path,
                "manifest_json": self.manifest_path,
            },
            "legacy_file_names": self.legacy_paths,
            "doback_enabled": self.doback_enable,
            "doback_port": self.doback_port,
            "active_doback_port": self.active_doback_port,
            "doback_baud": self.doback_baud,
            "doback_test_file": self.doback_test_file,
            "join_slop_sec": self.join_slop_sec,
            "trajectory_enabled": self.trajectory_enable,
            "trajectory_count": self.trajectory_count,
            "trajectory_distance_total_m": self.trajectory_distance_total,
            "trajectory_path_topic": self.trajectory_path_topic,
            "trajectory_marker_topic": self.trajectory_marker_topic,
            "trajectory_publish_rate": self.trajectory_publish_rate,
            "trajectory_min_distance": self.trajectory_min_distance,
            "trajectory_max_points": self.trajectory_max_points,
        }
        with open(self.manifest_path, "w") as handle:
            json.dump(manifest, handle, indent=2, sort_keys=True)
        rospy.loginfo("Metadata manifest saved: %s", self.manifest_path)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.shutdown_event.set()
        if self.server_socket is not None:
            try:
                self.server_socket.close()
            except Exception:
                pass
        if self.trajectory_timer is not None:
            try:
                self.trajectory_timer.shutdown()
            except Exception:
                pass
        self.write_manifest()
        self.gps_csv.close()
        self.map_csv.close()
        self.agv_trajectory_csv.close()
        self.gps_raw.close()
        self.doback_raw.close()
        self.doback_csv.close()


def main():
    rospy.init_node("mapping_metadata_logger")
    node = MetadataLogger()
    rospy.on_shutdown(node.close)
    rospy.spin()


if __name__ == "__main__":
    main()
