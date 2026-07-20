#!/usr/bin/env python3
"""Offline georeference export for AGV LiDAR maps.

Reads GPS/map-pose metadata, converts WGS84 fixes to a local ENU frame, fits a
2D map-to-ENU similarity transform, and exports trajectory and optional point
cloud products. PCD support is intentionally limited to ASCII PCD; binary PCDs
are reported as unsupported rather than silently mishandled.
"""

import argparse
import csv
import json
import math
import os
import sys
from datetime import datetime, timezone
from pathlib import Path


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


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


def validate_lat_lon_alt(lat, lon, alt):
    if lat is None or lon is None:
        raise ValueError("Datum latitude and longitude must be finite numbers")
    if lat < -90.0 or lat > 90.0:
        raise ValueError("Datum latitude out of range")
    if lon < -180.0 or lon > 180.0:
        raise ValueError("Datum longitude out of range")
    if alt is None:
        raise ValueError("Datum altitude must be a finite number")


def datum_from_manifest(path):
    if not path:
        return None
    with open(path) as handle:
        manifest = json.load(handle)
    policy = manifest.get("datum_policy", {})
    lat = safe_float(policy.get("datum_latitude"))
    lon = safe_float(policy.get("datum_longitude"))
    alt = safe_float(policy.get("datum_altitude"))
    if lat is None and lon is None and alt is None:
        return None
    validate_lat_lon_alt(lat, lon, alt)
    return {
        "mode": "metadata_manifest",
        "manifest": str(path),
        "latitude": lat,
        "longitude": lon,
        "altitude": alt,
    }


def require_float(value, field_name):
    parsed = safe_float(value)
    if parsed is None:
        raise ValueError("{} must be a finite number".format(field_name))
    return parsed


def truthy(value):
    text = str(value).strip().lower()
    return text in ("1", "true", "yes", "ok", "fix", "valid")


def lla_to_ecef(lat_deg, lon_deg, alt_m):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + alt_m) * cos_lat * cos_lon
    y = (n + alt_m) * cos_lat * sin_lon
    z = (n * (1.0 - WGS84_E2) + alt_m) * sin_lat
    return x, y, z


def ecef_to_enu(x, y, z, datum):
    lat0 = math.radians(datum["latitude"])
    lon0 = math.radians(datum["longitude"])
    x0, y0, z0 = lla_to_ecef(datum["latitude"], datum["longitude"], datum["altitude"])
    dx = x - x0
    dy = y - y0
    dz = z - z0
    sin_lat = math.sin(lat0)
    cos_lat = math.cos(lat0)
    sin_lon = math.sin(lon0)
    cos_lon = math.cos(lon0)
    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return east, north, up


def lla_to_enu(lat, lon, alt, datum):
    return ecef_to_enu(*lla_to_ecef(lat, lon, alt), datum=datum)


def load_rows(path):
    with open(path, newline="") as handle:
        return list(csv.DictReader(handle))


def accepted_row(row):
    if "accepted" in row and str(row.get("accepted", "")).strip() != "":
        return truthy(row.get("accepted"))
    return row.get("latitude") not in (None, "") and row.get("longitude") not in (None, "")


def choose_datum(rows, args):
    if (args.datum_latitude is None) != (args.datum_longitude is None):
        raise ValueError("--datum-latitude and --datum-longitude must be provided together")
    if args.datum_latitude is not None and args.datum_longitude is not None:
        validate_lat_lon_alt(args.datum_latitude, args.datum_longitude, args.datum_altitude)
        return {
            "mode": "manual_wgs84",
            "latitude": args.datum_latitude,
            "longitude": args.datum_longitude,
            "altitude": args.datum_altitude,
        }
    manifest_path = args.metadata_manifest
    if manifest_path is None:
        candidate = Path(args.trajectory).resolve().parent / "manifest.json"
        if candidate.exists():
            manifest_path = str(candidate)
    manifest_datum = datum_from_manifest(manifest_path)
    if manifest_datum is not None:
        return manifest_datum
    if manifest_path is not None and args.metadata_manifest is not None:
        raise ValueError("Metadata manifest does not contain a complete manual datum policy")
    if not args.allow_first_fix_datum:
        raise ValueError(
            "No explicit datum available. Pass --datum-latitude/--datum-longitude, "
            "--metadata-manifest with manual datum fields, or --allow-first-fix-datum for exploratory export."
        )
    for row in rows:
        if not accepted_row(row):
            continue
        lat = safe_float(row.get("latitude"))
        lon = safe_float(row.get("longitude"))
        if lat is None or lon is None:
            continue
        return {
            "mode": "first_valid_fix",
            "latitude": lat,
            "longitude": lon,
            "altitude": require_float(row.get("altitude"), "altitude"),
        }
    raise ValueError("No valid GPS row available to choose datum")


def build_pairs(rows, datum):
    pairs = []
    trajectory = []
    for row in rows:
        lat = safe_float(row.get("latitude"))
        lon = safe_float(row.get("longitude"))
        alt = require_float(row.get("altitude"), "altitude")
        if lat is None or lon is None:
            continue
        east, north, up = lla_to_enu(lat, lon, alt, datum)
        out = dict(row)
        out.update({"enu_e": east, "enu_n": north, "enu_u": up})
        trajectory.append(out)
        tf_ok = row.get("tf_ok", "1")
        if accepted_row(row) and truthy(tf_ok):
            map_x = require_float(row.get("map_x"), "map_x")
            map_y = require_float(row.get("map_y"), "map_y")
            map_z = require_float(row.get("map_z"), "map_z")
            pairs.append({"map": (map_x, map_y, map_z), "enu": (east, north, up), "row": row})
    return trajectory, pairs


def estimate_similarity_2d(pairs, min_pairs):
    if len(pairs) < min_pairs:
        raise ValueError("Need at least {} paired map/GPS samples, got {}".format(min_pairs, len(pairs)))
    map_pts = [item["map"] for item in pairs]
    enu_pts = [item["enu"] for item in pairs]
    mx = sum(p[0] for p in map_pts) / len(map_pts)
    my = sum(p[1] for p in map_pts) / len(map_pts)
    qx = sum(p[0] for p in enu_pts) / len(enu_pts)
    qy = sum(p[1] for p in enu_pts) / len(enu_pts)
    den = 0.0
    a_num = 0.0
    b_num = 0.0
    for p, q in zip(map_pts, enu_pts):
        px = p[0] - mx
        py = p[1] - my
        ex = q[0] - qx
        ny = q[1] - qy
        den += px * px + py * py
        a_num += ex * px + ny * py
        b_num += ny * px - ex * py
    if den == 0.0:
        raise ValueError("Map pose pairs have zero spatial spread")
    a = a_num / den
    b = b_num / den
    tx = qx - (a * mx - b * my)
    ty = qy - (b * mx + a * my)
    tz_values = [q[2] - p[2] for p, q in zip(map_pts, enu_pts)]
    tz = sum(tz_values) / len(tz_values)
    residuals = []
    for p, q in zip(map_pts, enu_pts):
        pred = transform_point(p, {"a": a, "b": b, "tx": tx, "ty": ty, "tz": tz})
        dx = pred[0] - q[0]
        dy = pred[1] - q[1]
        dz = pred[2] - q[2]
        residuals.append(math.sqrt(dx * dx + dy * dy + dz * dz))
    rms = math.sqrt(sum(value * value for value in residuals) / len(residuals))
    return {
        "type": "2d_similarity_plus_vertical_offset",
        "a": a,
        "b": b,
        "tx": tx,
        "ty": ty,
        "tz": tz,
        "scale": math.sqrt(a * a + b * b),
        "yaw_rad": math.atan2(b, a),
        "pair_count": len(pairs),
        "rms_residual_m": rms,
        "max_residual_m": max(residuals),
        "residuals_m": residuals,
    }


def transform_point(point, transform):
    x, y, z = point
    east = transform["a"] * x - transform["b"] * y + transform["tx"]
    north = transform["b"] * x + transform["a"] * y + transform["ty"]
    up = z + transform["tz"]
    return east, north, up


def write_csv(path, rows, fieldnames):
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def read_points_csv(path):
    rows = load_rows(path)
    points = []
    for row in rows:
        x = safe_float(row.get("x"))
        y = safe_float(row.get("y"))
        z = safe_float(row.get("z"))
        if x is None or y is None or z is None:
            raise ValueError("CSV point is missing numeric x/y/z fields")
        points.append((x, y, z, row))
    return points


def write_points_csv(path, points, transform):
    rows = []
    for x, y, z, row in points:
        east, north, up = transform_point((x, y, z), transform)
        out = dict(row)
        out.update({"enu_e": east, "enu_n": north, "enu_u": up})
        rows.append(out)
    fields = sorted(set().union(*[set(row.keys()) for row in rows])) if rows else ["x", "y", "z", "enu_e", "enu_n", "enu_u"]
    write_csv(path, rows, fields)


def read_ascii_pcd(path):
    header = []
    body_lines = []
    fields = []
    data_index_found = False
    with open(path, "rb") as handle:
        while True:
            raw_line = handle.readline()
            if raw_line == b"":
                break
            try:
                line = raw_line.decode("ascii")
            except UnicodeDecodeError:
                raise ValueError("Invalid PCD header encoding")
            header.append(line)
            parts = line.strip().split()
            if not parts:
                continue
            if parts[0].upper() == "FIELDS":
                fields = parts[1:]
            if parts[0].upper() == "DATA":
                if len(parts) < 2:
                    raise ValueError("Invalid PCD DATA line")
                if parts[1].lower() != "ascii":
                    raise ValueError("Only ASCII PCD is supported by this exporter")
                data_index_found = True
                break
        if not data_index_found or not fields:
            raise ValueError("Invalid PCD header")
        try:
            body_text = handle.read().decode("ascii")
        except UnicodeDecodeError:
            raise ValueError("Invalid ASCII PCD body encoding")
        body_lines = body_text.splitlines()
    points = []
    for line in body_lines:
        if not line.strip():
            continue
        values = line.strip().split()
        row = dict(zip(fields, values))
        x = safe_float(row.get("x"))
        y = safe_float(row.get("y"))
        z = safe_float(row.get("z"))
        if x is None or y is None or z is None:
            raise ValueError("PCD point is missing numeric x/y/z fields")
        points.append((x, y, z, row, values))
    return header, fields, points


def write_ascii_pcd(path, header, fields, points, transform):
    data_line = None
    for index, line in enumerate(header):
        if line.strip().upper().startswith("DATA "):
            data_line = index
            break
    if data_line is None:
        raise ValueError("Invalid PCD header")
    out_header = list(header[:data_line + 1])
    with open(path, "w") as handle:
        for line in out_header:
            handle.write(line)
        for x, y, z, row, values in points:
            transformed = transform_point((x, y, z), transform)
            out_values = list(values)
            for field, value in zip(("x", "y", "z"), transformed):
                if field in fields:
                    out_values[fields.index(field)] = "{:.6f}".format(value)
            handle.write(" ".join(out_values) + "\n")


def build_parser():
    parser = argparse.ArgumentParser(description="Offline georeference export for AGV LiDAR map products.")
    parser.add_argument("--trajectory", required=True, help="trajectory_gps_map.csv from mapping_gps_metadata_logger.py")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--output-prefix", default="map_georef")
    parser.add_argument("--points-csv", help="Optional CSV with x,y,z map-frame points.")
    parser.add_argument("--pcd", help="Optional ASCII PCD with x,y,z map-frame points.")
    parser.add_argument("--metadata-manifest", help="Optional GPS metadata manifest with manual datum policy.")
    parser.add_argument("--datum-latitude", type=float)
    parser.add_argument("--datum-longitude", type=float)
    parser.add_argument("--datum-altitude", type=float, default=0.0)
    parser.add_argument("--allow-first-fix-datum", action="store_true", help="Allow exploratory ENU origin from the first valid GPS row.")
    parser.add_argument("--min-pairs", type=int, default=3)
    parser.add_argument("--max-rms-warning", type=float, default=10.0)
    return parser


def main():
    args = build_parser().parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = load_rows(args.trajectory)
    datum = choose_datum(rows, args)
    trajectory, pairs = build_pairs(rows, datum)
    transform = estimate_similarity_2d(pairs, args.min_pairs)

    trajectory_path = output_dir / (args.output_prefix + "_trajectory_enu.csv")
    trajectory_fields = sorted(set().union(*[set(row.keys()) for row in trajectory])) if trajectory else ["enu_e", "enu_n", "enu_u"]
    write_csv(str(trajectory_path), trajectory, trajectory_fields)

    products = {"trajectory_enu_csv": str(trajectory_path)}
    warnings = []
    if transform["rms_residual_m"] > args.max_rms_warning:
        warnings.append("rms_residual_exceeds_warning_threshold")

    if args.points_csv:
        points = read_points_csv(args.points_csv)
        out_path = output_dir / (args.output_prefix + "_points_enu.csv")
        write_points_csv(str(out_path), points, transform)
        products["points_enu_csv"] = str(out_path)

    if args.pcd:
        try:
            header, fields, points = read_ascii_pcd(args.pcd)
            out_path = output_dir / (args.output_prefix + "_lidar_enu_ascii.pcd")
            write_ascii_pcd(str(out_path), header, fields, points, transform)
            products["lidar_enu_ascii_pcd"] = str(out_path)
        except ValueError as exc:
            warnings.append("pcd_not_exported: {}".format(exc))

    manifest = {
        "schema": "agv_mapping_georef_export_v1",
        "time_utc": utc_now(),
        "trajectory_input": args.trajectory,
        "points_csv_input": args.points_csv,
        "pcd_input": args.pcd,
        "datum": datum,
        "transform": transform,
        "warnings": warnings,
        "products": products,
    }
    manifest_path = output_dir / (args.output_prefix + "_georef_manifest.json")
    with manifest_path.open("w") as handle:
        json.dump(manifest, handle, allow_nan=False, indent=2, sort_keys=True)
    print(json.dumps({"manifest": str(manifest_path), "products": products, "warnings": warnings}, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
