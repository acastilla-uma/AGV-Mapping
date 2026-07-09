#!/usr/bin/env python3
"""Small offline evaluator for scout mapping PCD snapshots.

The tool intentionally uses only the Python standard library so it can run on
the robot without adding dependencies.  It supports common ASCII and binary PCD
files containing float x/y/z fields (PointXYZI and PointXYZRGB outputs from the
accumulator are both covered).
"""

import argparse
import csv
import json
import math
import os
import struct
import sys
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


Point = Tuple[float, float, float]


def _parse_header_line(line: str) -> Tuple[str, List[str]]:
    parts = line.strip().split()
    if not parts:
        return "", []
    return parts[0].upper(), parts[1:]


def read_pcd(path: str) -> Tuple[Dict[str, object], List[Point]]:
    header: Dict[str, object] = {}
    header_bytes = 0
    with open(path, "rb") as stream:
      while True:
          line = stream.readline()
          if not line:
              raise ValueError("PCD header ended before DATA line: %s" % path)
          header_bytes += len(line)
          text = line.decode("ascii", errors="replace").strip()
          if not text or text.startswith("#"):
              continue
          key, values = _parse_header_line(text)
          if key == "DATA":
              if not values:
                  raise ValueError("DATA line has no mode in %s" % path)
              header["DATA"] = values[0].lower()
              break
          header[key] = values

    fields = header.get("FIELDS", [])
    sizes = [int(v) for v in header.get("SIZE", [])]
    types = header.get("TYPE", [])
    counts = [int(v) for v in header.get("COUNT", ["1"] * len(fields))]
    if not fields or len(fields) != len(sizes) or len(fields) != len(types):
        raise ValueError("Unsupported/incomplete PCD header in %s" % path)
    if len(counts) != len(fields):
        counts = [1] * len(fields)

    offsets: Dict[str, Tuple[int, int, str]] = {}
    offset = 0
    for field, size, typ, count in zip(fields, sizes, types, counts):
        offsets[field] = (offset, size, typ)
        offset += size * count
    point_step = offset
    for required in ("x", "y", "z"):
        if required not in offsets:
            raise ValueError("PCD lacks required field '%s': %s" % (required, path))
        _, size, typ = offsets[required]
        if size != 4 or typ.upper() != "F":
            raise ValueError("Only float32 x/y/z PCD fields are supported: %s" % path)

    data_mode = header["DATA"]
    points: List[Point] = []
    if data_mode == "ascii":
        with open(path, "r", encoding="ascii", errors="replace") as stream:
            in_data = False
            for line in stream:
                stripped = line.strip()
                if not stripped:
                    continue
                if not in_data:
                    key, _ = _parse_header_line(stripped)
                    if key == "DATA":
                        in_data = True
                    continue
                values = stripped.split()
                try:
                    x = float(values[fields.index("x")])
                    y = float(values[fields.index("y")])
                    z = float(values[fields.index("z")])
                except (ValueError, IndexError):
                    continue
                if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                    points.append((x, y, z))
    elif data_mode == "binary":
        with open(path, "rb") as stream:
            stream.seek(header_bytes)
            payload = stream.read()
        n_points = len(payload) // point_step
        for i in range(n_points):
            base = i * point_step
            coords = []
            for field in ("x", "y", "z"):
                field_offset, _, _ = offsets[field]
                coords.append(struct.unpack_from("<f", payload, base + field_offset)[0])
            x, y, z = coords
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                points.append((x, y, z))
    else:
        raise ValueError("Unsupported PCD DATA mode '%s' in %s" % (data_mode, path))

    return header, points


def percentile(values: Sequence[float], q: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * q
    low = int(math.floor(rank))
    high = int(math.ceil(rank))
    if low == high:
        return ordered[low]
    weight = rank - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


def evaluate(path: str, plane: Optional[Sequence[float]], sensor_origin: str) -> Dict[str, object]:
    header, points = read_pcd(path)
    ranges = [math.sqrt(x * x + y * y + z * z) for x, y, z in points]
    result: Dict[str, object] = {
        "schema": "scout_mapping_quality_eval_v1",
        "path": os.path.abspath(path),
        "sensor_origin": sensor_origin,
        "pcd_data": header.get("DATA"),
        "declared_points": int(header.get("POINTS", [len(points)])[0]) if header.get("POINTS") else len(points),
        "finite_points": len(points),
        "range_p50_m": percentile(ranges, 0.50),
        "range_p95_m": percentile(ranges, 0.95),
    }
    if points:
        xs, ys, zs = zip(*points)
        result["bbox_min"] = [min(xs), min(ys), min(zs)]
        result["bbox_max"] = [max(xs), max(ys), max(zs)]
    else:
        result["bbox_min"] = None
        result["bbox_max"] = None

    if plane is not None:
        a, b, c, d = plane
        norm = math.sqrt(a * a + b * b + c * c)
        if norm <= 0.0:
            raise ValueError("Plane normal must be non-zero.")
        signed = [(a * x + b * y + c * z + d) / norm for x, y, z in points]
        abs_dist = [abs(v) for v in signed]
        p95 = percentile(abs_dist, 0.95)
        p5_signed = percentile(signed, 0.05)
        p95_signed = percentile(signed, 0.95)
        result["plane"] = {"a": a, "b": b, "c": c, "d": d}
        result["plane_p95_m"] = p95
        result["plane_thickness_m"] = (
            None if p5_signed is None or p95_signed is None else p95_signed - p5_signed
        )

    return result


def write_csv(path: str, rows: Iterable[Dict[str, object]]) -> None:
    flat_rows = []
    for row in rows:
        flat = dict(row)
        flat["bbox_min"] = json.dumps(flat.get("bbox_min"))
        flat["bbox_max"] = json.dumps(flat.get("bbox_max"))
        flat["plane"] = json.dumps(flat.get("plane"))
        flat_rows.append(flat)
    if not flat_rows:
        return
    with open(path, "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=sorted(flat_rows[0].keys()))
        writer.writeheader()
        writer.writerows(flat_rows)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcd", nargs="+", help="PCD file(s) to evaluate")
    parser.add_argument("--plane", nargs=4, type=float, metavar=("A", "B", "C", "D"),
                        help="Optional plane ax + by + cz + d = 0 for plane metrics")
    parser.add_argument("--sensor-origin", default="unknown",
                        help="Label stored in the result, e.g. lidar or camera")
    parser.add_argument("--output-json", help="Write JSON result to this file instead of stdout")
    parser.add_argument("--output-csv", help="Also write a flattened CSV summary")
    args = parser.parse_args(argv)

    results = [evaluate(path, args.plane, args.sensor_origin) for path in args.pcd]
    payload: object = results[0] if len(results) == 1 else {"schema": "scout_mapping_quality_eval_batch_v1", "results": results}
    encoded = json.dumps(payload, indent=2, sort_keys=True)
    if args.output_json:
        with open(args.output_json, "w") as stream:
            stream.write(encoded)
            stream.write("\n")
    else:
        print(encoded)
    if args.output_csv:
        write_csv(args.output_csv, results)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        raise SystemExit(1)
