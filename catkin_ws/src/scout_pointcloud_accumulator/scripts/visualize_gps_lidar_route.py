#!/usr/bin/env python3
"""Create an offline HTML view of a LiDAR cloud with the GPS/TF route overlaid."""

import argparse
import csv
import html
import json
import math
import sys
from pathlib import Path


def safe_float(value):
    if value in (None, "", "?"):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(parsed) or math.isinf(parsed):
        return None
    return parsed


def truthy(value):
    return str(value).strip().lower() in ("1", "true", "yes", "ok", "fix", "valid")


def read_route(path):
    points = []
    with open(path, newline="") as handle:
        for row in csv.DictReader(handle):
            if not truthy(row.get("accepted", "1")) or not truthy(row.get("tf_ok", "1")):
                continue
            x = safe_float(row.get("map_x"))
            y = safe_float(row.get("map_y"))
            z = safe_float(row.get("map_z"))
            lat = safe_float(row.get("latitude"))
            lon = safe_float(row.get("longitude"))
            alt = safe_float(row.get("altitude"))
            if x is None or y is None or lat is None or lon is None:
                continue
            points.append({
                "seq": row.get("sample_seq", ""),
                "x": x,
                "y": y,
                "z": 0.0 if z is None else z,
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "sats": row.get("sats", ""),
                "hdop": row.get("hdop", ""),
                "age_ms": row.get("measurement_age_ms", ""),
                "recv_time_utc": row.get("recv_time_utc", ""),
            })
    if not points:
        raise ValueError("No accepted tf_ok route rows found in {}".format(path))
    return points


def parse_pcd_header(path):
    header = []
    fields = []
    point_count = None
    with open(path, "rb") as handle:
        while True:
            raw = handle.readline()
            if raw == b"":
                raise ValueError("Invalid PCD header: missing DATA line")
            try:
                line = raw.decode("ascii")
            except UnicodeDecodeError:
                raise ValueError("Invalid PCD header encoding")
            header.append(line)
            parts = line.strip().split()
            if not parts:
                continue
            key = parts[0].upper()
            if key == "FIELDS":
                fields = parts[1:]
            elif key == "POINTS" and len(parts) >= 2:
                point_count = int(parts[1])
            elif key == "DATA":
                if len(parts) < 2 or parts[1].lower() != "ascii":
                    raise ValueError("Only ASCII PCD is supported by this visualizer")
                return fields, point_count, handle.tell()


def read_ascii_pcd_points(path, max_points):
    fields, point_count, data_offset = parse_pcd_header(path)
    for required in ("x", "y", "z"):
        if required not in fields:
            raise ValueError("PCD is missing {} field".format(required))
    x_index = fields.index("x")
    y_index = fields.index("y")
    z_index = fields.index("z")
    intensity_index = fields.index("intensity") if "intensity" in fields else None
    stride = 1
    if point_count and max_points > 0:
        stride = max(1, int(math.ceil(float(point_count) / float(max_points))))
    points = []
    scanned = 0
    with open(path, "r", encoding="ascii", errors="strict") as handle:
        handle.seek(data_offset)
        for line in handle:
            if not line.strip():
                continue
            scanned += 1
            if (scanned - 1) % stride != 0:
                continue
            values = line.strip().split()
            if len(values) <= max(x_index, y_index, z_index):
                continue
            x = safe_float(values[x_index])
            y = safe_float(values[y_index])
            z = safe_float(values[z_index])
            if x is None or y is None or z is None:
                continue
            point = {"x": x, "y": y, "z": z}
            if intensity_index is not None and len(values) > intensity_index:
                intensity = safe_float(values[intensity_index])
                if intensity is not None:
                    point["i"] = intensity
            points.append(point)
    if not points:
        raise ValueError("No drawable points found in {}".format(path))
    return points, {"loaded": len(points), "stride": stride, "declared_points": point_count}


def bounds_for(points, route):
    xs = [p["x"] for p in points] + [p["x"] for p in route]
    ys = [p["y"] for p in points] + [p["y"] for p in route]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    if min_x == max_x:
        min_x -= 1.0
        max_x += 1.0
    if min_y == max_y:
        min_y -= 1.0
        max_y += 1.0
    pad_x = (max_x - min_x) * 0.04
    pad_y = (max_y - min_y) * 0.04
    return {
        "min_x": min_x - pad_x,
        "max_x": max_x + pad_x,
        "min_y": min_y - pad_y,
        "max_y": max_y + pad_y,
    }


def write_html(path, pcd_path, trajectory_path, points, route, stats):
    payload = {
        "cloud": points,
        "route": route,
        "bounds": bounds_for(points, route),
        "stats": stats,
        "pcd": str(pcd_path),
        "trajectory": str(trajectory_path),
    }
    route_rows = "\n".join(
        "<tr><td>{seq}</td><td>{lat:.8f}</td><td>{lon:.8f}</td><td>{x:.3f}</td><td>{y:.3f}</td><td>{hdop}</td><td>{sats}</td></tr>".format(
            seq=html.escape(str(row["seq"])),
            lat=row["lat"],
            lon=row["lon"],
            x=row["x"],
            y=row["y"],
            hdop=html.escape(str(row["hdop"])),
            sats=html.escape(str(row["sats"])),
        )
        for row in route
    )
    document = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>AGV LiDAR GPS Route Viewer</title>
<style>
body { margin: 0; font-family: Arial, sans-serif; color: #202124; background: #f5f7f9; }
header { padding: 14px 18px; background: #16202a; color: white; }
main { display: grid; grid-template-columns: minmax(520px, 1fr) 430px; gap: 12px; padding: 12px; }
canvas { width: 100%; height: calc(100vh - 118px); background: #0f1720; border: 1px solid #253548; }
aside { height: calc(100vh - 118px); overflow: auto; background: white; border: 1px solid #ccd3da; padding: 12px; }
.small { color: #5f6b76; font-size: 12px; }
.legend span { display: inline-block; margin-right: 14px; }
.dot { width: 9px; height: 9px; border-radius: 50%; margin-right: 5px; vertical-align: middle; }
.cloud { background: #aab5c0; }
.route { background: #ffb020; }
.selected { background: #4cc9f0; }
table { border-collapse: collapse; width: 100%; font-size: 12px; }
th, td { border-bottom: 1px solid #e5e8eb; padding: 5px 4px; text-align: right; }
th:first-child, td:first-child { text-align: left; }
#info { padding: 8px; background: #eef6ff; border: 1px solid #bdd9f5; margin: 10px 0; min-height: 72px; }
@media (max-width: 980px) { main { grid-template-columns: 1fr; } canvas, aside { height: 70vh; } }
</style>
</head>
<body>
<header>
  <strong>AGV LiDAR GPS Route Viewer</strong>
  <div class="small">Top-down map frame view. Click a route point to inspect its GPS coordinate.</div>
</header>
<main>
  <canvas id="view" width="1400" height="900"></canvas>
  <aside>
    <div class="legend">
      <span><i class="dot cloud"></i>LiDAR points</span>
      <span><i class="dot route"></i>AGV route</span>
      <span><i class="dot selected"></i>Selected GPS fix</span>
    </div>
    <p class="small">PCD: __PCD__<br>Trajectory: __TRAJ__<br>Cloud points shown: __POINTS__ (stride __STRIDE__)</p>
    <div id="info">Click a route point.</div>
    <table>
      <thead><tr><th>seq</th><th>lat</th><th>lon</th><th>map_x</th><th>map_y</th><th>hdop</th><th>sats</th></tr></thead>
      <tbody>__ROWS__</tbody>
    </table>
  </aside>
</main>
<script>
const data = __DATA__;
const canvas = document.getElementById('view');
const ctx = canvas.getContext('2d');
const info = document.getElementById('info');
let selected = null;

function project(p) {
  const b = data.bounds;
  const w = canvas.width, h = canvas.height;
  const sx = (p.x - b.min_x) / (b.max_x - b.min_x);
  const sy = (p.y - b.min_y) / (b.max_y - b.min_y);
  return {x: 35 + sx * (w - 70), y: h - 35 - sy * (h - 70)};
}

function draw() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = '#0f1720';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = 'rgba(180, 193, 204, 0.42)';
  for (const p of data.cloud) {
    const q = project(p);
    ctx.fillRect(q.x, q.y, 1.4, 1.4);
  }
  ctx.strokeStyle = '#ffb020';
  ctx.lineWidth = 3;
  ctx.beginPath();
  data.route.forEach((p, index) => {
    const q = project(p);
    if (index === 0) ctx.moveTo(q.x, q.y); else ctx.lineTo(q.x, q.y);
  });
  ctx.stroke();
  data.route.forEach((p, index) => {
    const q = project(p);
    ctx.fillStyle = selected === index ? '#4cc9f0' : '#ffb020';
    ctx.beginPath();
    ctx.arc(q.x, q.y, selected === index ? 7 : 4.5, 0, Math.PI * 2);
    ctx.fill();
    if (index === 0 || index === data.route.length - 1 || index % 10 === 0) {
      ctx.fillStyle = '#f7fbff';
      ctx.font = '13px Arial';
      ctx.fillText(String(p.seq || index), q.x + 8, q.y - 8);
    }
  });
}

function setInfo(index) {
  selected = index;
  const p = data.route[index];
  info.innerHTML = '<strong>GPS fix seq ' + (p.seq || index) + '</strong><br>' +
    'lat/lon: ' + p.lat.toFixed(8) + ', ' + p.lon.toFixed(8) + '<br>' +
    'map: x=' + p.x.toFixed(3) + ' y=' + p.y.toFixed(3) + ' z=' + p.z.toFixed(3) + '<br>' +
    'hdop=' + (p.hdop || '') + ' sats=' + (p.sats || '') + ' age_ms=' + (p.age_ms || '') + '<br>' +
    'time=' + (p.recv_time_utc || '');
  draw();
}

canvas.addEventListener('click', (event) => {
  const rect = canvas.getBoundingClientRect();
  const cx = (event.clientX - rect.left) * canvas.width / rect.width;
  const cy = (event.clientY - rect.top) * canvas.height / rect.height;
  let best = null;
  let bestDist = Infinity;
  data.route.forEach((p, index) => {
    const q = project(p);
    const d = Math.hypot(q.x - cx, q.y - cy);
    if (d < bestDist) { best = index; bestDist = d; }
  });
  if (best !== null && bestDist < 28) setInfo(best);
});

draw();
setInfo(0);
</script>
</body>
</html>
"""
    document = document.replace("__DATA__", json.dumps(payload, sort_keys=True))
    document = document.replace("__PCD__", html.escape(str(pcd_path)))
    document = document.replace("__TRAJ__", html.escape(str(trajectory_path)))
    document = document.replace("__POINTS__", str(stats["loaded"]))
    document = document.replace("__STRIDE__", str(stats["stride"]))
    document = document.replace("__ROWS__", route_rows)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(document, encoding="utf-8")


def build_parser():
    parser = argparse.ArgumentParser(description="Generate an HTML route/cloud viewer for GPS-associated AGV LiDAR maps.")
    parser.add_argument("--trajectory", required=True, help="trajectory_gps_map.csv from mapping_gps_metadata_logger.py")
    parser.add_argument("--pcd", required=True, help="ASCII PCD in map frame.")
    parser.add_argument("--output", required=True, help="Output HTML path.")
    parser.add_argument("--max-points", type=int, default=80000, help="Maximum cloud points embedded in the HTML.")
    return parser


def main():
    args = build_parser().parse_args()
    if args.max_points < 1:
        raise ValueError("--max-points must be >= 1")
    route = read_route(args.trajectory)
    points, stats = read_ascii_pcd_points(args.pcd, args.max_points)
    write_html(args.output, args.pcd, args.trajectory, points, route, stats)
    print(json.dumps({"output": args.output, "route_points": len(route), "cloud_points": stats["loaded"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("ERROR: {}".format(exc), file=sys.stderr)
        sys.exit(1)
