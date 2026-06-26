#!/usr/bin/env python3
"""Show latest GPS/DOBACK/combined metadata CSV rows from AGV mapping sessions."""

import argparse
import csv
import os
import sys
import time
from pathlib import Path


DEFAULT_FILES = {
    "gps": "gps.csv",
    "doback": "doback.csv",
    "track": "trayectoria_gps_doback.csv",
    "agv": "trayectoria_agv_mapa.csv",
}


def repo_root_from_script():
    return Path(__file__).resolve().parents[2]


def latest_session(datos_dir):
    datos_dir = Path(datos_dir)
    candidates = []
    if not datos_dir.exists():
        return None
    for child in datos_dir.iterdir():
        if not child.is_dir():
            continue
        if any((child / name).exists() for name in DEFAULT_FILES.values()):
            candidates.append(child)
    if not candidates:
        return None
    candidates.sort(key=lambda path: path.stat().st_mtime, reverse=True)
    return candidates[0]


def read_last_csv_row(path):
    path = Path(path)
    if not path.exists():
        return None, 0
    last = None
    count = 0
    with path.open(newline="", encoding="utf-8", errors="replace") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            count += 1
            last = row
    return last, count


def pick(row, *keys):
    if not row:
        return ""
    for key in keys:
        value = row.get(key, "")
        if value not in (None, ""):
            return value
    return ""


def short(value, max_len=90):
    value = str(value or "")
    if len(value) <= max_len:
        return value
    return value[: max_len - 3] + "..."


def print_summary(session_dir):
    gps, gps_count = read_last_csv_row(session_dir / DEFAULT_FILES["gps"])
    doback, doback_count = read_last_csv_row(session_dir / DEFAULT_FILES["doback"])
    track, track_count = read_last_csv_row(session_dir / DEFAULT_FILES["track"])
    agv, agv_count = read_last_csv_row(session_dir / DEFAULT_FILES["agv"])

    print("=" * 78)
    print("Sesion metadata: {}".format(session_dir))
    print("GPS muestras: {} | DOBACK muestras: {} | Combinadas: {} | Trayectoria AGV: {}".format(gps_count, doback_count, track_count, agv_count))

    if gps:
        print("\nGPS ultimo:")
        print("  tiempo={} host={}".format(pick(gps, "recv_time_utc"), pick(gps, "remote_host")))
        print("  latitud={} longitud={} altitud_m={}".format(
            pick(gps, "latitud", "latitude"), pick(gps, "longitud", "longitude"), pick(gps, "altitud_m", "altitude")
        ))
        print("  satelites={} hdop={} fix_valido={} esperando_fix={}".format(
            pick(gps, "satelites", "sats"), pick(gps, "hdop"), pick(gps, "fix_valido", "fix_ok"), pick(gps, "esperando_fix", "waiting_for_fix")
        ))
        print("  mensaje={}".format(short(pick(gps, "mensaje_gps", "text", "gps_text"))))
    else:
        print("\nGPS ultimo: sin muestras todavia")

    if doback:
        print("\nDOBACK ultimo:")
        print("  tiempo={}".format(pick(doback, "recv_time_utc")))
        print("  ax={} ay={} az={} accmag={}".format(pick(doback, "ax"), pick(doback, "ay"), pick(doback, "az"), pick(doback, "accmag")))
        print("  gx={} gy={} gz={}".format(pick(doback, "gx"), pick(doback, "gy"), pick(doback, "gz")))
        print("  roll={} pitch={} yaw={} si={}".format(pick(doback, "roll"), pick(doback, "pitch"), pick(doback, "yaw"), pick(doback, "si")))
    else:
        print("\nDOBACK ultimo: sin muestras todavia")

    if track:
        print("\nFila combinada ultima:")
        print("  latitud={} longitud={} doback_ok={} doback_age_sec={} tf_ok={}".format(
            pick(track, "latitud", "latitude"), pick(track, "longitud", "longitude"), pick(track, "doback_ok"), pick(track, "doback_age_sec"), pick(track, "tf_ok")
        ))
        print("  map_x={} map_y={} map_yaw={}".format(pick(track, "map_x"), pick(track, "map_y"), pick(track, "map_yaw")))
    else:
        print("\nFila combinada ultima: sin muestras todavia")

    if agv:
        print("\nTrayectoria AGV mapa ultima:")
        print("  map_x={} map_y={} map_z={} map_yaw={}".format(
            pick(agv, "map_x"), pick(agv, "map_y"), pick(agv, "map_z"), pick(agv, "map_yaw")
        ))
        print("  delta_m={} total_m={}".format(pick(agv, "distance_from_previous_m"), pick(agv, "distance_total_m")))
    else:
        print("\nTrayectoria AGV mapa ultima: sin TF map -> base_link todavia")
    sys.stdout.flush()


def build_parser():
    parser = argparse.ArgumentParser(description="Monitor latest GPS/DOBACK metadata CSV files.")
    parser.add_argument("--metadata-dir", help="Specific session folder containing gps.csv/doback.csv/trayectoria_gps_doback.csv.")
    parser.add_argument("--datos-dir", default=str(repo_root_from_script() / "datos"), help="Parent datos folder used to auto-detect latest session.")
    parser.add_argument("--watch", type=float, default=0.0, help="Refresh every N seconds. 0 prints once.")
    return parser


def main():
    args = build_parser().parse_args()
    session = Path(args.metadata_dir) if args.metadata_dir else latest_session(args.datos_dir)
    if session is None:
        print("No encuentro sesiones con CSV dentro de {}".format(args.datos_dir), file=sys.stderr)
        return 1
    while True:
        print_summary(session)
        if args.watch <= 0:
            break
        time.sleep(args.watch)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
