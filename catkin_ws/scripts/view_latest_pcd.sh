#!/usr/bin/env bash
set -euo pipefail

export DISPLAY="${DISPLAY:-:0}"
MAP_DIR="${1:-/mnt/ros/maps}"

latest="$(ls -t "$MAP_DIR"/*.pcd 2>/dev/null | head -1 || true)"
if [ -z "$latest" ]; then
  echo "No PCD files found in $MAP_DIR"
  exit 1
fi

echo "Opening $latest"
exec pcl_viewer "$latest"

