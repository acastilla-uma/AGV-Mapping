#!/usr/bin/env bash
set -euo pipefail

export DISPLAY="${DISPLAY:-:0}"
MAP_DIR="${1:-/mnt/ros/maps}"

latest="$(find "$MAP_DIR" -type f -name '*.pcd' -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2- || true)"
if [ -z "$latest" ]; then
  echo "No PCD files found in $MAP_DIR"
  exit 1
fi

echo "Opening $latest"
exec pcl_viewer "$latest"
