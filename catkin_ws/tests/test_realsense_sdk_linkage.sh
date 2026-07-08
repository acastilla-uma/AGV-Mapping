#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PLUGIN="$ROOT_DIR/catkin_ws/devel/lib/librealsense2_camera.so"
CMAKE_FILE="$ROOT_DIR/catkin_ws/src/realsense/realsense-ros/realsense2_camera/CMakeLists.txt"

if [ ! -f "$PLUGIN" ]; then
  echo "FAIL: RealSense ROS plugin is not built: $PLUGIN" >&2
  exit 1
fi

required_sdk="$(sed -n 's/^set(REALSENSE2_REQUIRED_VERSION "\([^"]*\)")$/\1/p' "$CMAKE_FILE")"
required_abi="${required_sdk%.*}"
linked_sdk="$(ldd "$PLUGIN" | sed -n 's/.*librealsense2\.so\.\([^ ]*\).*/\1/p')"
built_sdk="$(strings "$PLUGIN" | grep -Fx "$required_sdk" | head -n 1 || true)"

if [ -z "$required_sdk" ] || [ "$linked_sdk" != "$required_abi" ] || [ "$built_sdk" != "$required_sdk" ]; then
  echo "FAIL: RealSense ROS SDK mismatch (headers=$built_sdk library=$linked_sdk)." >&2
  exit 1
fi

echo "RealSense ROS SDK linkage: PASS (headers=$built_sdk library=$linked_sdk)"
