#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/melodic/setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-$(cd "$SCRIPT_DIR/.." && pwd)}"
if [ -f "$WORKSPACE/devel/setup.bash" ]; then
  source "$WORKSPACE/devel/setup.bash"
fi

rosservice call /accumulator_node/save_accumulated "{}"

if rosnode list 2>/dev/null | grep -qx "/mapping_metadata_logger"; then
  rosservice call /mapping_metadata_logger/save_metadata "{}"
else
  echo "WARNING: /mapping_metadata_logger is not running; only PCD files were saved."
fi
