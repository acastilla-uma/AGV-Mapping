#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/melodic/setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-$(cd "$SCRIPT_DIR/.." && pwd)}"
if [ -f "$WORKSPACE/devel/setup.bash" ]; then
  source "$WORKSPACE/devel/setup.bash"
fi

rosservice call /accumulator_node/save_accumulated "{}"
