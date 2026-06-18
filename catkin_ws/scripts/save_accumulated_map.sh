#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/melodic/setup.bash
WORKSPACE="${WORKSPACE:-/mnt/ros/catkin_ws}"
if [ -f "$WORKSPACE/devel/setup.bash" ]; then
  source "$WORKSPACE/devel/setup.bash"
fi

rosservice call /accumulator_node/save_accumulated "{}"
