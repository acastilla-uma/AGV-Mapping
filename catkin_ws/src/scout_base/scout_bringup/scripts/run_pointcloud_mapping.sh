#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

exec roslaunch scout_bringup pointcloud_mapping.launch
