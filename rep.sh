#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/indigo/setup.bash"
source "/catkin_ws/devel/setup.bash"

exec "$@"

## Modified from https://github.com/PabloGN/Docker-raspbian-ros-indigo/blob/master/rep.sh
