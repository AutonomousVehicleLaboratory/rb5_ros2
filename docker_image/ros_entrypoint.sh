#!/usr/bin/env bash
set -e

source "/opt/ros/dashing/setup.bash"
source "/root/colcon_ws/install/setup.bash"

exec "$@"
