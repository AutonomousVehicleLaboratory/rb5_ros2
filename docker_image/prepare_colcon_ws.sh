#!/usr/bin/env bash
set -e

source "/opt/ros/dashing/setup.bash"

export DEBIAN_FRONTEND=noninteractive
apt-get update

COLCON_WS=/root/colcon_ws
mkdir -p $COLCON_WS/src

cd $COLCON_WS
colcon build --symlink-install

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
