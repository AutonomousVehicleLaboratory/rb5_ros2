#!/usr/bin/env bash
set -e

export DEBIAN_FRONTEND=noninteractive
apt-get update

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > \
    /etc/apt/sources.list.d/ros2-latest.list

apt-get update

apt-get install --yes libpython3-dev python3-pip python3-argcomplete

apt-get install --yes ros-dashing-desktop

apt-get install --yes ros-dashing-rmw-opensplice-cpp

export RTI_NC_LICENSE_ACCEPTED=YES
apt-get install --yes ros-dashing-rmw-connext-cpp

apt-get install --yes ros-dashing-launch-testing-ros ros-dashing-launch-ros ros-dashing-launch-testing ros-dashing-launch-ros-sandbox ros-dashing-launch ros-dashing-launch-testing-ament-cmake

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
