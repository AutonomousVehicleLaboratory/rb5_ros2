#!/usr/bin/env bash
set -e

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install --yes bash-completion dirmngr curl gnupg2 lsb-release

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
echo "deb [arch=arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > \
    /etc/apt/sources.list.d/ros2-latest.list

apt-get update
apt-get install --yes \
  build-essential \
  cmake \
  git \
  python3-pip \
  python-rosdep \
  python3-vcstool \
  unzip \
  wget

python3 -m pip install -U \
  argcomplete \
  colcon-common-extensions \
  colcon-mixin \
  flake8 \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  pytest-cov \
  pytest-runner \
  setuptools

apt-get install --no-install-recommends --yes \
  libasio-dev \
  libtinyxml2-dev

apt-get install --yes libpython3-dev python3-pip
apt-get install --yes libopensplice69

export RTI_NC_LICENSE_ACCEPTED=YES
apt-get install --yes rti-connext-dds-5.3.1

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*
