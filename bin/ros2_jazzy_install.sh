#!/bin/bash

# ROS 2 Jazzy Jalisco installation script.
# Roberto Masocco <robmasocco@gmail.com>
# April 23, 2025
# Copyright (c) 2025 Roberto Masocco

set -o errexit
set -o nounset
set -o pipefail
if [[ "${TRACE-0}" == "1" ]]; then set -o xtrace; fi

if [[ "${1-}" =~ ^-*h(elp)?$ ]]; then
  echo >&2 "Usage:"
  echo >&2 "    ros2_jazzy_install.sh"
  exit 1
fi

echo "Checking Universe repository..."
if ! apt-cache policy | grep -q 'universe'; then
	sudo apt install software-properties-common
	sudo add-apt-repository universe
fi

echo "Adding ROS 2 apt repository..."
sudo apt update && sudo apt install -y curl gnupg lsb-release python3 python3-pip
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

echo "Installing ROS 2 main packages..."
sudo apt install -y \
  ros-dev-tools \
  ros-jazzy-actuator-msgs \
  ros-jazzy-ament-lint \
  ros-jazzy-angles \
  ros-jazzy-cv-bridge \
  ros-jazzy-diagnostic-msgs \
  ros-jazzy-diagnostic-updater \
  ros-jazzy-eigen3-cmake-module \
  ros-jazzy-filters \
  ros-jazzy-geographic-msgs \
  ros-jazzy-geographic-info \
  ros-jazzy-gps-msgs \
  ros-jazzy-gps-tools \
  ros-jazzy-gps-umd \
  ros-jazzy-gpsd-client \
  ros-jazzy-image-common \
  ros-jazzy-image-geometry \
  ros-jazzy-image-pipeline \
  ros-jazzy-image-transport \
  ros-jazzy-image-transport-plugins \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-laser-filters \
  ros-jazzy-launch-testing \
  ros-jazzy-launch-testing-ament-cmake \
  ros-jazzy-launch-testing-ros \
  ros-jazzy-perception \
  ros-jazzy-perception-pcl \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-rmw-fastrtps-cpp \
  ros-jazzy-rmw-zenoh-cpp \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-desktop \
  ros-jazzy-rosidl-generator-dds-idl \
  ros-jazzy-rqt-robot-steering \
  ros-jazzy-vision-msgs \
  ros-jazzy-vision-opencv \
  ros-jazzy-xacro
