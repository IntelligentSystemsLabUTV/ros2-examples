#!/usr/bin/env bash

# Shell functions and commands for ROS 2 management.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# April 4, 2023

# shellcheck disable=SC1090

# Initialize ROS 2 environment according to the Zsh shell.
ros2init() {
  if [[ $# -ne 0 ]]; then
    export ROS_DOMAIN_ID=$1
  fi
  export ROS_VERSION=2
  export ROS_PYTHON_VERSION=3
  export ROS_DISTRO=humble

  # Check that the ROS 2 installation is present, and source it
  if [[ -f /opt/ros/$ROS_DISTRO/setup.zsh ]]; then
    source /opt/ros/$ROS_DISTRO/setup.zsh
  elif [[ -f /opt/ros/$ROS_DISTRO/install/setup.zsh ]]; then
    source /opt/ros/$ROS_DISTRO/install/setup.zsh
  else
    echo >&2 "ROS 2 installation not found."
    return 1
  fi

  # Source additional stuff for colcon argcomplete
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh

  # Source Ignition Gazebo stuff
  if [[ -f /opt/gazebo/fortress/install/setup.zsh ]]; then
    source /opt/gazebo/fortress/install/setup.zsh
  fi
  if [[ -f /opt/ros/ros_gz/install/local_setup.zsh ]]; then
    source /opt/ros/ros_gz/install/local_setup.zsh
  fi

  # Source our fork of rmw_fastrtps
  if [[ -f /opt/ros/rmw_fastrtps/install/local_setup.zsh ]]; then
    source /opt/ros/rmw_fastrtps/install/local_setup.zsh
  fi

  # Source additional DUA stuff
  if [[ -f /opt/ros/dua-utils/install/local_setup.zsh ]]; then
    source /opt/ros/dua-utils/install/local_setup.zsh
  fi

  # Source workspace if present
  if [[ -f /home/neo/workspace/install/local_setup.zsh ]]; then
    source /home/neo/workspace/install/local_setup.zsh
  fi
}

# Alias for Gazebo Classic that includes environment variables for HiDPI
alias gazebo='QT_AUTO_SCREEN_SCALE_FACTOR=0 QT_SCREEN_SCALE_FACTORS=[1.0] /usr/bin/gazebo'
