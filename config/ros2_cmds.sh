#!/bin/bash

# Useful functions and stuff to manage ROS 2 from the shell.
# Roberto Masocco <robmasocco@gmail.com>
# January 31, 2023
# Copyright (c) 2025 Roberto Masocco

# Initialize ROS 2 Jazzy Jalisco according to the shell
if [[ $# -ne 0 ]]; then
  export ROS_DOMAIN_ID=$1
fi
export ROS_VERSION=2
export ROS_PYTHON_VERSION=3
if [[ ! -d /opt/ros/jazzy ]]; then
  echo >&2 "No ROS 2 distribution installed!"
  return 1
fi
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
current_shell=$(ps -p $$ | awk 'NR==2 {print $4}')

# Source the ROS 2 installation according to the current shell
if echo "$current_shell" | grep -q 'bash'; then
  source /opt/ros/$ROS_DISTRO/setup.bash
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
elif echo "$current_shell" | grep -q 'zsh'; then
  source /opt/ros/$ROS_DISTRO/setup.zsh
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
  # echo "Remember to source install/local_setup.zsh in your workspaces!"
else
  source /opt/ros/$ROS_DISTRO/setup.sh
fi
