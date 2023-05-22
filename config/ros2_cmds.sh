#!/bin/bash

# Useful functions and stuff to manage ROS 2 from the shell.
# Roberto Masocco <robmasocco@gmail.com>
# January 31, 2023
# Copyright (c) 2023 Roberto Masocco

# Initialize ROS 2 Humble Hawksbill according to the shell
if [[ $# -ne 0 ]]; then
  export ROS_DOMAIN_ID=$1
fi
export ROS_VERSION=2
export ROS_PYTHON_VERSION=3
# Check that the distribution is installed
if [[ ! -d /opt/ros/humble ]]; then
  >&2 echo "No such ROS 2 distribution installed!"
  return 1
fi
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Source the ROS 2 installation according to the current shell
if echo "$SHELL" | grep -q 'bash'; then
  # shellcheck source=/dev/null
  source /opt/ros/$ROS_DISTRO/setup.bash
  # shellcheck source=/dev/null
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
elif echo "$SHELL" | grep -q 'zsh'; then
  # shellcheck source=/dev/null
  source /opt/ros/$ROS_DISTRO/setup.zsh
  # shellcheck source=/dev/null
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
  eval "$(register-python-argcomplete3 ros2)"
  # echo "Remember to source install/local_setup.zsh in your workspaces!"
else
  # shellcheck source=/dev/null
  source /opt/ros/$ROS_DISTRO/setup.sh
fi

# Alias for Gazebo that includes environment variables for HiDPI
alias gazebo='QT_AUTO_SCREEN_SCALE_FACTOR=0 QT_SCREEN_SCALE_FACTORS=[1.0] /usr/bin/gazebo'
