# Useful functions and stuff to manage ROS 2 from the shell.
# Roberto Masocco <robmasocco@gmail.com>
# November 19, 2021
# Copyright (c) 2021 Roberto Masocco

# Initialize ROS 2 Galactic Geochelone according to the shell.
# @param domain Domain ID to set initially; can be omitted.
ros2galactic() {
  if [[ $# -ne 0 ]]; then
    export ROS_DOMAIN_ID=$1
  fi
  export ROS_VERSION=2
  export ROS_PYTHON_VERSION=3
  # Check that the distribution is installed
  if [[ ! -d /opt/ros/galactic ]]; then
    >&2 echo "No such ROS 2 distribution installed!"
    return 1
  fi
  export ROS_DISTRO=galactic
  # Source the ROS 2 installation according to the current shell
  if [[ -n "$(echo $SHELL | grep 'bash')" ]]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
  elif [[ -n "$(echo $SHELL | grep 'zsh')" ]]; then
    source /opt/ros/$ROS_DISTRO/setup.zsh
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
    eval "$(register-python-argcomplete3 ros2)"
    # echo "Remember to source install/local_setup.zsh for your packages!"
  else
    source /opt/ros/$ROS_DISTRO/setup.sh
  fi
}

# Initialize ROS 2 Foxy Fitzroy according to the shell.
# @param domain Domain ID to set initially; can be omitted.
ros2foxy() {
  if [[ $# -ne 0 ]]; then
    export ROS_DOMAIN_ID=$1
  fi
  export ROS_VERSION=2
  export ROS_PYTHON_VERSION=3
  # Check that the distribution is installed
  if [[ ! -d /opt/ros/foxy ]]; then
    >&2 echo "No such ROS 2 distribution installed!"
    return 1
  fi
  export ROS_DISTRO=foxy
  # Source the ROS 2 installation according to the current shell
  if [[ -n "$(echo $SHELL | grep 'bash')" ]]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
  elif [[ -n "$(echo $SHELL | grep 'zsh')" ]]; then
    source /opt/ros/$ROS_DISTRO/setup.zsh
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
    eval "$(register-python-argcomplete3 ros2)"
    # echo "Remember to source install/local_setup.zsh for your packages!"
  else
    source /opt/ros/$ROS_DISTRO/setup.sh
  fi
}

# Initializes a ROS 2 workspace; must be ran inside it.
ros2wsinit() {
  if [[ -z "$ROS_DISTRO" ]]; then
    >&2 echo "No ROS 2 installation sourced!"
    return 1
  fi
  sudo rosdep init
  rosdep update
  rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
}

# Alias for Gazebo that includes environment variables for HiDPI
alias gazebo='QT_AUTO_SCREEN_SCALE_FACTOR=0 QT_SCREEN_SCALE_FACTORS=[1.0] /usr/bin/gazebo'
