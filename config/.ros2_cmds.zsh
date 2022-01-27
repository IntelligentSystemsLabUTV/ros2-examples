# Useful functions and stuff to manage ROS 2 from the Zsh shell.
#
# Roberto Masocco <robmasocco@gmail.com>
# Alessandro Tenaglia <alessandro.tenaglia42@gmail.com>
#
# January 27, 2022

# Initialize ROS 2 environment according to the Zsh shell.
# @param domain Domain ID to set initially; can be omitted.
ros2init() {
  if [[ $# -ne 0 ]]; then
    export ROS_DOMAIN_ID=$1
  fi
  export ROS_VERSION=2
  export ROS_PYTHON_VERSION=3
  export ROS_DISTRO=galactic

  # Source the ROS 2 installation with Zsh scripts
  source /opt/ros/$ROS_DISTRO/setup.zsh
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
  eval "$(register-python-argcomplete3 ros2)"
  # echo "Remember to source install/local_setup.zsh for your packages!"
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
