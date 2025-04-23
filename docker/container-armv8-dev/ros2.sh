#!/usr/bin/env bash

# Shell functions and commands for ROS 2 management.
#
# Roberto Masocco <r.masocco@dotxautomation.com>
#
# June 13, 2024

# Copyright 2024 dotX Automation s.r.l.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# shellcheck disable=SC1090,SC2086

# Initialize ROS 2 environment according to the current shell.
ros2init() {
  # Configure ROS environment
  export ROS_VERSION=2
  export ROS_PYTHON_VERSION=3
  export ROS_DISTRO=jazzy

  # Configure RMW environment
  if [[ $# -ne 0 ]]; then
    export ROS_DOMAIN_ID=$1
  else
    export ROS_DOMAIN_ID=0
  fi
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
  export ROS_STATIC_PEERS=""

  # Configure RMW Zenoh environment
  export ZENOH_ROUTER_CHECK_ATTEMPTS=1
  export ZENOH_ROUTER_CONFIG_URI="/etc/zenoh/rmw/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5"
  export ZENOH_SESSION_CONFIG_URI="/etc/zenoh/rmw/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5"
  export RUST_LOG=zenoh=info

  local curr_shell
  curr_shell=$(ps -p $$ | awk 'NR==2 {print $4}')

  # Write an if statement that checks if the file /etc/dua/target contains the word "jetson"
  if grep -q "jetson" /etc/dua/target; then
    # Source additional stuff for colcon argcomplete
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.$curr_shell

    # Check that the ROS 2 installation is present, and source it
    if [[ -f /opt/ros/$ROS_DISTRO/setup.$curr_shell ]]; then
      source /opt/ros/$ROS_DISTRO/setup.$curr_shell
    elif [[ -f /opt/ros/$ROS_DISTRO/install/setup.$curr_shell ]]; then
      source /opt/ros/$ROS_DISTRO/install/setup.$curr_shell
    else
      echo >&2 "ROS 2 installation not found."
      return 1
    fi
  else
    # Check that the ROS 2 installation is present, and source it
    if [[ -f /opt/ros/$ROS_DISTRO/setup.$curr_shell ]]; then
      source /opt/ros/$ROS_DISTRO/setup.$curr_shell
    elif [[ -f /opt/ros/$ROS_DISTRO/install/setup.$curr_shell ]]; then
      source /opt/ros/$ROS_DISTRO/install/setup.$curr_shell
    else
      echo >&2 "ROS 2 installation not found."
      return 1
    fi

    # Source additional stuff for colcon argcomplete
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.$curr_shell
  fi

  # Source Ignition Gazebo stuff
  if [[ -f /opt/gazebo/harmonic/install/setup.$curr_shell ]]; then
    source /opt/gazebo/harmonic/install/setup.$curr_shell
  fi
  if [[ -f /opt/ros/ros_gz/install/local_setup.$curr_shell ]]; then
    source /opt/ros/ros_gz/install/local_setup.$curr_shell
  fi

  # Source additional DUA stuff
  if [[ -f /opt/ros/dua-utils/install/local_setup.$curr_shell ]]; then
    source /opt/ros/dua-utils/install/local_setup.$curr_shell
  fi

  # Source workspace if present
  if [[ -f /home/neo/workspace/install/local_setup.$curr_shell ]]; then
    source /home/neo/workspace/install/local_setup.$curr_shell
  fi
}

# Alias for colcon build command with maximum output
alias cbuild='colcon build --event-handlers console_direct+ --symlink-install --continue-on-error'

# Aliases for ROS 2 daemon management
alias ros2start='ros2 daemon start'
alias ros2stop='ros2 daemon stop'
alias ros2status='ros2 daemon status'
alias ros2reset='ros2 daemon stop; ros2 daemon start'

# Helper function to validate ros2bag arguments
function _ros2bag_validate_args {
  # First argument must be either record or play
  if [[ "$1" != "record" ]] && [[ "$1" != "play" ]]; then
    return 1
  fi

  # For record mode, we need either 4 or 5 arguments
  if [[ "$1" == "record" ]]; then
    # First check if --use-sim-time is present
    local use_sim_time=false
    if [[ "$2" == "--use-sim-time" ]]; then
      use_sim_time=true
      # Shift arguments for easier validation
      shift 2
    else
      shift 1
    fi

    # Now check remaining arguments
    if [[ $# -ne 3 ]]; then
      return 1
    fi
    # Check that topics file exists and is readable
    if [[ ! -f "$1" ]]; then
      return 2
    fi
    if [[ ! -r "$1" ]]; then
      return 3
    fi
    # Check that cache size is a positive integer
    if ! [[ "$3" =~ ^[0-9]+$ ]]; then
      return 4
    fi
  fi

  # For play mode, we need at least 2 arguments (play + bag path)
  if [[ "$1" == "play" ]]; then
    if [[ $# -lt 2 ]]; then
      return 1
    fi
    # Check that bag directory exists
    if [[ ! -d "$2" ]]; then
      return 5
    fi
  fi

  return 0
}

# Function to record or play ROS 2 bags with various options
function ros2bag {
  function usage {
    echo >&2 "Usage:"
    echo >&2 "    ros2bag record [--use-sim-time] TOPICS_FILE OUTPUT_DIR CACHE_SIZE"
    echo >&2 "    ros2bag play BAG_PATH [PLAY_OPTIONS...]"
    echo >&2 ""
    echo >&2 "Arguments for record mode:"
    echo >&2 "    --use-sim-time: Optional. If present, enables ROS time simulation support."
    echo >&2 "                    No message will be recorded until the first message is received"
    echo >&2 "                    on the /clock topic."
    echo >&2 "    TOPICS_FILE:    Path to a text file containing full names of the topics to sample,"
    echo >&2 "                    one per line."
    echo >&2 "    OUTPUT_DIR:     Name of the directory to create in ~/workspace/logs/ to store the"
    echo >&2 "                    sampled data."
    echo >&2 "    CACHE_SIZE:     Approximately the amount of data in bytes that must be recorded"
    echo >&2 "                    within one second of sampling."
    echo >&2 ""
    echo >&2 "Arguments for play mode:"
    echo >&2 "    BAG_PATH:     Path to the ROS 2 bag directory."
    echo >&2 "    PLAY_OPTIONS: Additional options to pass to ros2 bag play."
    echo >&2 ""
    echo >&2 "The function will:"
    echo >&2 "    - In record mode: Start the recording process, which can be stopped with Ctrl+C."
    echo >&2 "    - In play mode:   Play back the bag with clock publishing enabled."
    echo >&2 "                      Remember to reset RViz and set use_sim_time to true everywhere!"
  }

  # Show help if requested
  if [[ "${1-}" =~ ^-*h(elp)?$ ]]; then
    usage
    return 1
  fi

  # Validate arguments
  _ros2bag_validate_args "$@"
  local validate_result=$?
  case $validate_result in
    0) ;;  # All good
    1)
      echo >&2 "ERROR: Invalid arguments"
      usage
      return 1
      ;;
    2)
      if [[ "$2" == "--use-sim-time" ]]; then
        echo >&2 "ERROR: Topics file does not exist: $3"
      else
        echo >&2 "ERROR: Topics file does not exist: $2"
      fi
      return 1
      ;;
    3)
      if [[ "$2" == "--use-sim-time" ]]; then
        echo >&2 "ERROR: Topics file is not readable: $3"
      else
        echo >&2 "ERROR: Topics file is not readable: $2"
      fi
      return 1
      ;;
    4)
      echo >&2 "ERROR: Cache size must be a positive integer"
      return 1
      ;;
    5)
      echo >&2 "ERROR: Bag directory does not exist: $2"
      return 1
      ;;
    *)
      echo >&2 "ERROR: Unknown validation error"
      return 1
      ;;
  esac

  # Handle record mode
  if [[ "$1" == "record" ]]; then
    # Check for --use-sim-time and prepare arguments
    local topics_file output_dir cache_size use_sim_time_opt
    if [[ "$2" == "--use-sim-time" ]]; then
      use_sim_time_opt="--use-sim-time"
      topics_file="$3"
      output_dir="$4"
      cache_size="$5"
    else
      use_sim_time_opt=""
      topics_file="$2"
      output_dir="$3"
      cache_size="$4"
    fi

    # Read topics from file
    local TOPICS=()
    while IFS= read -r line; do
      TOPICS+=("$line")
    done < "$topics_file"

    # Check that we have topics to record
    if [[ ${#TOPICS[@]} -eq 0 ]]; then
      echo >&2 "ERROR: No topics to record!"
      return 1
    fi

    # Show topics that will be recorded
    echo "The following topics will be recorded:"
    for topic in "${TOPICS[@]}"; do
      echo "$topic"
    done
    if [[ -n "$use_sim_time_opt" ]]; then
      echo -e "\nExternal time source will be used"
    fi
    printf '\n'

    # Wait for user acknowledgment
    read -r -p "Press Enter to start recording..."

    # Start recording
    if ! ros2 bag record \
      -o "/home/neo/workspace/logs/$output_dir" \
      --include-unpublished-topics \
      --max-cache-size "$cache_size" \
      $use_sim_time_opt \
      "${TOPICS[@]}"; then
      echo >&2 "ERROR: ros2 bag record failed"
      return 1
    fi

    return 0
  fi

  # Handle play mode
  if [[ "$1" == "play" ]]; then
    # Shift away the command and bag path
    local bag_path="$2"
    shift 2
    # Play the bag with standard options plus any additional ones
    if ! ros2 bag play -p --clock 10 --wait-for-all-acked 5000 "$@" "$bag_path"; then
      echo >&2 "ERROR: ros2 bag play failed"
      return 1
    fi
    return 0
  fi
}
