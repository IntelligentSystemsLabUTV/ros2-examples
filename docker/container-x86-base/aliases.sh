#!/usr/bin/env bash

# Custom aliases for container internal shell.
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

# Add custom, general-purpose aliases here.
# You can also source other files from sub-units included by this project.

alias ls='ls --color=auto'
alias ll='ls -lah'
alias valgrind-check='valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --verbose --log-file=valgrind.out'

# Source DUA commands
source ~/.dua_submod.sh
source ~/.dua_subtree.sh

# Routine to convert an angle in degrees [-180° +180°] to radians [-PI +PI].
function degrad {
  local angle_in_degrees="$1"
  angle_in_radians=$(python3 -c "import sys, math; angle=float(sys.argv[1]); print(math.radians((angle + 180) % 360 - 180))" "$angle_in_degrees")
  echo "$angle_in_radians"
}

# Routine to update dua-utils.
function utils-update {
  local repos_file

  # Get the current shell
  local curr_shell
  curr_shell=$(ps -p $$ | awk 'NR==2 {print $4}')

  # Download new repos file
  if [ -f /opt/dua-utils_repos_base.yaml ]; then
    wget -O /opt/dua-utils_repos_base.yaml https://raw.githubusercontent.com/dotX-Automation/dua-foundation/refs/heads/master/scripts/ros2/dua-utils_repos_base.yaml
    repos_file=/opt/dua-utils_repos_base.yaml
  elif [ -f /opt/dua-utils_repos_dev.yaml ]; then
    wget -O /opt/dua-utils_repos_dev.yaml https://raw.githubusercontent.com/dotX-Automation/dua-foundation/refs/heads/master/scripts/ros2/dua-utils_repos_dev.yaml
    repos_file=/opt/dua-utils_repos_dev.yaml
  else
    echo >&2 "No repos file found."
    return 1
  fi

  # Perform updates
  if [ -x /opt/build_dua_utils.sh ]; then
    echo "Cloning and building dua-utils from $repos_file ..."
    sh -c "rm -rf /opt/ros/dua-utils/*"
    sh -c "/opt/build_dua_utils.sh jazzy $repos_file"
  else
    echo >&2 "No build script found."
    return 1
  fi

  # Source new installation
  source "/opt/ros/dua-utils/install/local_setup.$curr_shell"
}

# Routine to configure Cyclone DDS to use specific network interfaces.
function cyclonedds-configure {
  local cyclonedds_uri='<CycloneDDS><Domain><General><Interfaces>'
  for interface in "$@"; do
    cyclonedds_uri+="<NetworkInterface name=\"$interface\" priority=\"default\" multicast=\"true\"/>"
  done
  cyclonedds_uri+='</Interfaces></General></Domain></CycloneDDS>'
  export CYCLONEDDS_URI="$cyclonedds_uri"
}
