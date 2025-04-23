#!/usr/bin/env bash

# DUA submodules management function.
#
# Roberto Masocco <r.masocco@dotxautomation.com>
# Alessandro Tenaglia <a.tenaglia@dotxautomation.com>
#
# June 14, 2024

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

function dua-submod {
  function usage {
    echo >&2 "Usage:"
    echo >&2 "    dua-submod [update|status]"
    echo >&2 "See dua-template.md for more info."
  }

  if [[ "${1-}" =~ ^-*h(elp)?$ ]]; then
    usage
    return 1
  fi

  # Function to update the submodules.
  function update {
    git submodule update --init --recursive
  }

  # Function to check the status of the submodules.
  function status {
    git submodule status --recursive
  }

  # Check if we have a command and run the specified function.
  case "${1-}" in
  update)
    shift
    update
    ;;
  status)
    shift
    status
    ;;
  *)
    echo >&2 "Unknown command: ${1-}"
    usage
    return 1
    ;;
  esac
}
