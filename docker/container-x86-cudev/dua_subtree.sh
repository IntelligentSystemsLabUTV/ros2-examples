#!/usr/bin/env bash

# DUA subtrees management function.
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

function dua-subtree {
  function usage {
    echo >&2 "Usage:"
    echo >&2 "    dua-subtree [add|remove|pull|push|remote-add|remote-remove] ARGS"
    echo >&2 "See dua-template.md for more info."
  }

  if [[ "${1-}" =~ ^-*h(elp)?$ ]]; then
    usage
    return 1
  fi

  # Function to add a subtree.
  function add {
    # Check input arguments
    if [[ $# -ne 3 ]]; then
      echo >&2 "Usage:"
      echo >&2 "    dua-subtree add REMOTE PREFIX BRANCH"
      echo >&2 "REMOTE may be an URL or a preexisting remote name."
      echo >&2 "PREFIX is the path to the subtree in the local repo."
      echo >&2 "BRANCH is the branch to pull from."
      return 1
    fi

    # Sanity-check the prefix: it must not exist yet
    if [[ -d "${2}" ]]; then
      echo >&2 "ERROR: prefix path ${2} already exists"
      return 1
    fi

    # Add the requested subtree
    git subtree add --prefix="${2}" "${1}" "${3}" --squash
  }

  # Function to remove a subtree.
  function remove {
    # Check input arguments
    if [[ $# -ne 1 ]]; then
      echo >&2 "Usage:"
      echo >&2 "    dua-subtree remove PREFIX"
      echo >&2 "PREFIX is the path to the subtree in the local repo."
      return 1
    fi

    # Check if the prefix exists
    if [[ ! -d "${1}" ]]; then
      echo >&2 "ERROR: prefix path ${1} does not exist"
      return 1
    fi

    # Remove the requested subtree
    git rm -r "${1}"
    echo "Remember to commit the removal!"
  }

  # Function to pull from a subtree.
  function pull {
    # Check input arguments
    if [[ $# -ne 3 ]]; then
      echo >&2 "Usage:"
      echo >&2 "    dua-subtree pull REMOTE PREFIX BRANCH"
      echo >&2 "REMOTE may be an URL or a preexisting remote name."
      echo >&2 "PREFIX is the path to the subtree in the local repo."
      echo >&2 "BRANCH is the branch to pull from."
      return 1
    fi

    # Check if the prefix exists
    if [[ ! -d "${2}" ]]; then
      echo >&2 "ERROR: prefix path ${2} does not exist"
      return 1
    fi

    # Pull from the requested subtree
    git subtree pull --prefix="${2}" "${1}" "${3}" --squash
  }

  # Function to push to a subtree.
  function push {
    # Check input arguments
    if [[ $# -ne 3 ]]; then
      echo >&2 "Usage:"
      echo >&2 "    dua-subtree push REMOTE PREFIX BRANCH"
      echo >&2 "REMOTE may be an URL or a preexisting remote name."
      echo >&2 "PREFIX is the path to the subtree in the local repo."
      echo >&2 "BRANCH is the branch to push to."
      return 1
    fi

    # Check if the prefix exists
    if [[ ! -d "${2}" ]]; then
      echo >&2 "ERROR: prefix path ${2} does not exist"
      return 1
    fi

    # Push to the requested subtree
    git subtree push --prefix="${2}" "${1}" "${3}"
  }

  # Function to add a remote to a subtree.
  function remote_add {
    # Check input arguments
    if [[ $# -ne 2 ]]; then
      echo >&2 "Usage:"
      echo >&2 "    dua-subtree remote-add NAME URL"
      echo >&2 "NAME is the name of the remote to add."
      echo >&2 "URL is the URL of the remote to add."
      return 1
    fi

    # Add the requested remote
    git remote add -f "${1}" "${2}"
  }

  # Function to remove a remote from a subtree.
  function remote_remove {
    # Check input arguments
    if [[ $# -ne 1 ]]; then
      echo >&2 "Usage:"
      echo >&2 "    dua-subtree remote-remove NAME"
      echo >&2 "NAME is the name of the remote to remove."
      return 1
    fi

    # Remove the requested remote
    git remote remove "${1}"
  }

  # Function to rename a remote.
  function remote_rename {
    # Check input arguments
    if [[ $# -ne 2 ]]; then
      echo >&2 "Usage:"
      echo >&2 "    dua-subtree remote-rename OLD NEW"
      echo >&2 "OLD is the name of the remote to rename."
      echo >&2 "NEW is the new name of the remote."
      return 1
    fi

    # Rename the requested remote
    git remote rename "${1}" "${2}"
  }

  # Check if we have a command and run the specified function.
  case "${1-}" in
  add)
    shift
    add "$@"
    ;;
  remove)
    shift
    remove "$@"
    ;;
  pull)
    shift
    pull "$@"
    ;;
  push)
    shift
    push "$@"
    ;;
  remote-add)
    shift
    remote_add "$@"
    ;;
  remote-remove)
    shift
    remote_remove "$@"
    ;;
  remote-rename)
    shift
    remote_rename "$@"
    ;;
  *)
    echo >&2 "Unknown command: ${1-}"
    usage
    return 1
    ;;
  esac
}
