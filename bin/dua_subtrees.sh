#!/usr/bin/env bash

# Subtrees management script for DUA.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# April 4, 2023

set -o errexit
set -o nounset
set -o pipefail
if [[ "${TRACE-0}" == "1" ]]; then set -o xtrace; fi

function usage {
  echo >&2 "Usage:"
  echo >&2 "    dua_subtrees.sh [--add|--remove|--pull|--push|--remote-add|--remote-remove] ARGS"
  echo >&2 "See the README for more info."
}

if [[ "${1-}" =~ ^-*h(elp)?$ ]]; then
  usage
  exit 1
fi

# Function to add a subtree.
function add {
  # Check input arguments
  if [[ $# -ne 3 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    dua_subtrees.sh --add REMOTE PREFIX BRANCH"
    echo >&2 "REMOTE may be an URL or a preexisting remote name."
    echo >&2 "PREFIX is the path to the subtree in the local repo."
    echo >&2 "BRANCH is the branch to pull from."
    exit 1
  fi

  # Sanity-check the prefix: it must not exist yet
  if [[ -d "${2}" ]]; then
    echo >&2 "ERROR: prefix path ${2} already exists"
    exit 1
  fi

  # Add the requested subtree
  git subtree add --prefix="${2}" "${1}" "${3}" --squash
}

# Function to remove a subtree.
function remove {
  # Check input arguments
  if [[ $# -ne 1 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    dua_subtrees.sh --remove PREFIX"
    echo >&2 "PREFIX is the path to the subtree in the local repo."
    exit 1
  fi

  # Check if the prefix exists
  if [[ ! -d "${1}" ]]; then
    echo >&2 "ERROR: prefix path ${1} does not exist"
    exit 1
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
    echo >&2 "    dua_subtrees.sh --pull REMOTE PREFIX BRANCH"
    echo >&2 "REMOTE may be an URL or a preexisting remote name."
    echo >&2 "PREFIX is the path to the subtree in the local repo."
    echo >&2 "BRANCH is the branch to pull from."
    exit 1
  fi

  # Check if the prefix exists
  if [[ ! -d "${2}" ]]; then
    echo >&2 "ERROR: prefix path ${2} does not exist"
    exit 1
  fi

  # Pull from the requested subtree
  git subtree pull --prefix="${2}" "${1}" "${3}" --squash
}

# Function to push to a subtree.
function push {
  # Check input arguments
  if [[ $# -ne 3 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    dua_subtrees.sh --push REMOTE PREFIX BRANCH"
    echo >&2 "REMOTE may be an URL or a preexisting remote name."
    echo >&2 "PREFIX is the path to the subtree in the local repo."
    echo >&2 "BRANCH is the branch to push to."
    exit 1
  fi

  # Check if the prefix exists
  if [[ ! -d "${2}" ]]; then
    echo >&2 "ERROR: prefix path ${2} does not exist"
    exit 1
  fi

  # Push to the requested subtree
  git subtree push --prefix="${2}" "${1}" "${3}"
}

# Function to add a remote to a subtree.
function remote_add {
  # Check input arguments
  if [[ $# -ne 2 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    dua_subtrees.sh --remote-add NAME URL"
    echo >&2 "NAME is the name of the remote to add."
    echo >&2 "URL is the URL of the remote to add."
    exit 1
  fi

  # Add the requested remote
  git remote add -f "${1}" "${2}"
}

# Function to remove a remote from a subtree.
function remote_remove {
  # Check input arguments
  if [[ $# -ne 1 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    dua_subtrees.sh --remote-remove NAME"
    echo >&2 "NAME is the name of the remote to remove."
    exit 1
  fi

  # Remove the requested remote
  git remote remove "${1}"
}

# Function to rename a remote.
function remote_rename {
  # Check input arguments
  if [[ $# -ne 2 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    dua_subtrees.sh --remote-rename OLD NEW"
    echo >&2 "OLD is the name of the remote to rename."
    echo >&2 "NEW is the new name of the remote."
    exit 1
  fi

  # Rename the requested remote
  git remote rename "${1}" "${2}"
}

# Check if we have a command and run the specified function.
case "${1-}" in
  --add)
    shift
    add "$@"
    ;;
  --remove)
    shift
    remove "$@"
    ;;
  --pull)
    shift
    pull "$@"
    ;;
  --push)
    shift
    push "$@"
    ;;
  --remote-add)
    shift
    remote_add "$@"
    ;;
  --remote-remove)
    shift
    remote_remove "$@"
    ;;
  --remote-rename)
    shift
    remote_rename "$@"
    ;;
  *)
    echo >&2 "Unknown command: ${1-}"
    usage
    exit 1
    ;;
esac
