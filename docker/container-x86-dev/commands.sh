#!/usr/bin/env bash

# Project-specific shell functions and commands.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# April 4, 2023

# Add yours, some convenient ones are provided below.
# You can also source other files from sub-units included by this project.

# Routine to convert an angle in degrees [-180° +180°] to radians [-PI +PI]
function degrad {
  if [[ $# -ne 1 ]] || [[ $1 -lt -180 ]] || [[ $1 -gt 180 ]]; then
    echo >&2 "Usage:"
    echo >&2 "    degrad ANGLE"
    echo >&2 "ANGLE must be in degrees and in [-180° +180°]"
    return 1
  fi
  local OP
  local FIRST
  local SECOND
  local RES
  OP="scale=6;$1*3.14159265359/180.0"
  RES="$(bc <<<"$OP")"
  FIRST="${RES:0:1}"
  SECOND="${RES:1:1}"
  if [[ $FIRST == "." ]]; then
    RES="0${RES}"
  fi
  if [[ $FIRST == "-" ]] && [[ $SECOND == "." ]]; then
    RES="-0.${RES:2:6}"
  fi
  echo "$RES"
}
