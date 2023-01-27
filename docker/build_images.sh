#!/bin/bash

# Automated images builder script.
#
# Roberto Masocco <robmasocco@gmail.com>
#
# April 23, 2022

# NOTE: Make sure that a Docker Hub PAT is enabled for this system!

# Check input arguments
TARGETS=()
BUILD_FLAGS=("--pull")
if [[ $# -eq 0 ]]; then
  echo >&2 "ERROR: No image specified"
  return 1
fi
for ARG in "$@"
do
  case $ARG in
    armv8)
      TARGETS+=("$ARG")
      ;;
    dev)
      TARGETS+=("$ARG")
      ;;
    nvidia)
      TARGETS+=("$ARG")
      ;;
    --*)
      BUILD_FLAGS+=("$ARG")
      ;;
    *)
      echo >&2 "ERROR: Invalid argument $ARG"
      return 1
      ;;
  esac
done
echo "Build options: " "${BUILD_FLAGS[@]}"

# Log in as ISL
docker login -u intelligentsystemslabutv

# Build requested images
for TARGET in "${TARGETS[@]}"; do
  echo "Building $TARGET image..."
  sleep 3
  docker-compose build "${BUILD_FLAGS[@]}" "$TARGET"
  echo "Pushing $TARGET image..."
  sleep 3
  docker-compose push "$TARGET"
done

echo "Done!"
