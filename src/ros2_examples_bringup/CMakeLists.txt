cmake_minimum_required(VERSION 3.8)
project(ros2_examples_bringup)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
