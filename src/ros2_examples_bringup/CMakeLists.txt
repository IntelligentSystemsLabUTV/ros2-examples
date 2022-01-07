cmake_minimum_required(VERSION 3.8)
project(ros2_examples_bringup)

# find dependencies
find_package(ament_cmake REQUIRED)

# install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
