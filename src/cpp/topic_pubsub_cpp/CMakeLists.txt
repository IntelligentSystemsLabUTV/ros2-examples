cmake_minimum_required(VERSION 3.8)
project(topic_pubsub_cpp)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Publisher
add_executable(pub src/pub.cpp)
target_compile_features(pub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  pub
  rclcpp
  std_msgs)

# Subscriber
add_executable(sub src/sub.cpp)
target_compile_features(sub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(sub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  sub
  rclcpp
  std_msgs)

# Resetting subscriber
add_executable(resetting_sub src/resetting_sub.cpp)
target_compile_features(resetting_sub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(resetting_sub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  resetting_sub
  rclcpp
  std_msgs)

# Install executables
install(TARGETS pub
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS sub
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS resetting_sub
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
