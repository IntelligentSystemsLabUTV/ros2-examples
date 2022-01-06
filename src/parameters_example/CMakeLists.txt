cmake_minimum_required(VERSION 3.8)
project(parameters_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(parametric_pub src/parametric_pub.cpp src/parametric_node.cpp)

target_include_directories(parametric_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(parametric_pub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  parametric_pub
  "rclcpp"
  "std_msgs"
)

# Executable
install(TARGETS parametric_pub
  DESTINATION lib/${PROJECT_NAME})

# Config file
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Launch file
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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
