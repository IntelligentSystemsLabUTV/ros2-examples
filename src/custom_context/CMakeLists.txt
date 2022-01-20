cmake_minimum_required(VERSION 3.8)
project(custom_context)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# custom context example
add_executable(custom_context src/custom_context.cpp)
target_include_directories(custom_context PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(custom_context PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  custom_context
  "rclcpp"
)
install(TARGETS custom_context
  DESTINATION lib/${PROJECT_NAME})

# signal handling example
add_executable(termination src/termination.cpp)
target_include_directories(termination PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(termination PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  termination
  "rclcpp"
)
install(TARGETS termination
  DESTINATION lib/${PROJECT_NAME})

# launch files
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
