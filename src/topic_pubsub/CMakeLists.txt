cmake_minimum_required(VERSION 3.5)
project(topic_pubsub)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# publisher
add_executable(pub src/pub.cpp)
target_include_directories(pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  pub
  "rclcpp"
  "std_msgs"
)

# subscriber
add_executable(sub src/sub.cpp)
target_include_directories(sub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  sub
  "rclcpp"
  "std_msgs"
)

# periodic subscriber
add_executable(periodic_sub src/periodic_sub.cpp)
target_include_directories(periodic_sub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  periodic_sub
  "rclcpp"
  "std_msgs"
)

# intrapc test
add_executable(intrapc_test src/intrapc_test.cpp)
target_include_directories(intrapc_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  intrapc_test
  "rclcpp"
  "std_msgs"
)

# intrapc publisher
add_executable(intrapc_pub src/intrapc_pub.cpp)
target_include_directories(intrapc_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  intrapc_pub
  "rclcpp"
  "std_msgs"
)

# intrapc subscriber
add_executable(intrapc_sub src/intrapc_sub.cpp)
target_include_directories(intrapc_sub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  intrapc_sub
  "rclcpp"
  "std_msgs"
)

install(TARGETS pub
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS sub
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS periodic_sub
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS intrapc_test
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS intrapc_pub
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS intrapc_sub
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
