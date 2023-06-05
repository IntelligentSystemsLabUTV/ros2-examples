cmake_minimum_required(VERSION 3.8)
project(aruco_detector)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(stanis_interfaces REQUIRED)
#find_package(stanis_qos REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)

# Aruco Detector standalone application
add_executable(aruco_detector
  src/aruco_detector/aruco_detector.cpp
  src/aruco_detector/ad_services.cpp
  src/aruco_detector/ad_subscriptions.cpp
  src/aruco_detector/ad_utils.cpp
  src/aruco_detector_app.cpp)
target_include_directories(aruco_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(
  aruco_detector
  ${OpenCV_LIBS}
  Threads::Threads)
ament_target_dependencies(
  aruco_detector
  "image_transport"
  "rclcpp"
  "sensor_msgs"
  #"stanis_interfaces"
  #"stanis_qos"
  "std_msgs"
  "std_srvs")

# Aruco Detector app
install(TARGETS aruco_detector
  DESTINATION lib/${PROJECT_NAME})

# Config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Launch files
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
