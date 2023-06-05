cmake_minimum_required(VERSION 3.8)
project(ros2_usb_camera)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(CUDA)
find_package(image_transport REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)

# USB Camera Driver node
add_library(usb_camera_driver SHARED
  src/usb_camera_driver/ucd_utils.cpp
  src/usb_camera_driver/usb_camera_driver.cpp)
target_compile_definitions(usb_camera_driver PRIVATE COMPOSITION_BUILDING_DLL)
if (CUDA_FOUND AND OpenCV_CUDA_VERSION)
  message(STATUS "OpenCV was built with CUDA version: ${OpenCV_CUDA_VERSION}")
  message(STATUS "Compiling code with OpenCV GPU API")
  target_compile_definitions(usb_camera_driver PUBLIC WITH_CUDA)
endif()
target_include_directories(usb_camera_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(usb_camera_driver Threads::Threads)
ament_target_dependencies(
  usb_camera_driver
  "camera_calibration_parsers"
  "camera_info_manager"
  "image_transport"
  "OpenCV"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "std_msgs"
  "std_srvs"
)
rclcpp_components_register_nodes(usb_camera_driver "USBCameraDriver::CameraDriverNode")

# USB Camera standalone application
add_executable(usb_camera_app
  src/ucd_app.cpp)
target_include_directories(usb_camera_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(usb_camera_app usb_camera_driver)
ament_target_dependencies(
  usb_camera_app
  "rclcpp")

# usb_camera_driver component
install(TARGETS usb_camera_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# usb_camera app
install(TARGETS usb_camera_app
  DESTINATION lib/${PROJECT_NAME})

# Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Config files
install(DIRECTORY config
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

# Make sure that the library path is exported even if the library itself is not
ament_environment_hooks("${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")

ament_package()
