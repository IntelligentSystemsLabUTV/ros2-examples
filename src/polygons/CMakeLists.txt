cmake_minimum_required(VERSION 3.8)
project(polygons)

#! it's highly suggested to generate this with colcon by specifying --library-name

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(polygon_base REQUIRED)

# triangle library
add_library(triangle src/triangle.cpp)
target_compile_features(triangle PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(triangle PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  triangle
  "pluginlib"
  "polygon_base"
)

# square library
add_library(square src/square.cpp)
target_compile_features(square PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(square PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  square
  "pluginlib"
  "polygon_base"
)

#! hidden visibility must be the default when building DSOs
set_target_properties(
  triangle
  square
  PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
)

# link plugins with base class for pluginlib
pluginlib_export_plugin_description_file(polygon_base plugins.xml)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(triangle PRIVATE "POLYGONS_BUILDING_LIBRARY")
target_compile_definitions(square PRIVATE "POLYGONS_BUILDING_LIBRARY")

# install library headers
install(
  DIRECTORY include/
  DESTINATION include
)

# install library binaries
install(
  TARGETS triangle
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS square
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

#! export headers and binaries for ament
ament_export_include_directories(
  include
)
ament_export_libraries(
  triangle
  square
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
