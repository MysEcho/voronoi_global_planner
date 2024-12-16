cmake_minimum_required(VERSION 3.8)
project(voronoi_global_planner)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(costmap_2d REQUIRED)

# Add library
add_library(voronoi_global_planner SHARED
  src/voronoi_global_planner.cpp
  src/voronoi_diagram.cpp
)

# Include directories
target_include_directories(voronoi_global_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(voronoi_global_planner
  rclcpp
  nav2_core
  nav_msgs
  pluginlib
  geometry_msgs
  tf2_ros
)

# Export the plugin
pluginlib_export_plugin_description_file(nav2_core voronoi_global_planner_plugin.xml)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install library
install(
  TARGETS voronoi_global_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(voronoi_global_planner)
ament_export_dependencies(
  rclcpp
  nav2_core
  nav_msgs
  pluginlib
  geometry_msgs
  tf2_ros
)

ament_package()