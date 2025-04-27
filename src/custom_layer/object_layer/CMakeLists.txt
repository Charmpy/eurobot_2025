cmake_minimum_required(VERSION 3.8)
project(object_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find package dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Set dependency lists for library and executable targets.
set(lib_dep_pkgs
  rclcpp
  nav2_costmap_2d
  pluginlib
  tf2
  tf2_geometry_msgs
)

set(sim_dep_pkgs
  rclcpp
  geometry_msgs
  tf2
)

include_directories(/opt/ros/jazzy/include/tf2_geometry_msgs)

# Create executable (for simulation or testing)
add_executable(ObjectSim 
  src/ObjectSim.cpp
)

# Create shared library for the costmap plugin.
add_library(object_layer SHARED
  plugin/object_layer.cpp
)

# Specify include directories.
target_include_directories(object_layer PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(ObjectSim PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link dependencies via ament.
ament_target_dependencies(object_layer ${lib_dep_pkgs})
ament_target_dependencies(ObjectSim ${sim_dep_pkgs})

# Optionally, set properties to control the runtime search path.
set_target_properties(object_layer PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "$ORIGIN/../../nav2_costmap_2d/lib;$ORIGIN/../../install/nav2_msgs/lib;/opt/ros/humble/lib"
)

# Export the plugin description file for pluginlib.
pluginlib_export_plugin_description_file(nav2_costmap_2d object_layer.xml)

# Install targets.
install(TARGETS 
  object_layer
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ObjectSim
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright/cpplint checks if desired.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
