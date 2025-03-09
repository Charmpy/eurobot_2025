cmake_minimum_required(VERSION 3.5)
project(custom_costmap_layer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

add_library(custom_costmap_layer SHARED
  src/custom_layer.cpp
)

ament_target_dependencies(custom_costmap_layer
  rclcpp
  nav2_costmap_2d
  pluginlib
)

install(TARGETS custom_costmap_layer
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(custom_costmap_layer)
ament_export_dependencies(rclcpp nav2_costmap_2d pluginlib)

pluginlib_export_plugin_description_file(nav2_costmap_2d custom_layer.xml)

ament_package()