cmake_minimum_required(VERSION 3.8)
project(gripper_plugin_gz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-common6 REQUIRED)
find_package(gz-math8 REQUIRED)
find_package(gz-plugin3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(gripper_plugin_gz SHARED
  src/gripper_plugin_gz.cc
)

# Link libraries using ament_target_dependencies for ROS packages
ament_target_dependencies(gripper_plugin_gz
  rclcpp
  std_msgs
)

# Link libraries
target_link_libraries(gripper_plugin_gz
  gz-sim8::gz-sim8
  gz-common6::gz-common6
  gz-math8::gz-math8
  gz-plugin3::gz-plugin3
)

# Install the plugin
install(TARGETS gripper_plugin_gz
  EXPORT gripper_plugin_gzTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Export the targets for other packages to use
ament_export_targets(gripper_plugin_gzTargets HAS_LIBRARY_TARGET)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

