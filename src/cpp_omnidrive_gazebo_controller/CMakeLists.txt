cmake_minimum_required(VERSION 3.8)
project(cpp_omnidrive_gazebo_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()




# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)


find_package(gz-transport13 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})
# find_package(gz REQUIRED)

add_executable(omni_gz_con src/cpp_omnidrive_gazebo_controller.cpp)
ament_target_dependencies(omni_gz_con rclcpp std_msgs geometry_msgs tf2 tf2_ros nav_msgs)
target_link_libraries(omni_gz_con gz-transport${GZ_TRANSPORT_VER}::core)

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

install(TARGETS
  omni_gz_con
  DESTINATION lib/${PROJECT_NAME})

ament_package()
