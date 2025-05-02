# cmake_minimum_required(VERSION 3.8)
# project(cpp_camera_reader)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# # find_package(gstreamer-1.0 REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(image_transport REQUIRED)
# find_package(image_transport_plugins REQUIRED)
# find_package(type_description_interfaces REQUIRED)
# find_package(OpenCV REQUIRED)
# find_package(cv_bridge REQUIRED)

# find_package(PkgConfig REQUIRED)
# pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
# pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

# # include_directories(
# #   include
# #   ${rclcpp_INCLUDE_DIRS}
# #   ${sensor_msgs_INCLUDE_DIRS}
# #   ${image_transport_INCLUDE_DIRS}
# #   ${image_transport_plugins_INCLUDE_DIRS}
# #   ${type_description_interfaces_INCLUDE_DIRS}
# #   ${GSTREAMER_INCLUDE_DIRS}
# #   ${GSTREAMER_APP_INCLUDE_DIRS}
# # )

# # add_executable(camera_node src/camera_node.cpp)
# # target_include_directories(camera_node PUBLIC
# #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# #   $<INSTALL_INTERFACE:include>)
# # target_link_libraries(camera_node
# #   ${GSTREAMER_LIBRARIES}
# #   ${GSTREAMER_APP_LIBRARIES}
# #   rclcpp::rclcpp
# #   sensor_msgs::sensor_msgs
# #   image_transport::image_transport
# #   rclcpp_components::component
# #   type_description_interfaces::type_description_interfaces
# # )

# # install(TARGETS camera_node
# #   DESTINATION lib/${PROJECT_NAME})

# # install(DIRECTORY include/
# #   DESTINATION include)

# # ament_export_include_directories(include)
# # ament_export_libraries(camera_node)
# # ament_export_dependencies(rclcpp sensor_msgs image_transport)
# add_executable(camera_node src/camera_node.cpp)

# # Настройка включения директорий
# target_include_directories(camera_node PUBLIC
#   include
#   ${GSTREAMER_INCLUDE_DIRS}
#   ${GSTREAMER_APP_INCLUDE_DIRS}
# )

# # Правильная линковка с использованием ament_target_dependencies
# ament_target_dependencies(camera_node
#   rclcpp
#   sensor_msgs
#   image_transport
#   cv_bridge
# )

# # Дополнительные библиотеки
# target_link_libraries(camera_node
#   ${GSTREAMER_LIBRARIES}
#   ${GSTREAMER_APP_LIBRARIES}
# )

# install(TARGETS
#   camera_node
#   DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()


# # Поиск GStreamer


# -----------------------------

cmake_minimum_required(VERSION 3.5)
project(cpp_camera_reader)

# Find ament dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Find OpenCV (используем современный способ)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# Find GStreamer using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

# Add executable
add_executable(camera_node src/camera_node.cpp)

# Include directories
target_include_directories(camera_node PRIVATE
  ${GSTREAMER_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(camera_node
  ${GSTREAMER_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Use ament_target_dependencies for ROS2 packages
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

# Install target
install(TARGETS camera_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()



