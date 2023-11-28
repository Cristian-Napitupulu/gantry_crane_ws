cmake_minimum_required(VERSION 3.8)
project(realsense_camera_yolo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# # Generate interface for realsense service-client
# find_package(sensor_msgs REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
# rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
#   "srv/RealsenseYOLO.srv"
#   DEPENDENCIES sensor_msgs
# )

# Create realsense_camera_node (cpp files)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(realsense_camera_yolo_interfaces REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)


include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  realsense2::realsense2
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${realsense_camera_yolo_interfaces_INCLUDE_DIRS}
)

add_executable(realsense_camera_node src/realsense_camera.cpp)

ament_target_dependencies(realsense_camera_node
  rclcpp
  sensor_msgs
  std_msgs
  realsense2
  OpenCV
  cv_bridge
  realsense_camera_yolo_interfaces
)

# link libraries for realsense_camera.cpp
target_link_libraries(
  realsense_camera_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  realsense2::realsense2
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
)

# Install cpp executables
install(TARGETS realsense_camera_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/realsense_yolo_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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

ament_package()
