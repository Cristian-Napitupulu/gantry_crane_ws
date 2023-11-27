cmake_minimum_required(VERSION 3.8)
project(realsense_camera_yolo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(realsense_camera_yolo_interfaces REQUIRED)

add_library(realsense_camera SHARED src/realsense_camera.cpp)

add_executable(realsense_camera_node src/realsense_camera.cpp)

ament_target_dependencies(realsense_camera
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  realsense2
  OpenCV
  cv_bridge
  realsense_camera_yolo_interfaces
)

# No need to manually include directories for cv_bridge
ament_target_dependencies(realsense_camera_node
  rclcpp
  rclcpp_action
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
  ${rclcpp_action_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${realsense_LIBRARIES}
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
)

install(TARGETS realsense_camera realsense_camera_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
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
