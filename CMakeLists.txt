cmake_minimum_required(VERSION 3.8)
project(finger_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find required packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk_examples REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(rosidl_default_generators REQUIRED)  

################################################################################
# Declare ROS messages and services
################################################################################
set(msg_files
  "msg/SetPosition.msg"
  "msg/SyncSetPosition.msg"
  "msg/BulkSetItem.msg"
)

set(srv_files
  "srv/GetPosition.srv"
  "srv/SyncGetPosition.srv"
  "srv/BulkGetItem.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  include
)

add_executable(read_write_node src/read_write_node.cpp)
ament_target_dependencies(read_write_node rclcpp std_msgs dynamixel_sdk)

add_executable(sync_read_write_node src/sync_read_write_node.cpp)
ament_target_dependencies(sync_read_write_node rclcpp std_msgs dynamixel_sdk)

add_executable(bulk_read_write_node src/bulk_read_write_node.cpp)
ament_target_dependencies(bulk_read_write_node rclcpp std_msgs dynamixel_sdk)

add_executable(indirect_address_node src/indirect_address_node.cpp)
ament_target_dependencies(indirect_address_node rclcpp std_msgs dynamixel_sdk)

################################################################################
# Install Targets
################################################################################
install(TARGETS 
  read_write_node
  sync_read_write_node
  bulk_read_write_node
  indirect_address_node
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

################################################################################
# Package Export
################################################################################
ament_export_dependencies(rclcpp std_msgs dynamixel_sdk)
ament_package()
