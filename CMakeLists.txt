cmake_minimum_required(VERSION 3.8)
project(finger_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(rosidl_default_generators REQUIRED)  
find_package(rosidl_default_runtime REQUIRED)

set(msg_files
  "msg/SetPosition.msg"
)

set(srv_files
  "srv/GetPosition.srv"
  "srv/GetTemperature.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime rclcpp std_msgs dynamixel_sdk)

add_executable(read_write_node src/read_write_node.cpp)
target_link_libraries(read_write_node
  finger_manipulation__rosidl_typesupport_cpp
)
ament_target_dependencies(read_write_node rclcpp std_msgs dynamixel_sdk rosidl_default_runtime)

install(TARGETS 
  read_write_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
