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
  "msg/GoalPosition.msg"
  "msg/GoalCurrent.msg"
)

set(srv_files
  "srv/GetCurrent.srv"
  "srv/GetPosition.srv"
  "srv/GetPositionLimits.srv"
  "srv/SetPositionLimits.srv"
  "srv/GetTemperature.srv"
  "srv/GetTorqueEnabled.srv"
  "srv/SetOperatingMode.srv"
  "srv/SetTorqueEnabled.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime rclcpp std_msgs dynamixel_sdk)

add_executable(finger_driver src/finger_driver.cpp)
target_include_directories(finger_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(finger_driver
  finger_manipulation__rosidl_typesupport_cpp
)
ament_target_dependencies(finger_driver rclcpp std_msgs dynamixel_sdk rosidl_default_runtime)

install(TARGETS 
  finger_driver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
