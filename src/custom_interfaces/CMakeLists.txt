cmake_minimum_required(VERSION 3.5)
project(custom_interfaces)

find_package(rosidl_default_generators REQUIRED)

ament_package_xml()
message(STATUS "ament_package_xml() parsing completed")
message(STATUS "Groups: ${custom_interfaces_MEMBER_OF_GROUPS}")
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveMotors.action"
  DEPENDENCIES builtin_interfaces
)

ament_package()
