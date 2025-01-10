cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveMotors.action"
  "srv/PumpControl.srv"
  "srv/ValveControl.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
