#!/bin/bash
exec "$@"
export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH
source install/setup.bash
ros2 action send_goal /servo_control/set_servo_angle custom_interfaces/action/MoveMotors "{motor_id: 1, target_angle: 53.0, target_speed: 5.0, servo_type: 180}"
