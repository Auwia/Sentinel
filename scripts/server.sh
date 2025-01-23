#!/bin/bash
source install/setup.bash
exec "$@"
export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH
ros2 launch robot_controller robot_control_launch.py
