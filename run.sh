#!/bin/bash
source install/setup.bash
exec "$@"

rm -rf build/ install/ log/
export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH
colcon build --symlink-install
source install/setup.bash

ros2 launch robot_controller robot_control_launch.py
