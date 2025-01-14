#!/bin/bash
exec "$@"
export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH
source install/setup.bash
ros2 service call /valve_control custom_interfaces/srv/ValveControl "{turn_on: false}"
