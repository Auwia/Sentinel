#!/bin/bash
exec "$@"
export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH
source install/setup.bash
ros2 service call /pump_control custom_interfaces/srv/PumpControl "{turn_on: false}"
