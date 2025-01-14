#!/bin/bash
source /opt/ros/jazzy/setup.bash
exec "$@"
rm -rf build/ install/ log/
export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH
colcon build
