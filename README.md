# Sentinel

ROS 2 - Jazzy Jalisco - https://docs.ros.org/en/jazzy/Installation.html

You will need to run this command on every new shell you open to have access to the ROS 2 commands, like so:

source /opt/ros/jazzy/setup.bash

start turtlesim
export LIBGL_ALWAYS_SOFTWARE=1
ros2 run turtlesim turtlesim_node

Use turtlesim
ros2 run turtlesim turtle_teleop_key

RQT
rqt or rqt --force-discover // in case of issue
rqt_graph

ROS node list:
ros2 node list

ROS topic list:
ros2 topic list 

From Server Raspberry:
mosquitto_sub -h 192.168.0.133 -p 8883 -t "test/topic" --cafile ca.crt --cert client.crt --key client.key

From Client PC:
mosquitto_pub --cafile ca.crt --cert client.crt --key client.key -h 192.168.0.133 -p 8883 -t "test/topic" -m "Hello from PC!"
