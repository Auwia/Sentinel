# Sentinel

Developed with ROS 2 - Jazzy Jalisco - https://docs.ros.org/en/jazzy/Installation.html

You will need to run this command on every new shell you open to have access to the ROS 2 commands, like so:

git clone git@github.com:Auwia/Sentinel.git

cd Sentinel

colcon build

source install/setup.bash

Server:
ros2 launch robot_controller robot_control_launch.py

Client:
ros2 action send_goal /servo_relay_control/set_servo_angle custom_interfaces/action/MoveMotors "{motor_id: 0, target_speed: 10.0, target_angle: 190.0, servo_type: 360}"

From Server Raspberry:
mosquitto_sub -h 192.168.0.133 -p 8883 -t "test/topic" --cafile ca.crt --cert client.crt --key client.key

From Client PC:
mosquitto_pub --cafile ca.crt --cert client.crt --key client.key -h 192.168.0.133 -p 8883 -t "test/topic" -m "Hello from PC!"
