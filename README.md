# Sentinel

Developed with ROS 2 - Jazzy Jalisco - https://docs.ros.org/en/jazzy/Installation.html

git clone git@github.com:Auwia/Sentinel.git

cd Sentinel

./scripts/setup_env.sh

./scripts/build.sh

./scripts/run.sh


###############################

Server:
ros2 launch robot_controller robot_control_launch.py

Client:
engine moves:
ros2 action send_goal /servo_control/set_servo_angle custom_interfaces/action/MoveMotors "{motor_id: 0, target_angle: 45.0, target_speed: 5.0, servo_type: 180}"

pump on:
ros2 service call /pump_control custom_interfaces/srv/PumpControl "{turn_on: true}"
pump off:
ros2 service call /pump_control custom_interfaces/srv/PumpControl "{turn_on: false}"

valve on:
ros2 service call /valve_control custom_interfaces/srv/ValveControl "{turn_on: true}"
valve off:
ros2 service call /valve_control custom_interfaces/srv/ValveControl "{turn_on: false}"

From Server Raspberry:
mosquitto_sub -h 192.168.0.133 -p 8883 -t "test/topic" --cafile ca.crt --cert client.crt --key client.key

From Client PC:
mosquitto_pub --cafile ca.crt --cert client.crt --key client.key -h 192.168.0.133 -p 8883 -t "test/topic" -m "Hello from PC!"
