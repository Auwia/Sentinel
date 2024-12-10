from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sentinel_base',
            executable='servo_controller',
            name='servo_controller'
        ),
        Node(
            package='sentinel_base',
            executable='pump_control',
            name='pump_control'
        ),
        Node(
            package='sentinel_base',
            executable='valve_control',
            name='valve_control'
        ),
    ])
