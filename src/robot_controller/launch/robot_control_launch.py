import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('robot_controller'),
        'config', 'params.yaml'
    )

    print(f"Params file path: {params_file}")

    return LaunchDescription([
        Node(
            package='robot_controller',
            executable='servo_relay_control',
            name='servo_relay_control',
            output='screen',
            parameters=[params_file], 
        ),
        Node(
            package='robot_controller',
            executable='camera_processor',
            name='camera_processor',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='robot_controller',
            executable='calibration_gui',
            name='calibration_gui',
            output='screen',
            parameters=[params_file]
        )
    ])
