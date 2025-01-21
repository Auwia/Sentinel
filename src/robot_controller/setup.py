from setuptools import setup

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_control_launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
        ('share/ament_index/resource_index/packages', ['resource/robot_controller']),  # Marker corretto
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='auwia',
    maintainer_email='massimo.manganiello@gmail.com',
    description='Robot Controller Package',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'servo_control_service = robot_controller.servo_control_service:main',
            'camera_processor = robot_controller.camera_processor:main',
            'pump_control = robot_controller.pump_control:main',
            'valve_control = robot_controller.valve_control:main',
            'cruscotto = robot_controller.cruscotto:main',
        ],
    },
)
