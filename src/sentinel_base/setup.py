from setuptools import setup

package_name = 'sentinel_base'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/base_launch.py']),
        ('share/' + package_name + '/config', ['config/pca9685_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='auwia',
    maintainer_email='massimo.manganiello@gmail.com',
    description='Hardware control for Sentinel project',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_controller = sentinel_base.servo_controller:main',
            'pump_control = sentinel_base.pump_control:main',
            'valve_control = sentinel_base.valve_control:main',
        ],
    },
)
