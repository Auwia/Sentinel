from setuptools import setup, find_packages

package_name = 'sentinel_app'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=['sentinel_app', 'sentinel_app.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/app_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer_name',
    maintainer_email='maintainer_email@example.com',
    description='A brief description of the sentinel_app package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'motor_action_server = sentinel_app.motor_action_server:main',
            'motor_action_client = sentinel_app.motor_action_client:main',
        ],
    },
)
