from setuptools import setup

package_name = 'sentinel_app'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': 'sentinel_app'},  # Indica la directory contenente i file Python
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/app_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='auwia',
    maintainer_email='massimo.manganiello@gmail.com',
    description='Application logic for Sentinel project',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'motor_action_server = sentinel_app.motor_action_server:main',
            'motor_action_client = sentinel_app.motor_action_client:main',
        ],
    },
)
