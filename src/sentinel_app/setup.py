from setuptools import setup

package_name = 'sentinel_app'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/app_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='auwia',
    maintainer_email='auwia@example.com',
    description='Application logic for Sentinel project',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_control = sentinel_app.main_control:main',
        ],
    },
)
