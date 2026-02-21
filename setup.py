from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'terralift'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(exclude=['test']),

    # Install non-python resources
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        # Launch files
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),

        # Config files (EKF)
        ('share/' + package_name + '/config',
            glob('config/*.yaml')),

        # Nav2 parameter files
        ('share/' + package_name + '/nav2',
            glob('nav2/*.yaml')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='Logan Peterson',
    maintainer_email='brushfire257@gmail.com',

    description='Core ROS 2 package for the Terralift robot, containing all robot control, communication, and behavior code.',

    license='MIT',
    tests_require=['pytest'],

    # Executable nodes
    entry_points={
        'console_scripts': [

            # Existing test nodes
            'motor_test_node = terralift.motor_test_node:main',
            'motor_test_keyboard = terralift.motor_test_keyboard:main',

            # Drive & Actuators
            'drivetrain_node = terralift.drivetrain_node:main',
            'lift_arm_node = terralift.lift_arm_node:main',
            'led_node = terralift.led_node:main',

            # Sensors
            'imu_node = terralift.imu_node:main',

            # State & Control
            'robot_master_node = terralift.robot_master_node:main',
            'teleop_node = terralift.teleop_node:main',

            # Research / bringup helpers
            'open_loop_odom = terralift.open_loop_odom_node:main',
            'sensor_odom = terralift.sensor_odom_node:main',
            'cmd_vel_to_mecanum = terralift.cmd_vel_to_mecanum_node:main',
            'rosbag_node = terralift.rosbag_node:main',
        ],
    },
)
