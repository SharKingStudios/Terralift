from setuptools import find_packages, setup

package_name = 'motor_teleop_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/fl_only.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Logan Peterson',
    maintainer_email='brushfire257@gmail.com',
    description='Robot motor slow-speed test (RPi.GPIO via rpi-lgpio) + keyboard teleop',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_test_node = motor_teleop_demo.motor_test_node:main',
            'motor_test_keyboard = motor_teleop_demo.motor_test_keyboard:main',
        ],
    },
)
