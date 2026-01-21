from setuptools import find_packages, setup

package_name = 'terralift'

setup(
    name=package_name,
    version='0.0.1',
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
    description='Core ROS 2 package for the Terralift robot, containing all robot control, communication, and behavior code.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_test_node = terralift.motor_test_node:main',
            'motor_test_keyboard = terralift.motor_test_keyboard:main',
        ],
    },
)
