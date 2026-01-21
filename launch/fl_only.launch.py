from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_teleop_demo',
            executable='motor_test_node',
            output='screen',
            parameters=[{
                'dir_pin': 16,      # BCM pins
                'pwm_pin': 14,
                'pwm_hz': 200,      # nice & gentle for testing
                'test_duty': 30.0,  # percent when commanded FWD/BWD
                'forward_high': True  # set False if your wiring inverts direction
            }]
        ),
    ])
