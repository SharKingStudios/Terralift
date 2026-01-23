from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'slam_mode': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'mode': 'mapping',
            'resolution': 0.05,
            'max_laser_range': 12.0,
            'minimum_time_interval': 0.2,
            'transform_publish_period': 0.05,
            'map_update_interval': 2.0,
            'use_pose_extrapolator': False
        }]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        slam_node,
        rviz
    ])
