from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Run SLAM Toolbox in a headless-safe way.

    On the robot (Ubuntu Server), keep `use_rviz:=false`.
    On your laptop, set `use_rviz:=true` if you want this launch to open RViz.
    """

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (Gazebo) if true'
    )

    use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Start RViz2 (set true on laptop, false on robot)'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),

            # Frames
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',

            # Sensor
            'scan_topic': '/scan',

            # Mapping mode
            'mode': 'mapping',

            # Practical defaults for a small indoor robot
            'resolution': 0.05,
            'max_laser_range': 12.0,
            'minimum_time_interval': 0.2,
            'transform_publish_period': 0.05,
            'map_update_interval': 2.0,
            'use_pose_extrapolator': False,
            'scan_queue_size': 50,
        }]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        use_sim_time,
        use_rviz,
        slam_node,
        rviz,
    ])
