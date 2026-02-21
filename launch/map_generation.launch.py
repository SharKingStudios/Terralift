from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


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

    pkg_share = get_package_share_directory('terralift')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_mapping.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {
                # Launch-controlled
                'use_sim_time': LaunchConfiguration('use_sim_time'),

                # Hard overrides to keep frames consistent
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
            },
        ]
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
