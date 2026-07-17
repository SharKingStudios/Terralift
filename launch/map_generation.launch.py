from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch.event_handlers import OnProcessExit
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

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

    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
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

    wait_for_scan = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            'echo "Waiting for /scan and /odom before configuring SLAM..."; '
            'until ros2 topic echo /scan --once >/dev/null 2>&1; do sleep 1; done; '
            'until ros2 topic echo /odom --once >/dev/null 2>&1; do sleep 1; done; '
            'sleep 1; '
            'echo "/scan and /odom are publishing; configuring SLAM."',
        ],
        output='screen',
    )

    configure_slam = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_scan,
            on_exit=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                )
            ],
        )
    )

    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
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
        wait_for_scan,
        configure_slam,
        activate_slam,
        rviz,
    ])
