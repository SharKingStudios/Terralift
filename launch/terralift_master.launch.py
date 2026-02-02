from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ----------------------------
    # Launch arguments
    # ----------------------------
    nav2_config = DeclareLaunchArgument(
        'nav2_config',
        default_value='rpp_smac2d.yaml',
        description='Nav2 parameter file'
    )

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    # ----------------------------
    # Paths
    # ----------------------------
    pkg_share = get_package_share_directory('terralift')

    ekf_config = os.path.join(pkg_share, 'config', 'ekf_imu.yaml')

    # IMPORTANT:
    # LaunchConfiguration is a substitution, so you cannot use os.path.join with it.
    nav2_params = PathJoinSubstitution([
        FindPackageShare('terralift'),
        'nav2',
        LaunchConfiguration('nav2_config'),
    ])

    # ----------------------------
    # Core nodes (always on)
    # ----------------------------
    master_node = Node(
        package='terralift',
        executable='master_node',
        name='master_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    led_node = Node(
        package='terralift',
        executable='led_node',
        name='led_node',
        output='screen'
    )

    # ----------------------------
    # Actuators
    # ----------------------------
    drivetrain_node = Node(
        package='terralift',
        executable='drivetrain_node',
        name='drivetrain',
        output='screen'
    )

    lift_arm_node = Node(
        package='terralift',
        executable='lift_arm_node',
        name='lift_arm',
        output='screen'
    )

    # ----------------------------
    # Sensors
    # ----------------------------
    imu_node = Node(
        package='terralift',
        executable='imu_node',
        name='imu',
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rplidar.launch.py')
        )
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'arducam.launch.py')
        )
    )

    # ----------------------------
    # Nav2 bringup (master controlled)
    # ----------------------------
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params,
            'autostart': 'false'
        }.items()
    )

    # ----------------------------
    # Launch everything
    # ----------------------------
    return LaunchDescription([
        nav2_config,
        use_sim_time,

        master_node,
        led_node,

        drivetrain_node,
        lift_arm_node,

        imu_node,
        ekf_node,
        lidar_launch,
        camera_launch,

        nav2_bringup
    ])
