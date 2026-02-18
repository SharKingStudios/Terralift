import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('terralift')

    # ----------- Args -----------
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    autostart = DeclareLaunchArgument('autostart', default_value='true')

    # Select one of the 4 experiment configurations
    nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value='rpp_smac2d.yaml',
        description='Nav2 params YAML in terralift/nav2/ (e.g., rpp_smac2d.yaml)'
    )

    # Enable SLAM Toolbox (recommended for your no-encoder setup)
    use_slam = DeclareLaunchArgument('use_slam', default_value='true')

    # TF offsets (edit these once, then leave them stable for all trials)
    laser_x = DeclareLaunchArgument('laser_x', default_value='0.0')
    laser_y = DeclareLaunchArgument('laser_y', default_value='0.0')
    laser_z = DeclareLaunchArgument('laser_z', default_value='0.2')
    laser_roll = DeclareLaunchArgument('laser_roll', default_value='0.0')
    laser_pitch = DeclareLaunchArgument('laser_pitch', default_value='0.0')
    laser_yaw = DeclareLaunchArgument('laser_yaw', default_value='0.0')

    imu_x = DeclareLaunchArgument('imu_x', default_value='0.0')
    imu_y = DeclareLaunchArgument('imu_y', default_value='0.0')
    imu_z = DeclareLaunchArgument('imu_z', default_value='0.0')
    imu_roll = DeclareLaunchArgument('imu_roll', default_value='0.0')
    imu_pitch = DeclareLaunchArgument('imu_pitch', default_value='0.0')
    imu_yaw = DeclareLaunchArgument('imu_yaw', default_value='0.0')

    # Cmd scaling for nav2 -> drivetrain
    max_vx = DeclareLaunchArgument('max_vx_mps', default_value='0.6')
    max_vy = DeclareLaunchArgument('max_vy_mps', default_value='0.6')
    max_wz = DeclareLaunchArgument('max_wz_rps', default_value='1.8')

    # ----------- Sensors & Base Control -----------
    imu = Node(
        package='terralift',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'imu_config.yaml')],
    )

    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'rplidar.launch.py')),
    )

    drivetrain = Node(
        package='terralift',
        executable='drivetrain_node',
        name='drivetrain_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'drivetrain_config.yaml')],
    )

    # Static TF: base_link -> laser and base_link -> imu_link
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            LaunchConfiguration('laser_x'),
            LaunchConfiguration('laser_y'),
            LaunchConfiguration('laser_z'),
            LaunchConfiguration('laser_roll'),
            LaunchConfiguration('laser_pitch'),
            LaunchConfiguration('laser_yaw'),
            'base_link',
            'laser',
        ],
    )

    base_to_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        arguments=[
            LaunchConfiguration('imu_x'),
            LaunchConfiguration('imu_y'),
            LaunchConfiguration('imu_z'),
            LaunchConfiguration('imu_roll'),
            LaunchConfiguration('imu_pitch'),
            LaunchConfiguration('imu_yaw'),
            'base_link',
            'imu_link',
        ],
    )

    # Open-loop odometry for SLAM/Nav2 (no wheel encoders)
    open_loop_odom = Node(
        package='terralift',
        executable='open_loop_odom',
        name='open_loop_odom',
        output='screen',
        parameters=[{
            'cmd_vel_topic': '/cmd_vel',
            'imu_topic': '/imu/data',
            'odom_topic': '/odom',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': True,
            'use_imu_yaw': True,
            'rate_hz': 50.0,
        }],
    )

    # Nav2 cmd_vel (m/s) -> drivetrain cmd_mecanum (-1..1)
    cmd_adapter = Node(
        package='terralift',
        executable='cmd_vel_to_mecanum',
        name='cmd_vel_to_mecanum',
        output='screen',
        parameters=[{
            'input_topic': '/cmd_vel',
            'output_topic': '/cmd_mecanum',
            'max_vx_mps': LaunchConfiguration('max_vx_mps'),
            'max_vy_mps': LaunchConfiguration('max_vy_mps'),
            'max_wz_rps': LaunchConfiguration('max_wz_rps'),
        }],
    )

    # ----------- SLAM Toolbox (optional) -----------
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'map_generation.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # robot is headless; run RViz on your laptop
            'use_rviz': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_slam')),
    )

    # ----------- Nav2 (navigation only; SLAM provides /map + map->odom) -----------
    nav2_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'params_file': os.path.join(pkg_share, 'nav2', LaunchConfiguration('nav2_params')),

            # We run SLAM Toolbox ourselves; don't start map_server/amcl inside Nav2.
            'slam': 'False',
            'use_localization': 'False',
            'map': '',
        }.items(),
    )

    return LaunchDescription([
        use_sim_time,
        autostart,
        nav2_params,
        use_slam,
        laser_x, laser_y, laser_z, laser_roll, laser_pitch, laser_yaw,
        imu_x, imu_y, imu_z, imu_roll, imu_pitch, imu_yaw,
        max_vx, max_vy, max_wz,

        imu,
        rplidar,
        drivetrain,
        base_to_laser_tf,
        base_to_imu_tf,
        open_loop_odom,
        cmd_adapter,
        slam,
        nav2_launch,
    ])
