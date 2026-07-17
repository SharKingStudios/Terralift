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

    odom_cmd_topic = DeclareLaunchArgument(
        'odom_cmd_topic',
        default_value='/cmd_vel',
        description='Twist topic in m/s used as the open-loop odom motion prior'
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
    robot_master_node = Node(
        package='terralift',
        executable='robot_master_node',
        name='robot_master_node',
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
        output='screen',
        parameters=[{
            'cmd_topic': '/cmd_mecanum',
            'odom_topic': '/odom',
            'max_vx_mps': 0.6,
            'max_vy_mps': 0.6,
            'max_wz_rps': 1.5,
        }]
    )

    lift_arm_node = Node(
        package='terralift',
        executable='lift_arm_node',
        name='lift_arm',
        output='screen'
    )


    cmd_adapter = Node(
        package='terralift',
        executable='cmd_vel_to_mecanum',
        name='cmd_vel_to_mecanum',
        output='screen',
        parameters=[{
            'input_topic': '/cmd_vel',
            'output_topic': '/cmd_mecanum',
            'max_vx_mps': 0.6,
            'max_vy_mps': 0.6,
            'max_wz_rps': 1.5,
        }]
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

    open_loop_odom = Node(
        package='terralift',
        executable='open_loop_odom',
        name='open_loop_odom',
        output='screen',
        parameters=[{
            'imu_topic': '/imu/data',
            'cmd_vel_topic': LaunchConfiguration('odom_cmd_topic'),
            'odom_topic': '/open_loop_odom',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'rate_hz': 50.0,
            'publish_tf': False,
            'cmd_timeout': 0.35,
            'vel_response_tau': 0.18,
            'idle_velocity_decay': 4.0,
            'max_vx_mps': 0.6,
            'max_vy_mps': 0.6,
            'max_wz_rps': 1.5,
            'use_imu_orientation': True,
            'use_imu_gyro': True,
            'imu_yaw_blend_gain': 2.0,
            'imu_wz_lpf_gain': 12.0,
            'require_imu_yaw': False,
        }]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', '/odom')]
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
        odom_cmd_topic,

        robot_master_node,
        led_node,

        drivetrain_node,
        lift_arm_node,
        cmd_adapter,

        imu_node,
        open_loop_odom,
        ekf_node,
        lidar_launch,
        camera_launch,

        nav2_bringup
    ])
