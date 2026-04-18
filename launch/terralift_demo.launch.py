import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('terralift')

    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    autostart = DeclareLaunchArgument('autostart', default_value='true')
    use_slam = DeclareLaunchArgument('use_slam', default_value='true')
    use_apriltags = DeclareLaunchArgument('use_apriltags', default_value='true')
    nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value='rpp_smac2d.yaml',
        description='Nav2 params YAML in terralift/nav2/'
    )

    joy_topic = DeclareLaunchArgument('joy_topic', default_value='/joy')
    nav_cmd_topic = DeclareLaunchArgument('nav_cmd_topic', default_value='/cmd_vel_nav_safe')
    teleop_cmd_topic = DeclareLaunchArgument('teleop_cmd_topic', default_value='/cmd_vel_teleop')
    drive_cmd_topic = DeclareLaunchArgument('drive_cmd_topic', default_value='/cmd_vel_demo_drive')
    odom_vel_response_tau = DeclareLaunchArgument('odom_vel_response_tau', default_value='0.0')

    camera_device = DeclareLaunchArgument('camera_device', default_value='/dev/video0')
    camera_info_url = DeclareLaunchArgument(
        'camera_info_url',
        default_value='package://terralift/config/ost.yaml'
    )

    laser_x = DeclareLaunchArgument('laser_x', default_value='0.0')
    laser_y = DeclareLaunchArgument('laser_y', default_value='0.0')
    laser_z = DeclareLaunchArgument('laser_z', default_value='0.09')
    laser_roll = DeclareLaunchArgument('laser_roll', default_value='0.0')
    laser_pitch = DeclareLaunchArgument('laser_pitch', default_value='0.0')
    laser_yaw = DeclareLaunchArgument('laser_yaw', default_value='3.141592653589793')

    imu_x = DeclareLaunchArgument('imu_x', default_value='0.0')
    imu_y = DeclareLaunchArgument('imu_y', default_value='0.0')
    imu_z = DeclareLaunchArgument('imu_z', default_value='0.0')
    imu_roll = DeclareLaunchArgument('imu_roll', default_value='0.0')
    imu_pitch = DeclareLaunchArgument('imu_pitch', default_value='0.0')
    imu_yaw = DeclareLaunchArgument('imu_yaw', default_value='0.0')

    max_vx = DeclareLaunchArgument('max_vx_mps', default_value='1.22')
    max_vy = DeclareLaunchArgument('max_vy_mps', default_value='1.22')
    max_wz = DeclareLaunchArgument('max_wz_rps', default_value='3.14')

    cam_x = DeclareLaunchArgument('cam_x', default_value='-0.16')
    cam_y = DeclareLaunchArgument('cam_y', default_value='0.0')
    cam_z = DeclareLaunchArgument('cam_z', default_value='0.025')
    cam_roll = DeclareLaunchArgument('cam_roll', default_value='3.141592653589793')
    cam_pitch = DeclareLaunchArgument('cam_pitch', default_value='-0.26179939')
    cam_yaw = DeclareLaunchArgument('cam_yaw', default_value='3.141592653589793')

    apriltag_params = DeclareLaunchArgument(
        'apriltag_params',
        default_value=PathJoinSubstitution([FindPackageShare('terralift'), 'config', 'apriltag.yaml'])
    )
    tag_map_file = DeclareLaunchArgument(
        'tag_map_file',
        default_value=PathJoinSubstitution([FindPackageShare('terralift'), 'config', 'tag_map.yaml'])
    )

    home_x = DeclareLaunchArgument('home_x', default_value='0.0')
    home_y = DeclareLaunchArgument('home_y', default_value='0.0')
    home_yaw = DeclareLaunchArgument('home_yaw', default_value='0.0')
    pov_up_x = DeclareLaunchArgument('pov_up_x', default_value='1.0')
    pov_up_y = DeclareLaunchArgument('pov_up_y', default_value='0.0')
    pov_up_yaw = DeclareLaunchArgument('pov_up_yaw', default_value='0.0')
    pov_left_x = DeclareLaunchArgument('pov_left_x', default_value='0.0')
    pov_left_y = DeclareLaunchArgument('pov_left_y', default_value='1.0')
    pov_left_yaw = DeclareLaunchArgument('pov_left_yaw', default_value='0.0')

    research_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'terralift_research.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'nav2_params': LaunchConfiguration('nav2_params'),
            'use_slam': LaunchConfiguration('use_slam'),
            'enable_trial_runner': 'false',
            'auto_close': 'false',
            'record_bag': 'false',
            'record_all_topics': 'false',
            'cmd_vel_input_topic': LaunchConfiguration('drive_cmd_topic'),
            'odom_cmd_topic': LaunchConfiguration('drive_cmd_topic'),
            'odom_vel_response_tau': LaunchConfiguration('odom_vel_response_tau'),
            'use_apriltags': LaunchConfiguration('use_apriltags'),
            'camera_device': LaunchConfiguration('camera_device'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'laser_x': LaunchConfiguration('laser_x'),
            'laser_y': LaunchConfiguration('laser_y'),
            'laser_z': LaunchConfiguration('laser_z'),
            'laser_roll': LaunchConfiguration('laser_roll'),
            'laser_pitch': LaunchConfiguration('laser_pitch'),
            'laser_yaw': LaunchConfiguration('laser_yaw'),
            'imu_x': LaunchConfiguration('imu_x'),
            'imu_y': LaunchConfiguration('imu_y'),
            'imu_z': LaunchConfiguration('imu_z'),
            'imu_roll': LaunchConfiguration('imu_roll'),
            'imu_pitch': LaunchConfiguration('imu_pitch'),
            'imu_yaw': LaunchConfiguration('imu_yaw'),
            'max_vx_mps': LaunchConfiguration('max_vx_mps'),
            'max_vy_mps': LaunchConfiguration('max_vy_mps'),
            'max_wz_rps': LaunchConfiguration('max_wz_rps'),
            'cam_x': LaunchConfiguration('cam_x'),
            'cam_y': LaunchConfiguration('cam_y'),
            'cam_z': LaunchConfiguration('cam_z'),
            'cam_roll': LaunchConfiguration('cam_roll'),
            'cam_pitch': LaunchConfiguration('cam_pitch'),
            'cam_yaw': LaunchConfiguration('cam_yaw'),
            'apriltag_params': LaunchConfiguration('apriltag_params'),
            'tag_map_file': LaunchConfiguration('tag_map_file'),
        }.items(),
    )

    lift_arm = Node(
        package='terralift',
        executable='lift_arm_node',
        name='lift_arm',
        output='screen',
    )

    demo_mode = Node(
        package='terralift',
        executable='demo_mode_node',
        name='demo_mode_node',
        output='screen',
        parameters=[{
            'joy_topic': LaunchConfiguration('joy_topic'),
            'nav_cmd_topic': LaunchConfiguration('nav_cmd_topic'),
            'home_x': LaunchConfiguration('home_x'),
            'home_y': LaunchConfiguration('home_y'),
            'home_yaw': LaunchConfiguration('home_yaw'),
            'pov_up_x': LaunchConfiguration('pov_up_x'),
            'pov_up_y': LaunchConfiguration('pov_up_y'),
            'pov_up_yaw': LaunchConfiguration('pov_up_yaw'),
            'pov_left_x': LaunchConfiguration('pov_left_x'),
            'pov_left_y': LaunchConfiguration('pov_left_y'),
            'pov_left_yaw': LaunchConfiguration('pov_left_yaw'),
        }],
    )

    cmd_arbiter = Node(
        package='terralift',
        executable='cmd_vel_arbiter',
        name='cmd_vel_arbiter',
        output='screen',
        parameters=[{
            'nav_topic': LaunchConfiguration('nav_cmd_topic'),
            'teleop_topic': LaunchConfiguration('teleop_cmd_topic'),
            'output_topic': LaunchConfiguration('drive_cmd_topic'),
            'teleop_msg_timeout_sec': 0.50,
            'teleop_override_timeout_sec': 0.45,
            'teleop_deadband': 0.01,
        }],
    )

    return LaunchDescription([
        use_sim_time,
        autostart,
        use_slam,
        use_apriltags,
        nav2_params,
        joy_topic,
        nav_cmd_topic,
        teleop_cmd_topic,
        drive_cmd_topic,
        odom_vel_response_tau,
        camera_device,
        camera_info_url,
        laser_x,
        laser_y,
        laser_z,
        laser_roll,
        laser_pitch,
        laser_yaw,
        imu_x,
        imu_y,
        imu_z,
        imu_roll,
        imu_pitch,
        imu_yaw,
        max_vx,
        max_vy,
        max_wz,
        cam_x,
        cam_y,
        cam_z,
        cam_roll,
        cam_pitch,
        cam_yaw,
        apriltag_params,
        tag_map_file,
        home_x,
        home_y,
        home_yaw,
        pov_up_x,
        pov_up_y,
        pov_up_yaw,
        pov_left_x,
        pov_left_y,
        pov_left_yaw,
        research_stack,
        lift_arm,
        demo_mode,
        cmd_arbiter,
    ])
