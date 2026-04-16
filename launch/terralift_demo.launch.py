import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('terralift')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf_imu.yaml')

    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    autostart = DeclareLaunchArgument('autostart', default_value='true')

    nav2_params = DeclareLaunchArgument(
        'nav2_params',
        default_value='rpp_smac2d.yaml',
        description='Nav2 params YAML in terralift/nav2/'
    )

    use_slam = DeclareLaunchArgument('use_slam', default_value='true')
    use_apriltags = DeclareLaunchArgument('use_apriltags', default_value='true')
    camera_device = DeclareLaunchArgument('camera_device', default_value='/dev/video0')
    camera_info_url = DeclareLaunchArgument(
        'camera_info_url',
        default_value='package://terralift/config/ost.yaml'
    )

    joy_topic = DeclareLaunchArgument('joy_topic', default_value='/joy')
    nav_cmd_topic = DeclareLaunchArgument('nav_cmd_topic', default_value='/cmd_vel_nav_safe')
    teleop_cmd_topic = DeclareLaunchArgument('teleop_cmd_topic', default_value='/cmd_vel_teleop')
    drive_cmd_topic = DeclareLaunchArgument('drive_cmd_topic', default_value='/cmd_vel_demo_drive')

    max_vx = DeclareLaunchArgument('max_vx_mps', default_value='1.22')
    max_vy = DeclareLaunchArgument('max_vy_mps', default_value='1.22')
    max_wz = DeclareLaunchArgument('max_wz_rps', default_value='3.14')

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

    imu = Node(
        package='terralift',
        executable='imu_node',
        name='imu_node',
        output='screen',
    )

    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'rplidar.launch.py')),
    )

    drivetrain = Node(
        package='terralift',
        executable='drivetrain_node',
        name='drivetrain_node',
        output='screen',
    )

    lift_arm = Node(
        package='terralift',
        executable='lift_arm_node',
        name='lift_arm',
        output='screen',
    )

    led_node = Node(
        package='terralift',
        executable='led_node',
        name='led_node',
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
        }],
    )

    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            LaunchConfiguration('laser_x'),
            LaunchConfiguration('laser_y'),
            LaunchConfiguration('laser_z'),
            LaunchConfiguration('laser_yaw'),
            LaunchConfiguration('laser_pitch'),
            LaunchConfiguration('laser_roll'),
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
            LaunchConfiguration('imu_yaw'),
            LaunchConfiguration('imu_pitch'),
            LaunchConfiguration('imu_roll'),
            'base_link',
            'imu_link',
        ],
    )

    base_to_camera_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_link_tf',
        arguments=[
            LaunchConfiguration('cam_x'),
            LaunchConfiguration('cam_y'),
            LaunchConfiguration('cam_z'),
            LaunchConfiguration('cam_yaw'),
            LaunchConfiguration('cam_pitch'),
            LaunchConfiguration('cam_roll'),
            'base_link',
            'camera_link',
        ],
        condition=IfCondition(LaunchConfiguration('use_apriltags')),
    )

    camera_link_to_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_optical_tf',
        arguments=[
            '0', '0', '0',
            '-1.57079632679', '0', '-1.57079632679',
            'camera_link',
            'camera_optical_frame',
        ],
        condition=IfCondition(LaunchConfiguration('use_apriltags')),
    )

    camera_optical_to_camera_alias_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_to_camera_alias_tf',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'camera_optical_frame',
            'camera',
        ],
        condition=IfCondition(LaunchConfiguration('use_apriltags')),
    )

    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('camera_device'),
            'pixel_format': 'YUYV',
            'output_encoding': 'mono8',
            'image_size': [640, 480],
            'frame_id': 'camera_optical_frame',
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'camera_name': 'camera',
        }],
        remappings=[('image_raw', 'image_mono')],
        condition=IfCondition(LaunchConfiguration('use_apriltags')),
    )

    apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        namespace='apriltag',
        output='screen',
        parameters=[LaunchConfiguration('apriltag_params')],
        remappings=[
            ('image_rect', '/camera/image_mono'),
            ('camera_info', '/camera/camera_info'),
        ],
        condition=IfCondition(LaunchConfiguration('use_apriltags')),
    )

    tag_snapper = Node(
        package='terralift',
        executable='tag_pose_to_odom_reset',
        name='tag_pose_to_odom_reset',
        output='screen',
        parameters=[{
            'tag_map_file': LaunchConfiguration('tag_map_file'),
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'cam_frame': 'camera_optical_frame',
            'tag_frame_prefix': 'tag_',
            'detections_topic': '/apriltag/detections',
            'reset_topic': '/tag_reset_pose',
            'min_decision_margin': 5.0,
            'min_interval_sec': 0.50,
        }],
        condition=IfCondition(LaunchConfiguration('use_apriltags')),
    )

    open_loop_odom = Node(
        package='terralift',
        executable='open_loop_odom',
        name='open_loop_odom',
        output='screen',
        parameters=[{
            'imu_topic': '/imu/data',
            'cmd_vel_topic': LaunchConfiguration('drive_cmd_topic'),
            'odom_topic': '/open_loop_odom',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'rate_hz': 50.0,
            'publish_tf': False,
            'reset_pose_topic': '/tag_reset_pose',
            'cmd_timeout': 0.35,
            'vel_response_tau': 0.18,
            'idle_velocity_decay': 4.0,
            'max_vx_mps': LaunchConfiguration('max_vx_mps'),
            'max_vy_mps': LaunchConfiguration('max_vy_mps'),
            'max_wz_rps': LaunchConfiguration('max_wz_rps'),
            'use_imu_orientation': True,
            'use_imu_gyro': True,
            'imu_yaw_blend_gain': 2.0,
            'imu_wz_lpf_gain': 12.0,
            'require_imu_yaw': False,
        }],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[('odometry/filtered', '/odom')],
    )

    cmd_adapter = Node(
        package='terralift',
        executable='cmd_vel_to_mecanum',
        name='cmd_vel_to_mecanum',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('drive_cmd_topic'),
            'output_topic': '/cmd_mecanum',
            'max_vx_mps': LaunchConfiguration('max_vx_mps'),
            'max_vy_mps': LaunchConfiguration('max_vy_mps'),
            'max_wz_rps': LaunchConfiguration('max_wz_rps'),
        }],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'map_generation.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_rviz': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_slam')),
    )

    slam_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='slam_lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'bond_timeout': 15.0,
            'node_names': ['slam_toolbox'],
        }],
        condition=IfCondition(LaunchConfiguration('use_slam')),
    )

    nav2_dir = get_package_share_directory('nav2_bringup')
    nav2_params_file = PathJoinSubstitution([
        FindPackageShare('terralift'),
        'nav2',
        LaunchConfiguration('nav2_params')
    ])

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'params_file': nav2_params_file,
            'slam': 'False',
            'use_localization': 'False',
            'map': '',
            'use_composition': 'False',
            'respawn': 'False',
            'rviz': 'False',
        }.items(),
    )

    return LaunchDescription([
        use_sim_time, autostart, nav2_params, use_slam,
        use_apriltags, camera_device, camera_info_url,
        joy_topic, nav_cmd_topic, teleop_cmd_topic, drive_cmd_topic,
        max_vx, max_vy, max_wz,
        laser_x, laser_y, laser_z, laser_roll, laser_pitch, laser_yaw,
        imu_x, imu_y, imu_z, imu_roll, imu_pitch, imu_yaw,
        cam_x, cam_y, cam_z, cam_roll, cam_pitch, cam_yaw,
        apriltag_params, tag_map_file,
        home_x, home_y, home_yaw,
        pov_up_x, pov_up_y, pov_up_yaw,
        pov_left_x, pov_left_y, pov_left_yaw,

        imu,
        rplidar,
        drivetrain,
        lift_arm,
        led_node,
        demo_mode,
        cmd_arbiter,
        base_to_laser_tf,
        base_to_imu_tf,
        base_to_camera_link_tf,
        camera_link_to_optical_tf,
        camera_optical_to_camera_alias_tf,
        camera,
        apriltag,
        tag_snapper,
        open_loop_odom,
        ekf_node,
        cmd_adapter,
        slam,
        slam_lifecycle_manager,
        nav2_launch,
    ])
