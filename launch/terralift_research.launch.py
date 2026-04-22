import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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
        default_value='rpp_smac2d_course_valid.yaml',
        description='Nav2 params YAML in terralift/nav2/'
    )

    use_slam = DeclareLaunchArgument('use_slam', default_value='true')
    enable_trial_runner = DeclareLaunchArgument('enable_trial_runner', default_value='true')
    auto_close = DeclareLaunchArgument('auto_close', default_value='true')
    record_bag = DeclareLaunchArgument('record_bag', default_value='true')
    record_all_topics = DeclareLaunchArgument('record_all_topics', default_value='false')
    bag_base_dir = DeclareLaunchArgument('bag_base_dir', default_value='~/terralift_bags')
    trial_prefix = DeclareLaunchArgument('trial_prefix', default_value='trial')
    environment_id = DeclareLaunchArgument('environment_id', default_value='default_env')
    trial_notes = DeclareLaunchArgument('trial_notes', default_value='')
    parameter_set_id = DeclareLaunchArgument('parameter_set_id', default_value='')
    nav2_version = DeclareLaunchArgument('nav2_version', default_value='unknown')
    robot_model = DeclareLaunchArgument('robot_model', default_value='terralift')
    robot_firmware_driver_versions = DeclareLaunchArgument('robot_firmware_driver_versions', default_value='unknown')
    startup_delay_sec = DeclareLaunchArgument('startup_delay_sec', default_value='4.0')
    goal_timeout_sec = DeclareLaunchArgument('goal_timeout_sec', default_value='90.0')
    goal_frame = DeclareLaunchArgument('goal_frame', default_value='map')
    goal_x = DeclareLaunchArgument('goal_x', default_value='2.5')
    goal_y = DeclareLaunchArgument('goal_y', default_value='3.4')
    goal_yaw = DeclareLaunchArgument('goal_yaw', default_value='0.0')
    cmd_vel_input_topic = DeclareLaunchArgument('cmd_vel_input_topic', default_value='/cmd_vel_nav_safe')
    odom_cmd_topic = DeclareLaunchArgument('odom_cmd_topic', default_value='/cmd_vel_nav_safe')
    odom_vel_response_tau = DeclareLaunchArgument('odom_vel_response_tau', default_value='0.18')

    # TF offsets
    laser_x = DeclareLaunchArgument('laser_x', default_value='0.0')
    laser_y = DeclareLaunchArgument('laser_y', default_value='0.0')
    laser_z = DeclareLaunchArgument('laser_z', default_value='0.09')
    laser_roll  = DeclareLaunchArgument('laser_roll',  default_value='0.0')
    laser_pitch = DeclareLaunchArgument('laser_pitch', default_value='0.0')
    laser_yaw   = DeclareLaunchArgument('laser_yaw',   default_value='3.141592653589793')

    imu_x = DeclareLaunchArgument('imu_x', default_value='0.0')
    imu_y = DeclareLaunchArgument('imu_y', default_value='0.0')
    imu_z = DeclareLaunchArgument('imu_z', default_value='0.0')
    imu_roll  = DeclareLaunchArgument('imu_roll',  default_value='0.0')
    imu_pitch = DeclareLaunchArgument('imu_pitch', default_value='0.0')
    imu_yaw   = DeclareLaunchArgument('imu_yaw',   default_value='0.0')

    # Cmd scaling
    max_vx = DeclareLaunchArgument('max_vx_mps', default_value='1.22')
    max_vy = DeclareLaunchArgument('max_vy_mps', default_value='1.22')
    max_wz = DeclareLaunchArgument('max_wz_rps', default_value='3.14')

    # AprilTags / Camera
    use_apriltags = DeclareLaunchArgument('use_apriltags', default_value='true')
    camera_device = DeclareLaunchArgument('camera_device', default_value='/dev/video0')

    camera_info_url = DeclareLaunchArgument(
        'camera_info_url',
        default_value='package://terralift/config/ost.yaml'
    )

    # base_link -> camera_link (physical mount pose)
    cam_x = DeclareLaunchArgument('cam_x', default_value='-0.16')
    cam_y = DeclareLaunchArgument('cam_y', default_value='0.0')
    cam_z = DeclareLaunchArgument('cam_z', default_value='0.025')
    cam_roll  = DeclareLaunchArgument('cam_roll',  default_value='3.141592653589793')
    cam_pitch = DeclareLaunchArgument('cam_pitch', default_value='-0.26179939')
    cam_yaw   = DeclareLaunchArgument('cam_yaw',   default_value='3.141592653589793')

    apriltag_params = DeclareLaunchArgument(
        'apriltag_params',
        default_value=PathJoinSubstitution([FindPackageShare('terralift'), 'config', 'apriltag.yaml'])
    )

    tag_map_file = DeclareLaunchArgument(
        'tag_map_file',
        default_value=PathJoinSubstitution([FindPackageShare('terralift'), 'config', 'tag_map.yaml'])
    )

    # Sensors & Base
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
        parameters=[{
            'cmd_topic': '/cmd_mecanum',
            'odom_topic': '/odom',
            'max_vx_mps': LaunchConfiguration('max_vx_mps'),
            'max_vy_mps': LaunchConfiguration('max_vy_mps'),
            'max_wz_rps': LaunchConfiguration('max_wz_rps'),
        }],
    )

    led_node = Node(
        package='terralift',
        executable='led_node',
        name='led_node',
        output='screen',
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
        remappings=[
            ('image_raw', 'image_mono'),
        ],
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
            'cmd_vel_topic': LaunchConfiguration('odom_cmd_topic'),
            'odom_topic': '/open_loop_odom',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'rate_hz': 50.0,
            'publish_tf': False,
            'reset_pose_topic': '/tag_reset_pose',
            'cmd_timeout': 0.35,
            'vel_response_tau': LaunchConfiguration('odom_vel_response_tau'),
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
        remappings=[
            ('odometry/filtered', '/odom'),
        ],
    )

    cmd_adapter = Node(
        package='terralift',
        executable='cmd_vel_to_mecanum',
        name='cmd_vel_to_mecanum',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('cmd_vel_input_topic'),
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

    research_trial_runner = Node(
        package='terralift',
        executable='research_trial_runner',
        name='research_trial_runner',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_trial_runner')),
        parameters=[{
            'record_bag': LaunchConfiguration('record_bag'),
            'record_all_topics': LaunchConfiguration('record_all_topics'),
            'bag_base_dir': LaunchConfiguration('bag_base_dir'),
            'trial_prefix': LaunchConfiguration('trial_prefix'),
            'environment_id': LaunchConfiguration('environment_id'),
            'trial_notes': LaunchConfiguration('trial_notes'),
            'nav2_params_name': LaunchConfiguration('nav2_params'),
            'parameter_set_id': LaunchConfiguration('parameter_set_id'),
            'nav2_version': LaunchConfiguration('nav2_version'),
            'robot_model': LaunchConfiguration('robot_model'),
            'robot_firmware_driver_versions': LaunchConfiguration('robot_firmware_driver_versions'),
            'use_apriltags': LaunchConfiguration('use_apriltags'),
            'use_slam': LaunchConfiguration('use_slam'),
            'auto_close': LaunchConfiguration('auto_close'),
            'startup_delay_sec': LaunchConfiguration('startup_delay_sec'),
            'goal_timeout_sec': LaunchConfiguration('goal_timeout_sec'),
            'goal_frame': LaunchConfiguration('goal_frame'),
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y'),
            'goal_yaw': LaunchConfiguration('goal_yaw'),
        }],
    )

    shutdown_on_trial_runner_exit = RegisterEventHandler(
        condition=IfCondition(LaunchConfiguration('enable_trial_runner')),
        event_handler=OnProcessExit(
            target_action=research_trial_runner,
            on_exit=[EmitEvent(event=Shutdown(reason='research trial runner exited'))],
        )
    )

    return LaunchDescription([
        use_sim_time, autostart, nav2_params, use_slam, enable_trial_runner,
        auto_close, record_bag, record_all_topics, bag_base_dir,
        trial_prefix, environment_id, trial_notes, parameter_set_id,
        nav2_version, robot_model, robot_firmware_driver_versions,
        startup_delay_sec, goal_timeout_sec, goal_frame, goal_x, goal_y, goal_yaw,
        cmd_vel_input_topic, odom_cmd_topic, odom_vel_response_tau,
        laser_x, laser_y, laser_z, laser_roll, laser_pitch, laser_yaw,
        imu_x, imu_y, imu_z, imu_roll, imu_pitch, imu_yaw,
        max_vx, max_vy, max_wz,
        use_apriltags, camera_device, camera_info_url,
        cam_x, cam_y, cam_z, cam_roll, cam_pitch, cam_yaw,
        apriltag_params, tag_map_file,

        imu, rplidar, drivetrain, led_node,
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
        research_trial_runner,
        shutdown_on_trial_runner_exit,
    ])
