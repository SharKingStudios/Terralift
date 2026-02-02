from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='RPLidar serial port'
    )

    serial_baudrate = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='RPLidar baudrate'
    )

    frame_id = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='Laser frame id'
    )

    angle_compensate = DeclareLaunchArgument(
        'angle_compensate',
        default_value='true',
        description='Angle compensation'
    )

    scan_mode = DeclareLaunchArgument(
        'scan_mode',
        default_value='Standard',
        description='Scan mode (driver/model dependent)'
    )

    node = Node(
        package='rplidar_ros',
        executable='rplidar_node',   # if this fails, check: ros2 pkg executables rplidar_ros
        name='rplidar',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'frame_id': LaunchConfiguration('frame_id'),
            'angle_compensate': LaunchConfiguration('angle_compensate'),
            'scan_mode': LaunchConfiguration('scan_mode'),
        }]
    )

    return LaunchDescription([
        serial_port,
        serial_baudrate,
        frame_id,
        angle_compensate,
        scan_mode,
        node
    ])
