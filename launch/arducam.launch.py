from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_device = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='V4L2 device (USB camera usually /dev/video0)'
    )

    camera_name = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Namespace for camera topics'
    )

    # Camera node (publishes image_raw + camera_info)
    cam = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        output='screen',
        namespace=LaunchConfiguration('camera_name'),
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
        }]
    )

    # Rectification node (publishes image_rect)
    rectify = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify',
        output='screen',
        namespace=LaunchConfiguration('camera_name'),
    )

    return LaunchDescription([
        video_device,
        camera_name,
        cam,
        rectify
    ])
