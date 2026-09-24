from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('color_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_rect_raw'),
        DeclareLaunchArgument('port', default_value='8081'),
        DeclareLaunchArgument('fps', default_value='15'),
        DeclareLaunchArgument('depth_max_mm', default_value='6000.0'),
        Node(
            package='ros2_webrtc_streamer',
            executable='webrtc_streamer',
            name='realsense_webrtc_bridge',
            output='screen',
            parameters=[],
            arguments=[
                '--color-topic', LaunchConfiguration('color_topic'),
                '--depth-topic', LaunchConfiguration('depth_topic'),
                '--port', LaunchConfiguration('port'),
                '--fps', LaunchConfiguration('fps'),
                '--depth-max-mm', LaunchConfiguration('depth_max_mm'),
            ],
        ),
    ])
