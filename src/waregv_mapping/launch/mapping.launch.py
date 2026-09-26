import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    waregv_mapping_dir = get_package_share_directory("waregv_mapping")

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value='false',
        description='Use simulation clock'
    )

    slam_params_file_arg = DeclareLaunchArgument(
        name='slam_params_file',
        default_value=os.path.join(waregv_mapping_dir, 'config', 'slam_toolbox.yaml'),
        description='Path to SLAM parameters file'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration("slam_params_file"),
            {'use_sim_time': LaunchConfiguration("use_sim_time")}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_params_file_arg,
        slam_toolbox_node,
    ])