import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    waregv_mapping_dir = get_package_share_directory("waregv_mapping")
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time", 
        default_value='false'
    )
    
    slam_params_file_arg = DeclareLaunchArgument(
        name='slam_params_file',
        default_value=os.path.join(waregv_mapping_dir, 'config', 'slam_toolbox.yaml')
    )

    # Include SLAM Toolbox (online_async)
    slam_launch_path = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
    
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration("use_sim_time"), 
            'slam_params_file': LaunchConfiguration("slam_params_file")
        }.items() 
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_params_file_arg,
        slam_toolbox_node,
    ])