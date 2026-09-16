from launch import LaunchDescription
from pathlib import Path
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction, GroupAction
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import UnlessCondition, IfCondition


def generate_launch_description():
    
    waregv_mapping_dir = get_package_share_directory("waregv_mapping")
    
    mapping_check_arg = DeclareLaunchArgument(name="mapping_enable", default_value='true')
    
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    slam_launch_path = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
    
    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=os.path.join(waregv_mapping_dir, 'config', 'slam_toolbox.yaml')
    )
    
    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value='true')

    
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        condition=IfCondition(LaunchConfiguration("mapping_enable")),
        launch_arguments={'use_sim_time': LaunchConfiguration("use_sim_time"), "slam_params_file":slam_params_file }.items() 
    )

    return LaunchDescription([
        use_sim_time_arg,
                          mapping_check_arg,
                              slam_toolbox_node,
                              
                             ])
