from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions.node import Node


def generate_launch_description():
    waregv_navigation_dir = get_package_share_directory("waregv_navigation")
    
    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value='false')
    
    nav2_params_file = os.path.join(waregv_navigation_dir, 'config', 'nav2_params.yaml')
    
    nav2_launch_path = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')

    nav_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_path),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration("use_sim_time"),
                    'params_file': nav2_params_file,
                    'use_amcl': 'false',       
        'autostart': 'true',
                }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        nav_node,
      
    ])
