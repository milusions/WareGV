from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import  LaunchConfiguration, PythonExpression
from launch.conditions import  IfCondition
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions.node import Node


def generate_launch_description():
    
    waregv_navigation_dir = get_package_share_directory("waregv_navigation")
    waregv_mapping_dir = get_package_share_directory("waregv_mapping")

    mapping_check_arg = DeclareLaunchArgument(name="mapping_enable", default_value='true')
    navigation_check_arg = DeclareLaunchArgument(name="navigation_enable", default_value='true')
    map_name_arg = DeclareLaunchArgument(name="map_name", default_value='small_warehouse')
 
    map_name = LaunchConfiguration("map_name")
    
    map_file = PathJoinSubstitution([waregv_mapping_dir, "maps", map_name, [map_name,".yaml"]])
   
    nav2_params_file = os.path.join(waregv_navigation_dir, 'config', 'nav2_params.yaml')
    
    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value='true')
    
    nav_node = GroupAction(
            actions=[
                IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("mapping_enable"), "' == 'false' and '",
            LaunchConfiguration("navigation_enable"), "' == 'true'"
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration("use_sim_time"),
            'params_file': nav2_params_file,
            'map':map_file,
            'autostart': 'true',
        
        }.items()
    ),
                IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )),
             condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("mapping_enable"), "' == 'true' and '",
            LaunchConfiguration("navigation_enable"), "' == 'true'"
        ])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration("use_sim_time"),
              'params_file': nav2_params_file,
         
            'autostart': 'true',
        
        }.items()
    )
            ]
        )
    
    commander_node = Node(
        package="waregv_navigation",
        executable="commander",
   
         parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        output="screen",
    )
    
    commander_rest_server_node = Node(
        package="waregv_navigation",
        executable="commander_rest_server",
         parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_name_arg,
                          mapping_check_arg,
                          navigation_check_arg,
                            #   nav_node,
                            #   commander_node,
                              commander_rest_server_node,
                              
                             ])
