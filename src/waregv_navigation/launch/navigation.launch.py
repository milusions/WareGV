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
    waregv_mapping_dir = get_package_share_directory("waregv_mapping")

    mapping_check_arg = DeclareLaunchArgument(name="mapping_enable", default_value='true')
    navigation_check_arg = DeclareLaunchArgument(name="navigation_enable", default_value='true')
    start_commander_arg = DeclareLaunchArgument(name="start_commander", default_value='false')  # driver launch already runs it
    map_name_arg = DeclareLaunchArgument(name="map_name", default_value='small_warehouse')
    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value='false')
 
    map_name = LaunchConfiguration("map_name")
    
    # Maps are saved as maps/<name>/<name>.yaml (NOT flat). A wrong path makes map_server fail,
    # the lifecycle manager aborts and Nav2 never becomes active.
    map_file = PathJoinSubstitution([waregv_mapping_dir, "maps", map_name, PythonExpression(["'", map_name, ".yaml'"])])
    nav2_params_file = os.path.join(waregv_navigation_dir, 'config', 'nav2_params.yaml')
    
    nav_node = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py'
                )),
                condition=IfCondition(PythonExpression([
                    "'", LaunchConfiguration("mapping_enable"), "' == 'false' and '",
                    LaunchConfiguration("navigation_enable"), "' == 'true'"
                ])),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration("use_sim_time"),
                    'params_file': nav2_params_file,
                    'map': map_file,
                    'autostart': 'true',
                }.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py'
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
        condition=IfCondition(LaunchConfiguration("start_commander")),
    )

    return LaunchDescription([
        use_sim_time_arg,
        start_commander_arg,
        map_name_arg,
        mapping_check_arg,
        navigation_check_arg,
        nav_node,
        commander_node,
    ])