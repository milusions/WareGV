from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions.node import Node


def generate_launch_description():

    mapping_check_arg = DeclareLaunchArgument(name="mapping_enable", default_value='true')
    navigation_check_arg = DeclareLaunchArgument(name="navigation_enable", default_value='true')
    map_name_arg = DeclareLaunchArgument(name="map_name", default_value='small_warehouse')
    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value='true')
 
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
 
        commander_node,
        # commander_rest_server_node
    ])