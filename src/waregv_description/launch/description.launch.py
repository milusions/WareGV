import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value='false',
        description="Use simulation (Gazebo) clock if true"
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")

    urdf_file_path = PathJoinSubstitution([
        get_package_share_directory("waregv_description"),
        "urdf",
        "waregv.urdf.xacro"
    ])
    
    controllers_yaml_path = PathJoinSubstitution([
        get_package_share_directory("waregv_description"),
        "config",
        "controller.yaml"
    ])
    
    # Process Xacro into URDF string, passing the controllers_yaml_path
    robot_description = ParameterValue(
        Command([
            "xacro ", urdf_file_path, 
            " controllers_yaml_path:=", controllers_yaml_path
        ]),
        value_type=str
    )

    # 1. Robot State Publisher (Publishes /robot_description topic)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time
        }]
    ) 

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
    ])