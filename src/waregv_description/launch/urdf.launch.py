import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    # Launch Arguments
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value="waregv.urdf.xacro"
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time", 
        default_value='true'
    )
    
    urdf_visualization_arg = DeclareLaunchArgument(
        name="visualize", 
        default_value='false'
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # Path & Robot Description
    urdf_file_path = PathJoinSubstitution([
        get_package_share_directory("waregv_description"), 
        "urdf", 
        LaunchConfiguration('model')
    ])
    
    robot_description = ParameterValue(
        Command(["xacro ", urdf_file_path]), 
        value_type=str
    )
    
    # Conditions
    is_visualize_active = IfCondition(
        PythonExpression(["'", LaunchConfiguration("visualize"), "' == 'true'"])
    )
    
    # Nodes
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time
        }]
    )
    
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=is_visualize_active
    )
    
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-d", PathJoinSubstitution([
            get_package_share_directory("waregv_description"), 
            "rviz", 
            "urdf.rviz"
        ])],
        condition=is_visualize_active
    )

    return LaunchDescription([
        use_sim_time_arg,
        model_arg,
        urdf_visualization_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2
    ])