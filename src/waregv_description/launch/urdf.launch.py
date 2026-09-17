from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions.if_condition import IfCondition
from launch.substitutions.python_expression import PythonExpression
from launch.substitutions.path_join_substitution import PathJoinSubstitution


def generate_launch_description():
    
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value="waregv.urdf.xacro"
    )
    
    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value='true')
    
    
    urdf_file_path = PathJoinSubstitution([get_package_share_directory("waregv_description"), "urdf",LaunchConfiguration('model') ])
    
    urdf_visualization_arg = DeclareLaunchArgument(name="visualize", default_value='false')
    
    robot_description = ParameterValue(Command(["xacro ", urdf_file_path]), value_type=str)
    
    
    is_visualize_active = IfCondition(
        PythonExpression(["'", LaunchConfiguration("visualize"), "' == 'true'"])
    )
    
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":robot_description,'use_sim_time': LaunchConfiguration("use_sim_time"), }]
    )
    
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        condition=is_visualize_active
       
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        arguments=["-d", PathJoinSubstitution([get_package_share_directory("waregv_description"), "rviz", "urdf.rviz"])],
         condition=is_visualize_active
       
    )
    return LaunchDescription(
        
        [use_sim_time_arg,
         model_arg,
         urdf_visualization_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2]
        )
