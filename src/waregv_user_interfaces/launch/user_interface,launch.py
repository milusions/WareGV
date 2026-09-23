from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions.python_expression import PythonExpression
from launch.substitutions.launch_configuration import LaunchConfiguration
from waregv_ws.src.waregv_user_interfaces import waregv_user_interfaces


def generate_launch_description():
    
    
    waregv_user_interfaces = Node(
        package="waregv_user_interfaces",
        executable="odometry",
             parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time"), "wheel_radius":LaunchConfiguration('wheel_radius')}],
        output="screen",
    )
    


    return LaunchDescription([
 
                              waregv_user_interfaces,
                              
                             ])
