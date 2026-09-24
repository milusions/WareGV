from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions.node import Node


def generate_launch_description():

   
    dashboard_backend = Node(
        package="waregv_dashboard",
        executable="dashboard_backend",
        parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    return LaunchDescription([
 
     dashboard_backend
    ])