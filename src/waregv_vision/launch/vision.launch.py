from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions.node import Node


def generate_launch_description():

   
    aruco_node = Node(
        package="waregv_vision",
        executable="aruco_node",
        parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        output="screen",
    )
    
    camera_streamer = Node(
            package="waregv_vision",
            executable="camera_streamer",
            parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}],
            output="screen",
        )

    return LaunchDescription([
 
     aruco_node,
     camera_streamer
    ])