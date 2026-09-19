from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions.python_expression import PythonExpression
from launch.substitutions.launch_configuration import LaunchConfiguration


def generate_launch_description():
    
    wheel_radius_arg = DeclareLaunchArgument(name="wheel_radius", default_value='0.0325')
    
    
    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value='true')
    
    waregv_odometry = Node(
        package="waregv_odometry",
        executable="odometry",
             parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time"), "wheel_radius":LaunchConfiguration('wheel_radius')}],
        output="screen",
    )
    
    odom_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_footprint"],
            parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}]
       
    )

    return LaunchDescription([
        wheel_radius_arg,
        use_sim_time_arg,
 odom_frame,
                            #   waregv_odometry,
                              
                             ])
