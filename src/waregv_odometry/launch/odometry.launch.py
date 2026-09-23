from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions.python_expression import PythonExpression
from launch.substitutions.launch_configuration import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    wheel_radius_arg = DeclareLaunchArgument(name="wheel_radius", default_value='0.036')
    
    
    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value='true')
    waregv_odometry_dir = get_package_share_directory('waregv_odometry')
    ekf_config = os.path.join(waregv_odometry_dir, 'config', 'ekf.yaml')
    
    waregv_odometry = Node(
        package="waregv_odometry",
        executable="odometry",
             parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time"), "wheel_radius":LaunchConfiguration('wheel_radius')}],
        output="screen",
    ),
    ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
            remappings=[('odometry/filtered', '/odom')],
        ),
    
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
                              waregv_odometry,
                              ekf_node
                              
                             ])
