import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('waregv_odometry')
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),   # 'false' on the real rover
        DeclareLaunchArgument('wheel_radius', default_value='0.03706'),
        DeclareLaunchArgument('wheel_separation', default_value='0.39598'),  # calibrate!
        DeclareLaunchArgument('publish_euler_debug', default_value='true'),
    ]

    # NOTE: no commas after these assignments (a trailing comma makes a tuple).
    wheel_odometry = Node(
        package='waregv_odometry',
        executable='wheel_odometry_node',
        name='wheel_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'wheel_radius': ParameterValue(LaunchConfiguration('wheel_radius'), value_type=float),
            'wheel_separation': ParameterValue(LaunchConfiguration('wheel_separation'), value_type=float),
            'gyro_scale': 0.97656,
            'ekf_mode': True,         
        }],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config,
                    {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        remappings=[('odometry/filtered', '/odom')],
    )

  

    return LaunchDescription(args+[ekf_node])