import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    waregv_navigation_dir = get_package_share_directory("waregv_navigation")

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    max_linear_velocity = LaunchConfiguration('max_linear_velocity')
    max_angular_velocity = LaunchConfiguration('max_angular_velocity')
    map_yaml_file = LaunchConfiguration('map_yaml_file')

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    params_file_arg = DeclareLaunchArgument(
        name='params_file',
        default_value=os.path.join(waregv_navigation_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all Nav2 nodes'
    )

    max_linear_velocity_arg = DeclareLaunchArgument(
        name='max_linear_velocity',
        default_value='0.11',
        description='Maximum linear velocity to override in Nav2 Params'
    )

    max_angular_velocity_arg = DeclareLaunchArgument(
        name='max_angular_velocity',
        default_value='0.35',
        description='Maximum angular/rotational velocity to override in Nav2 Params'
    )

    map_yaml_file_arg = DeclareLaunchArgument(
        name='map_yaml_file',
        default_value=os.path.join(waregv_navigation_dir, 'maps', 'warehouse_map.yaml'),
        description='Full path to static map yaml file'
    )

    param_substitutions = {
        'max_linear_vel': max_linear_velocity,
        'desired_linear_vel': max_linear_velocity,
        'max_angular_vel': max_angular_velocity,
        'max_rotational_vel': max_angular_velocity,
        'yaml_filename': map_yaml_file,
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params, {'use_sim_time': use_sim_time}]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params, {'use_sim_time': use_sim_time}]
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params, {'use_sim_time': use_sim_time}]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params, {'use_sim_time': use_sim_time}]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params, {'use_sim_time': use_sim_time}]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        max_linear_velocity_arg,
        max_angular_velocity_arg,
        map_yaml_file_arg,
        map_server,
        amcl,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
    ])