import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    waregv_nav_dir = get_package_share_directory("waregv_navigation")
    waregv_mapping_dir = get_package_share_directory("waregv_mapping")
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    initial_mode = LaunchConfiguration('initial_mode')

    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock'
    )
    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(waregv_nav_dir, 'config', 'nav2_params.yaml'),
        description='Nav2 configuration file'
    )
    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(waregv_mapping_dir, 'config', 'slam_toolbox.yaml'),
        description='SLAM Toolbox configuration file'
    )
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml_file',
        default_value=os.path.join(waregv_nav_dir, 'maps', 'warehouse_map.yaml'),
        description='Map file for AMCL'
    )
    initial_mode_arg = DeclareLaunchArgument(
        'initial_mode',
        default_value='slam_only',
        description='Initial mode: slam_only | nav2_amcl | slam_nav2'
    )

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file,
    }

    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # 1. SLAM Toolbox Node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}]
    )

    # 2. Map Server Node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_nav2_params]
    )

    # 3. AMCL Node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_nav2_params]
    )

    # 4. Nav2 Controllers and Planners
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_nav2_params]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_nav2_params]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_nav2_params]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_nav2_params]
    )

    # 5. Dynamic Mode Manager
    mode_manager_node = Node(
        package='waregv_navigation',
        executable='mode_manager',
        name='mode_manager',
        output='screen',
        parameters=[{'initial_mode': initial_mode}]
    )
    
    auto_mode_switcher = Node(
            package='waregv_navigation',
            executable='auto_mode_switcher',
            name='auto_mode_switcher',
            output='screen',
            parameters=[{'initial_mode': initial_mode}]
        )

    return LaunchDescription([
        use_sim_time_arg,
        nav2_params_arg,
        slam_params_arg,
        map_yaml_arg,
        initial_mode_arg,
        slam_toolbox_node,
        map_server_node,
        amcl_node,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        mode_manager_node,
        auto_mode_switcher
    ])