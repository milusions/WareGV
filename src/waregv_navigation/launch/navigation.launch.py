import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    waregv_navigation_dir = get_package_share_directory("waregv_navigation")
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    max_linear_velocity = LaunchConfiguration('max_linear_velocity')
    max_angular_velocity = LaunchConfiguration('max_angular_velocity')
    mode = LaunchConfiguration('mode')
    map_name = LaunchConfiguration('map_name')
    
    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value='false')
    params_file_arg = DeclareLaunchArgument(name='params_file', default_value=os.path.join(waregv_navigation_dir, 'config', 'nav2_params.yaml'))
    max_linear_velocity_arg = DeclareLaunchArgument(name='max_linear_velocity', default_value='0.11')
    max_angular_velocity_arg = DeclareLaunchArgument(name='max_angular_velocity', default_value='0.35')
    mode_arg = DeclareLaunchArgument(name='mode', default_value='slam_with_nav2')
    map_name_arg = DeclareLaunchArgument(name='map_name', default_value='map')

    param_substitutions = {
        'max_linear_vel': max_linear_velocity,
        'desired_linear_vel': max_linear_velocity,
        'max_angular_vel': max_angular_velocity,
        'max_rotational_vel': max_angular_velocity,
    }

    configured_params = RewrittenYaml(
        source_file=params_file, root_key='', param_rewrites=param_substitutions, convert_types=True
    )

    # Standard Nav Nodes
    lifecycle_nodes = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']
    
    controller_server = Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen', parameters=[configured_params, {'use_sim_time': use_sim_time}])
    planner_server = Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen', parameters=[configured_params, {'use_sim_time': use_sim_time}])
    behavior_server = Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen', parameters=[configured_params, {'use_sim_time': use_sim_time}])
    bt_navigator = Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen', parameters=[configured_params, {'use_sim_time': use_sim_time}])

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'autostart': True}, {'node_names': lifecycle_nodes}]
    )

    # --- AMCL & MAP SERVER CONDITIONS ---
    is_amcl = IfCondition(PythonExpression(["'", mode, "' == 'nav2_with_amcl'"]))
    amcl_lifecycle_nodes = ['map_server', 'amcl']

    map_server = Node(
        condition=is_amcl, package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[configured_params, {'yaml_filename': [os.path.expanduser('~/waregv/waregv_ws/src/waregv_mapping/maps/loaded/'), map_name, '.yaml']}]
    )
    
    amcl = Node(
        condition=is_amcl, package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[configured_params]
    )

    # Dedicated lifecycle manager for Localization (prevents hanging if AMCL isn't launched)
    amcl_lifecycle_manager = Node(
        condition=is_amcl, package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization', output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'autostart': True}, {'node_names': amcl_lifecycle_nodes}]
    )

    return LaunchDescription([
        use_sim_time_arg, params_file_arg, max_linear_velocity_arg, max_angular_velocity_arg, mode_arg, map_name_arg,
        controller_server, planner_server, behavior_server, bt_navigator, lifecycle_manager,
        map_server, amcl, amcl_lifecycle_manager
    ])