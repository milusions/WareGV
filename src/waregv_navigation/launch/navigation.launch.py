import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # 1. Get the path to the package directory to locate the params file
    waregv_navigation_dir = get_package_share_directory("waregv_navigation")
    
    # 2. Setup Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    max_linear_velocity = LaunchConfiguration('max_linear_velocity')
    max_angular_velocity = LaunchConfiguration('max_angular_velocity')
    
    # 3. Declare Launch Arguments
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

    # 4. Create the RewrittenYaml parameter substitution logic
    param_substitutions = {
        'max_linear_vel': max_linear_velocity,
        'desired_linear_vel': max_linear_velocity,
        'max_angular_vel': max_angular_velocity,
        'max_rotational_vel': max_angular_velocity,
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # 5. Define the Lifecycle Nodes required for basic Navigation
    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator']

    # Node A: Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params, {'use_sim_time': use_sim_time}]
    )

    # Node B: Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params, {'use_sim_time': use_sim_time}]
    )

    # Node C: Behavior Server 
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params, {'use_sim_time': use_sim_time}]
    )

    # Node D: Behavior Tree Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params, {'use_sim_time': use_sim_time}]
    )

    # 6. Define the Lifecycle Manager Node
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': lifecycle_nodes}
        ]
    )

    # 7. Return the LaunchDescription object
    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        max_linear_velocity_arg,
        max_angular_velocity_arg,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager
    ])