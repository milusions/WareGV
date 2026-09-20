import json
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources.frontend_launch_description_source import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.actions.group_action import GroupAction


def generate_launch_description():
    
    waregv_bringup_dir = get_package_share_directory("waregv_bringup")
    
    # Launch Arguments
    mapping_enable_arg = DeclareLaunchArgument(name="mapping_enable", default_value='true')
    navigation_enable_arg = DeclareLaunchArgument(name="navigation_enable", default_value='true')
    world_name_arg = DeclareLaunchArgument("world_name", default_value="small_warehouse")
    max_linear_velocity_arg = DeclareLaunchArgument(name="max_linear_velocity", default_value='0.5')
    max_angular_velocity_arg = DeclareLaunchArgument(name="max_angular_velocity", default_value=str(np.pi))
    wheel_radius_arg = DeclareLaunchArgument(name="wheel_radius", default_value='0.0385')
    wheel_base_arg = DeclareLaunchArgument(name="wheel_base", default_value='0.158')
    model_arg = DeclareLaunchArgument(name="model", default_value="waregv.urdf.xacro")
    robot_spawn_z_arg = DeclareLaunchArgument(name="spawn_z", default_value='0.5')
    map_name_arg = DeclareLaunchArgument(name="map_name", default_value='small_warehouse')
 
    map_name_conf = LaunchConfiguration("map_name")
    mapping_enable_conf = LaunchConfiguration("mapping_enable")
    navigation_enable_conf = LaunchConfiguration("navigation_enable")
    world_name_conf = LaunchConfiguration("world_name")
    max_linear_velocity_conf = LaunchConfiguration("max_linear_velocity")
    max_angular_velocity_conf = LaunchConfiguration("max_angular_velocity")
    wheel_radius_conf = LaunchConfiguration("wheel_radius")
    wheel_base_conf = LaunchConfiguration("wheel_base")
    model_conf = LaunchConfiguration("model")
    robot_spawn_z = LaunchConfiguration("spawn_z")
    
    use_sim_time = 'true' 
    
    # Bridges
    rosbridge_dir = get_package_share_directory('rosbridge_server')
    
    rosbridge_node = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(
        os.path.join(rosbridge_dir, 'launch', 'rosbridge_websocket_launch.xml')
    ),
    launch_arguments={'port': '9090', 'ssl': 'false','output': 'log'}.items(),
)
    
    qos_overrides = {
        "/initialpose": {"durability": "volatile"},
        "/map": {"durability": "transient_local", "reliability": "reliable", "history": "keep_last", "depth": 1}
    }


    foxglove_bridge = GroupAction(
        actions=[
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('foxglove_bridge'),
                        'launch',
                        'foxglove_bridge_launch.xml'
                    ])
                ),
                launch_arguments={
                    'port': '8765',
                    'topic_qos_overrides': json.dumps(qos_overrides)
                }.items()
            )
        ],
        scoped=True,
        forwarding=True,
       
    )
    
    # URDF & Gazebo
    waregv_urdf_launch_file_path = os.path.join(get_package_share_directory("waregv_description"), 'launch', 'urdf.launch.py')
    waregv_urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waregv_urdf_launch_file_path),
        launch_arguments={"use_sim_time": use_sim_time, "model": model_conf}.items() 
    )

    gazebo_sim_launch_file_path = os.path.join(get_package_share_directory("waregv_gazebo_sim"), 'launch', 'gazebo.launch.py')
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_sim_launch_file_path),
        launch_arguments={"world_name": world_name_conf}.items() 
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-world", world_name_conf, "-topic", "robot_description", "-name", "waregv", "-z", robot_spawn_z]
    )

    # Controller Spawners (Delayed by 8s to allow Gazebo & controller_manager to load)
    # Controller Spawners (Increased delay and added timeout flag)
    controller_spawners = GroupAction(
       
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "60"
                ],
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "velocity_controller",
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "60"
                ],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        ]
    )

    # Twist Mux
    twist_mux_node_config_filepath = os.path.join(waregv_bringup_dir, 'config', 'twist_mux.yaml')
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_node_config_filepath, {"use_stamped": False, "use_sim_time": True}],
        remappings=[('/cmd_vel_out', '/cmd_vel_unstamped')]
    )
    
    # Navigation, Mapping, Odometry, Controllers
    waregv_controller_launch_file_path = os.path.join(get_package_share_directory("waregv_controller"), 'launch', 'controller.launch.py')
    waregv_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waregv_controller_launch_file_path),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "max_angular_velocity": max_angular_velocity_conf, 
            "max_linear_velocity": max_linear_velocity_conf,
            "wheel_radius": wheel_radius_conf,
            "wheel_base": wheel_base_conf,
     
        }.items() 
    )
    

    waregv_web_dashboard_launch_file_path = os.path.join(get_package_share_directory("waregv_navigation"), 'launch', 'dashboard.launch.py')
    waregv_web_dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waregv_web_dashboard_launch_file_path),
        launch_arguments={
            "use_sim_time": use_sim_time,
  
        }.items() 
    )
    
    waregv_odometry_launch_file_path = os.path.join(get_package_share_directory("waregv_odometry"), 'launch', 'odometry.launch.py')
    waregv_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waregv_odometry_launch_file_path),
        launch_arguments={"use_sim_time": use_sim_time, "wheel_radius": wheel_radius_conf}.items() 
    )

    # RViz (Delayed by 12s)
    # rviz_filename = PythonExpression(["'navigation.rviz' if '", navigation_enable_conf, "' == 'true' else 'mapping.rviz'"])
    # delayed_rviz = TimerAction(
    #     period=12.0,
    #     actions=[
    #         Node(
    #             package="rviz2",
    #             executable="rviz2",
    #             name="rviz2",
    #             output="screen",
    #             parameters=[{'use_sim_time': True}],
    #             arguments=["-d", PathJoinSubstitution([waregv_bringup_dir, "rviz", rviz_filename])]
    #         )
    #     ]
    # )

    return LaunchDescription([
        mapping_enable_arg,
        robot_spawn_z_arg,
        navigation_enable_arg,
        map_name_arg,
        world_name_arg,
        max_linear_velocity_arg,
        max_angular_velocity_arg,
        wheel_radius_arg,
        wheel_base_arg,
        model_arg,
        foxglove_bridge,
        rosbridge_node,
        waregv_urdf,
        gazebo_sim,
        gz_spawn_entity,
        controller_spawners,
        twist_mux_node,
        waregv_odometry,
        waregv_controller,
waregv_web_dashboard,
        # delayed_rviz,
    ])