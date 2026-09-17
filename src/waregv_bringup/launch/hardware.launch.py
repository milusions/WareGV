from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import PathJoinSubstitution
import numpy as np
from launch.launch_description_sources.frontend_launch_description_source import FrontendLaunchDescriptionSource


def generate_launch_description():
    
    waregv_bringup_dir = get_package_share_directory("waregv_bringup")
    
    mapping_enable_arg = DeclareLaunchArgument(name="mapping_enable", default_value='true')
    navigation_enable_arg = DeclareLaunchArgument(name="navigation_enable", default_value='true')
    max_linear_velocity_arg = DeclareLaunchArgument(name="max_linear_velocity", default_value='0.5')
    max_angular_velocity_arg = DeclareLaunchArgument(name="max_angular_velocity", default_value=str(np.pi))
    wheel_radius_arg = DeclareLaunchArgument(name="wheel_radius", default_value='0.0325')
    wheel_base_arg = DeclareLaunchArgument(name="wheel_base", default_value='0.176')
    map_name_arg = DeclareLaunchArgument(name="map_name", default_value='small_warehouse')
 
    map_name_conf = LaunchConfiguration("map_name")
    mapping_enable_conf = LaunchConfiguration("mapping_enable")
    navigation_enable_conf = LaunchConfiguration("navigation_enable")
    max_linear_velocity_conf = LaunchConfiguration("max_linear_velocity")
    max_angular_velocity_conf = LaunchConfiguration("max_angular_velocity")
    wheel_radius_conf = LaunchConfiguration("wheel_radius")
    wheel_base_conf = LaunchConfiguration("wheel_base")

    use_sim_time = 'false' 
    
    rosbridge_dir = get_package_share_directory('rosbridge_server')
    
    rosbridge_node = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(rosbridge_dir, 'launch', 'rosbridge_websocket_launch.xml')
        ),

        launch_arguments={
            'port': '9090',
            'ssl': 'false'
        }.items()
    )
    
    twist_mux_node_config_filepath = os.path.join(waregv_bringup_dir, 'config', 'twist_mux.yaml')

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_node_config_filepath, {"use_stamped": False}],
        remappings=[('/cmd_vel_out', '/cmd_vel_unstamped')]
    )
    
    heartbeat_light = Node(
        package='waregv_heartbeat',
        executable='heartbeat_light',
        name='heartbeat_light',
            parameters=[{'use_sim_time': use_sim_time}],
       
    )
    

    waregv_controller_launch_file_path = os.path.join(get_package_share_directory("waregv_controller"), 'launch', 'controller.launch.py')
    waregv_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waregv_controller_launch_file_path),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "max_angular_velocity": max_angular_velocity_conf, 
            "max_linear_velocity": max_linear_velocity_conf,
              "wheel_radius": wheel_radius_conf,
                "wheel_base": wheel_base_conf
        }.items() 
    )
    
    waregv_driver_launch_file_path = os.path.join(get_package_share_directory("waregv_driver"), 'launch', 'driver.launch.py')
    
    waregv_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waregv_driver_launch_file_path),
        launch_arguments={"use_sim_time":use_sim_time}.items() 
    )
    
    waregv_mapping_launch_file_path = os.path.join(get_package_share_directory("waregv_mapping"), 'launch', 'mapping.launch.py')
    waregv_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waregv_mapping_launch_file_path),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "mapping_enable": mapping_enable_conf
        }.items() 
    )
    
    waregv_navigation_launch_file_path = os.path.join(get_package_share_directory("waregv_navigation"), 'launch', 'navigation.launch.py')
    waregv_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waregv_navigation_launch_file_path),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "mapping_enable": mapping_enable_conf, 
            "navigation_enable": navigation_enable_conf,
            "map_name":map_name_conf
        }.items() 
    )
    
    waregv_odometry_launch_file_path = os.path.join(get_package_share_directory("waregv_odometry"), 'launch', 'odometry.launch.py')
    waregv_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waregv_odometry_launch_file_path),
        launch_arguments={
            "use_sim_time": use_sim_time, 
            "wheel_radius": wheel_radius_conf
        }.items() 
    )

    return LaunchDescription([
        mapping_enable_arg,
        navigation_enable_arg,
        max_linear_velocity_arg,
        map_name_arg,
        max_angular_velocity_arg,
        wheel_radius_arg,
        wheel_base_arg,
        heartbeat_light,
        rosbridge_node,
        waregv_driver,
        twist_mux_node,
        waregv_odometry,
        waregv_controller,
        waregv_mapping,
        waregv_navigation
    ])

