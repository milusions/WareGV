from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import PathJoinSubstitution
import numpy as np


def generate_launch_description():
    
    waregv_bringup_dir = get_package_share_directory("waregv_bringup")
    
    mapping_enable_arg = DeclareLaunchArgument(name="mapping_enable", default_value='true')
    navigation_enable_arg = DeclareLaunchArgument(name="navigation_enable", default_value='true')
    world_name_arg = DeclareLaunchArgument("world_name", default_value="small_warehouse")
    max_linear_velocity_arg = DeclareLaunchArgument(name="max_linear_velocity", default_value='0.5')
    max_angular_velocity_arg = DeclareLaunchArgument(name="max_angular_velocity", default_value=str(np.pi))
    wheel_radius_arg = DeclareLaunchArgument(name="wheel_radius", default_value='0.0325')
    wheel_base_arg = DeclareLaunchArgument(name="wheel_base", default_value='0.176')
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
    robot_spawn_z = [LaunchConfiguration("spawn_z")]
    
    use_sim_time = 'true' 
    
    
    
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-world", world_name_conf, "-topic", "robot_description", "-name", "waregv", "-z", robot_spawn_z]
        
    )
    
    
    
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
   
    spawn_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    twist_mux_node_config_filepath = os.path.join(waregv_bringup_dir, 'config', 'twist_mux.yaml')

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_node_config_filepath, {"use_stamped": False}],
        remappings=[('/cmd_vel_out', '/cmd_vel_unstamped')]
    )
    
    rviz_filename = PythonExpression(["'navigation.rviz' if '", navigation_enable_conf, "' == 'true' else 'mapping.rviz'"])
    
    delayed_rviz = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                parameters=[{'use_sim_time': True}],
                arguments=["-d", PathJoinSubstitution([waregv_bringup_dir, "rviz", rviz_filename])]
            )
        ]
    )
    
    gazebo_sim_launch_file_path = os.path.join(get_package_share_directory("waregv_gazebo_sim"), 'launch', 'gazebo.launch.py')
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_sim_launch_file_path),
        launch_arguments={"world_name": world_name_conf}.items() 
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
    
    waregv_urdf_launch_file_path = os.path.join(get_package_share_directory("waregv_description"), 'launch', 'urdf.launch.py')
    waregv_urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waregv_urdf_launch_file_path),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "model": model_conf
        }.items() 
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
        robot_spawn_z_arg,
        navigation_enable_arg,
        map_name_arg,
        world_name_arg,
        max_linear_velocity_arg,
        max_angular_velocity_arg,
        wheel_radius_arg,
        wheel_base_arg,
        model_arg,
        waregv_urdf,
        gazebo_sim,
        gz_spawn_entity,
        spawn_joint_state_broadcaster,
        spawn_velocity_controller,
        twist_mux_node,
        waregv_odometry,
        waregv_controller,
        waregv_mapping,
        waregv_navigation,
        delayed_rviz,
    ])
