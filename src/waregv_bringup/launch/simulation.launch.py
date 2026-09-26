import json
import os
import numpy as np

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources.frontend_launch_description_source import (
    FrontendLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    waregv_bringup_dir = get_package_share_directory("waregv_bringup")
    
    rosbridge_dir = get_package_share_directory("rosbridge_server")
   
    world_name_arg = DeclareLaunchArgument(
        "world_name", default_value="small_warehouse"
    )
    max_linear_velocity_arg = DeclareLaunchArgument(
        name="max_linear_velocity", default_value="0.11"
    )
    max_angular_velocity_arg = DeclareLaunchArgument(
        name="max_angular_velocity", default_value=str(0.35)
    )
    wheel_radius_arg = DeclareLaunchArgument(
        name="wheel_radius", default_value="0.035"
    )
    track_width_arg = DeclareLaunchArgument(
        name="track_width", default_value="0.168"
    )
    model_arg = DeclareLaunchArgument(
        name="model", default_value="waregv.urdf.xacro"
    )
    robot_spawn_z_arg = DeclareLaunchArgument(name="spawn_z", default_value="0.5")

    max_linear_velocity_conf = LaunchConfiguration("max_linear_velocity")
    max_angular_velocity_conf = LaunchConfiguration("max_angular_velocity")
    wheel_radius_conf = LaunchConfiguration("wheel_radius")
    track_width_conf = LaunchConfiguration("track_width")
    world_name_conf = LaunchConfiguration("world_name")
    model_conf = LaunchConfiguration("model")
    robot_spawn_z = LaunchConfiguration("spawn_z")

    use_sim_time = "true"

    qos_overrides = {
        "/initialpose": {"durability": "volatile"},
        "/map": {
            "durability": "transient_local",
            "reliability": "reliable",
            "history": "keep_last",
            "depth": 1,
        },
    }

    waregv_urdf_launch_file_path = os.path.join(
        get_package_share_directory("waregv_description"),
        "launch",
        "description.launch.py",
    )
    waregv_urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waregv_urdf_launch_file_path),
        launch_arguments={"use_sim_time": use_sim_time, "model": model_conf}.items(),
    )

    gazebo_sim_launch_file_path = os.path.join(
        get_package_share_directory("waregv_gazebo_sim"), "launch", "gazebo.launch.py"
    )
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_sim_launch_file_path),
        launch_arguments={"world_name": world_name_conf,"use_sim_time": use_sim_time}.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world",
            world_name_conf,
            "-topic",
            "robot_description",
            "-name",
            "waregv",
            "-z",
            robot_spawn_z,
        ],
    )

    controller_spawners = GroupAction(
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "60",
                ],
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "velocity_controller",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "60",
                ],
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
        ]
    )
   
    twist_mux_node_config_filepath = os.path.join(
        waregv_bringup_dir, "config", "twist_mux.yaml"
    )
    
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[twist_mux_node_config_filepath, {"use_stamped": False}],
        remappings=[("/cmd_vel_out", "/cmd_vel_unstamped")],
    )

    waregv_odometry_launch_file_path = os.path.join(
            get_package_share_directory("waregv_odometry"),
            "launch",
            "odometry.launch.py",
        )
    waregv_odometry = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(waregv_odometry_launch_file_path),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "wheel_radius": wheel_radius_conf,
            }.items(),
        )
        
    waregv_controller_launch_file_path = os.path.join(
        get_package_share_directory("waregv_controller"),
        "launch",
        "controller.launch.py",
    )
    
    waregv_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waregv_controller_launch_file_path),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "max_angular_velocity": max_angular_velocity_conf,
            "max_linear_velocity": max_linear_velocity_conf,
            "wheel_radius": wheel_radius_conf,
            "track_width": track_width_conf,
        }.items(),
    )

    rosbridge_node = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(rosbridge_dir, "launch", "rosbridge_websocket_launch.xml")
        ),
        launch_arguments={
            "port": "9090",
            "ssl": "false",
            "output": "log",
        }.items(),
    )

    foxglove_bridge = GroupAction(
            actions=[
                IncludeLaunchDescription(
                    XMLLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [
                                FindPackageShare("foxglove_bridge"),
                                "launch",
                                "foxglove_bridge_launch.xml",
                            ]
                        )
                    ),
                    launch_arguments={
                        "port": "8765",
                        "address": "0.0.0.0",
                        "topic_qos_overrides": json.dumps(qos_overrides),
                    }.items(),
                )
            ],
            scoped=True,
            forwarding=True,
        )
    
    waregv_mapping_launch_file_path = os.path.join(
            get_package_share_directory("waregv_mapping"), "launch", "mapping.launch.py"
        )
    waregv_mapping = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(waregv_mapping_launch_file_path),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        )
    
    waregv_navigation_launch_file_path = os.path.join(
            get_package_share_directory("waregv_navigation"),
            "launch",
            "navigation.launch.py",
        )
    waregv_navigation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(waregv_navigation_launch_file_path),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "max_linear_velocity": max_linear_velocity_conf,
                "max_angular_velocity": max_angular_velocity_conf,
            }.items(),
        )
    waregv_suite_launch_file_path = os.path.join(
                get_package_share_directory("waregv_suite"), "launch", "suite.launch.py"
            )
    waregv_suite = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(waregv_suite_launch_file_path),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            )
    
    
    waregv_user_interfaces_launch_file_path = os.path.join(
                get_package_share_directory("waregv_user_interfaces"), "launch", "user_interfaces.launch.py"
            )
    waregv_user_interfaces = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(waregv_user_interfaces_launch_file_path),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            )
    
    return LaunchDescription(
        [
            robot_spawn_z_arg,
            world_name_arg,
            max_linear_velocity_arg,
            max_angular_velocity_arg,
            wheel_radius_arg,
            track_width_arg,
            model_arg,
            waregv_urdf,
            rosbridge_node,
            foxglove_bridge,
            waregv_user_interfaces,
            gazebo_sim,
            gz_spawn_entity,
            controller_spawners,
            twist_mux_node,
            waregv_odometry,
            waregv_controller,
            waregv_mapping,
            waregv_navigation,
            waregv_suite
        ]
    )