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

    max_linear_velocity_arg = DeclareLaunchArgument(
        name="max_linear_velocity", default_value="0.1"
    )
    max_angular_velocity_arg = DeclareLaunchArgument(
        name="max_angular_velocity", default_value=str(np.pi / 4)
    )
    wheel_radius_arg = DeclareLaunchArgument(
        name="wheel_radius", default_value="0.0385"
    )
    wheel_base_arg = DeclareLaunchArgument(
        name="wheel_base", default_value="0.158"
    )
    map_name_arg = DeclareLaunchArgument(name="map_name", default_value="map")

    max_linear_velocity_conf = LaunchConfiguration("max_linear_velocity")
    max_angular_velocity_conf = LaunchConfiguration("max_angular_velocity")
    wheel_radius_conf = LaunchConfiguration("wheel_radius")
    wheel_base_conf = LaunchConfiguration("wheel_base")

    use_sim_time = "false"

    qos_overrides = {
        "/initialpose": {"durability": "volatile"},
        "/map": {
            "durability": "transient_local",
            "reliability": "reliable",
            "history": "keep_last",
            "depth": 1,
        },
    }

    waregv_description_launch_file_path = os.path.join(
        get_package_share_directory("waregv_description"),
        "launch",
        "description.launch.py",
    )
    waregv_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waregv_description_launch_file_path),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    waregv_hardware_launch_file_path = os.path.join(
        get_package_share_directory("waregv_hardware"), "launch", "hardware.launch.py"
    )
    waregv_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waregv_hardware_launch_file_path),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    twist_mux_node_config_filepath = os.path.join(
        waregv_bringup_dir, "config", "twist_mux.yaml"
    )
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[twist_mux_node_config_filepath, {"use_stamped": False}],
        remappings=[("/cmd_vel_out", "/cmd_vel")],
    )
    
    efk_node_params_file = os.path.join(
        waregv_bringup_dir, "config", "ekf_filter.yaml"
    )

    robot_localization_node = Node(
    package="robot_localization",  # Fixed package name
    executable="ekf_node",
    name="ekf_filter_node",
    parameters=[efk_node_params_file],
    remappings=[
        ('odometry/filtered', '/odometry/filtered')
    ]
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
            "wheel_base": wheel_base_conf,
        }.items(),
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

    waregv_dashboard_launch_file_path = os.path.join(
        get_package_share_directory("waregv_dashboard"),
        "launch",
        "dashboard.launch.py",
    )
    waregv_dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waregv_dashboard_launch_file_path),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    return LaunchDescription(
[
            max_linear_velocity_arg,
            max_angular_velocity_arg,
            wheel_radius_arg,
            wheel_base_arg,
            map_name_arg,
            waregv_description,
            waregv_hardware,
            twist_mux_node,
            robot_localization_node,
            waregv_controller,
            waregv_mapping,
            waregv_navigation,
            rosbridge_node,
            foxglove_bridge,
            waregv_dashboard,
        ]
    )
