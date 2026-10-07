import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
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

    # ----- Launch arguments -------------------------------------------------
    max_linear_velocity_arg = DeclareLaunchArgument(
        name="max_linear_velocity", default_value="0.30"
    )
    max_angular_velocity_arg = DeclareLaunchArgument(
        name="max_angular_velocity", default_value=str(0.15)
    )
    wheel_radius_arg = DeclareLaunchArgument(
        name="wheel_radius", default_value="0.03706"
    )
    track_width_arg = DeclareLaunchArgument(
        name="track_width", default_value="0.39598"
    )
    map_name_arg = DeclareLaunchArgument(name="map_name", default_value="map")

    max_linear_velocity_conf = LaunchConfiguration("max_linear_velocity")
    max_angular_velocity_conf = LaunchConfiguration("max_angular_velocity")
    wheel_radius_conf = LaunchConfiguration("wheel_radius")
    track_width_conf = LaunchConfiguration("track_width")

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

    # =========================================================
    # Nodes and Includes
    # =========================================================
    waregv_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("waregv_description"),
                "launch",
                "description.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    waregv_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("waregv_hardware"),
                "launch",
                "hardware.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[
            os.path.join(waregv_bringup_dir, "config", "twist_mux.yaml"),
            {"use_stamped": False},
        ],
        remappings=[("/cmd_vel_out", "/cmd_vel_unstamped")],
        output="screen",
    )

    waregv_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("waregv_odometry"),
                "launch",
                "odometry.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "wheel_radius": wheel_radius_conf,
        }.items(),
    )

    waregv_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("waregv_controller"),
                "launch",
                "controller.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "max_angular_velocity": max_angular_velocity_conf,
            "max_linear_velocity": max_linear_velocity_conf,
            "wheel_radius": wheel_radius_conf,
            "track_width": track_width_conf,
        }.items(),
    )

    waregv_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("waregv_mapping"),
                "launch",
                "mapping.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    waregv_suite = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("waregv_suite"), "launch", "suite.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    waregv_vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("waregv_vision"),
                "launch",
                "vision.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
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

    waregv_user_interfaces = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("waregv_user_interfaces"),
                "launch",
                "user_interfaces.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    waregv_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("waregv_navigation"),
                "launch",
                "navigation.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "max_linear_velocity": max_linear_velocity_conf,
            "max_angular_velocity": max_angular_velocity_conf,
        }.items(),
    )

    return LaunchDescription(
        [
            max_linear_velocity_arg,
            max_angular_velocity_arg,
            wheel_radius_arg,
            track_width_arg,
            map_name_arg,
            waregv_description,
            waregv_hardware,
            twist_mux_node,
            waregv_odometry,
            waregv_controller,
            waregv_mapping,
            waregv_suite,
            waregv_vision,
            rosbridge_node,
            foxglove_bridge,
            waregv_user_interfaces,
            waregv_navigation,
        ]
    )