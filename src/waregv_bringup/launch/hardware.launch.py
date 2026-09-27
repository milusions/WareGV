import json
import os
import numpy as np

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources.frontend_launch_description_source import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression


def generate_launch_description():
    waregv_bringup_dir = get_package_share_directory("waregv_bringup")
    rosbridge_dir = get_package_share_directory("rosbridge_server")

    # --- Mode selector ---
    # One of: slam_only, slam_with_nav2, nav2_with_amcl
    # Default matches the dashboard's default so a bare `ros2 launch` and a
    # ModeSwitcher deploy end up in the same state.
    mode_arg = DeclareLaunchArgument(
        name="mode",
        default_value="slam_only",
        description="Choose mode: slam_only, slam_with_nav2, or nav2_with_amcl",
    )
    mode = LaunchConfiguration("mode")

    # --- Tunables (unchanged) ---
    max_linear_velocity_arg = DeclareLaunchArgument(name="max_linear_velocity", default_value="0.11")
    max_angular_velocity_arg = DeclareLaunchArgument(name="max_angular_velocity", default_value="0.35")
    wheel_radius_arg = DeclareLaunchArgument(name="wheel_radius", default_value="0.036")
    track_width_arg = DeclareLaunchArgument(name="track_width", default_value="0.192")
    map_name_arg = DeclareLaunchArgument(name="map_name", default_value="map")

    max_linear_velocity_conf = LaunchConfiguration("max_linear_velocity")
    max_angular_velocity_conf = LaunchConfiguration("max_angular_velocity")
    wheel_radius_conf = LaunchConfiguration("wheel_radius")
    track_width_conf = LaunchConfiguration("track_width")
    map_name_conf = LaunchConfiguration("map_name")

    # --- Sim time as a launch argument so ModeSwitcher can override it ---
    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="false")
    use_sim_time = LaunchConfiguration("use_sim_time")

    qos_overrides = {
        "/initialpose": {"durability": "volatile"},
        "/map": {"durability": "transient_local", "reliability": "reliable",
                 "history": "keep_last", "depth": 1},
    }

    # =========================================================
    # Always-on core components (run in every mode)
    # =========================================================
    waregv_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("waregv_description"),
                         "launch", "description.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    waregv_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("waregv_hardware"),
                         "launch", "hardware.launch.py")
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

    waregv_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("waregv_controller"),
                         "launch", "controller.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "max_angular_velocity": max_angular_velocity_conf,
            "max_linear_velocity": max_linear_velocity_conf,
            "wheel_radius": wheel_radius_conf,
            "track_width": track_width_conf,
        }.items(),
    )

    waregv_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("waregv_odometry"),
                         "launch", "odometry.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "wheel_radius": wheel_radius_conf,
        }.items(),
    )

    waregv_suite = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("waregv_suite"),
                         "launch", "suite.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    waregv_user_interfaces = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("waregv_user_interfaces"),
                         "launch", "user_interfaces.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    rosbridge_node = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(rosbridge_dir, "launch", "rosbridge_websocket_launch.xml")
        ),
        launch_arguments={"port": "9090", "ssl": "false", "output": "log"}.items(),
    )

    foxglove_bridge = GroupAction(
        actions=[
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("foxglove_bridge"), "launch",
                         "foxglove_bridge_launch.xml"]
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

    # =========================================================
    # Vision & Aruco lifecycle auto-start (all modes)
    # =========================================================
    waregv_vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("waregv_vision"),
                         "launch", "vision.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    aruco_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_aruco",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"autostart": True},
            {"node_names": ["aruco_tracker_node"]},
        ],
    )

    # =========================================================
    # Conditional SLAM (slam_only + slam_with_nav2)
    # =========================================================
    waregv_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("waregv_mapping"),
                         "launch", "mapping.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(
            PythonExpression(["'", mode, "' in ['slam_only', 'slam_with_nav2']"])
        ),
    )

    # =========================================================
    # Conditional Navigation (slam_with_nav2 + nav2_with_amcl)
    # =========================================================
    waregv_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("waregv_navigation"),
                         "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "max_linear_velocity": max_linear_velocity_conf,
            "max_angular_velocity": max_angular_velocity_conf,
            "mode": mode,
            "map_name": map_name_conf,
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", mode, "' in ['slam_with_nav2', 'nav2_with_amcl']"])
        ),
    )

    # =========================================================
    # AMCL initial-pose setter — only when we have a real map name
    # =========================================================
    # The previous condition only checked the mode. If the dashboard ever
    # launched nav2_with_amcl with an empty map_name (e.g. a bad manual
    # invocation), this node would publish a bogus initial pose into an
    # unloaded map_server. Gate on both the mode and a non-empty map name.
    amcl_pose_setter = Node(
        package="waregv_navigation",
        executable="amcl_initial_pose_node",
        name="amcl_initial_pose_node",
        output="screen",
        condition=IfCondition(
            PythonExpression([
                "'", mode, "' == 'nav2_with_amcl'",
                " and '", map_name_conf, "' not in ['', 'map']",
            ])
        ),
    )

    return LaunchDescription([
        mode_arg,
        max_linear_velocity_arg,
        max_angular_velocity_arg,
        wheel_radius_arg,
        track_width_arg,
        map_name_arg,
        use_sim_time_arg,

        # Core
        waregv_description,
        rosbridge_node,
        foxglove_bridge,
        waregv_user_interfaces,
        waregv_hardware,
        twist_mux_node,
        waregv_odometry,
        waregv_controller,
        waregv_suite,

        # Vision
        waregv_vision,
        aruco_lifecycle_manager,

        # Conditional
        waregv_mapping,
        waregv_navigation,
        amcl_pose_setter,
    ])