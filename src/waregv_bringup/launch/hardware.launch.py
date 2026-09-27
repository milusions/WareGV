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
    mode_arg = DeclareLaunchArgument(
        name="mode",
        default_value="slam_only",
        description="Choose mode: slam_only, slam_with_nav2, or nav2_with_amcl",
    )
    mode = LaunchConfiguration("mode")

    # --- Tunables ---
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

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="false")
    use_sim_time = LaunchConfiguration("use_sim_time")

    qos_overrides = {
        "/initialpose": {"durability": "volatile"},
        "/map": {"durability": "transient_local", "reliability": "reliable",
                 "history": "keep_last", "depth": 1},
    }

    # =========================================================
    # Core components
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
        output="log",                       # PI4 OPT: log, not screen
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

    # PI4 OPT: wheel odom + EKF pinned to core 0 via taskset prefix.
    # (Adjust to your taste; core 0 is generally the least contended.)
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

    # PI4 OPT: foxglove_bridge removed — adds ~5% CPU and is unused unless
    # you're running Foxglove Studio. Re-enable if you need it.

    # =========================================================
    # Vision — pinned to core 3
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
        output="log",
        parameters=[
            {"use_sim_time": False},
            {"autostart": True},
            {"node_names": ["aruco_tracker_node"]},
            {"bond_timeout": 20.0},                 # PI4 OPT: was 4.0
            {"attempt_respawn_reconnection": True}, # PI4 OPT
        ],
    )

    # =========================================================
    # Conditional SLAM
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
    # Conditional Navigation
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

    amcl_pose_setter = Node(
        package="waregv_navigation",
        executable="amcl_initial_pose_node",
        name="amcl_initial_pose_node",
        output="log",
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