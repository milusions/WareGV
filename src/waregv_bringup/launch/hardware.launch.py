import json
import os
import numpy as np

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources.frontend_launch_description_source import (
    FrontendLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def _banner(stage_num, name):
    """Uniform stage banner."""
    return LogInfo(
        msg=f"\n============================================================\n"
            f"  [STAGE {stage_num}] {name}\n"
            f"============================================================"
    )


def _wait_for_topic(topic, timeout=180.0):
    """
    Blocking readiness probe: exits 0 once `topic` has emitted at least one message.
    Uses default (reliable) QoS — match this to the publisher.
    """
    return ExecuteProcess(
        cmd=["ros2", "topic", "echo", "--once", topic],
        output="log",
        name=f"wait_for_{topic.strip('/').replace('/', '_')}",
    )


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
    # STAGE 1 — Description & Hardware drivers
    # =========================================================
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

    # =========================================================
    # STAGE 2 — Hardware sensor readiness barrier
    # =========================================================
    joint_states_gate = _wait_for_topic("/joint_states", timeout=120.0)

    # =========================================================
    # STAGE 3 — /tf readiness barrier
    # =========================================================
    tf_gate = _wait_for_topic("/tf", timeout=120.0)

    # =========================================================
    # STAGE 4 — Robot-side autonomy (twist_mux, odometry, controller)
    # =========================================================
    twist_mux_node_config_filepath = os.path.join(
        waregv_bringup_dir, "config", "twist_mux.yaml"
    )
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[twist_mux_node_config_filepath, {"use_stamped": False}],
        remappings=[("/cmd_vel_out", "/cmd_vel_unstamped")],
        output="screen",
    )

    waregv_odometry_launch_file_path = os.path.join(
        get_package_share_directory("waregv_odometry"), "launch", "odometry.launch.py"
    )
    waregv_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waregv_odometry_launch_file_path),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "wheel_radius": wheel_radius_conf,
        }.items(),
    )

    waregv_controller_launch_file_path = os.path.join(
        get_package_share_directory("waregv_controller"), "launch", "controller.launch.py"
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

    # =========================================================
    # STAGE 5 — Mapping (Loaded first for CPU priority and stability)
    # =========================================================
    waregv_mapping_launch_file_path = os.path.join(
        get_package_share_directory("waregv_mapping"), "launch", "mapping.launch.py"
    )
    waregv_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waregv_mapping_launch_file_path),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # =========================================================
    # STAGE 6 — Suite & Vision
    # =========================================================
    waregv_suite_launch_file_path = os.path.join(
        get_package_share_directory("waregv_suite"), "launch", "suite.launch.py"
    )
    waregv_suite = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waregv_suite_launch_file_path),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    waregv_vision_launch_file_path = os.path.join(
        get_package_share_directory("waregv_vision"), "launch", "vision.launch.py"
    )
    waregv_vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waregv_vision_launch_file_path),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # =========================================================
    # STAGE 7 — Bridges & User Interfaces
    # =========================================================
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
                        [FindPackageShare("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml"]
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

    waregv_user_interfaces_launch_file_path = os.path.join(
        get_package_share_directory("waregv_user_interfaces"), "launch", "user_interfaces.launch.py"
    )
    waregv_user_interfaces = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waregv_user_interfaces_launch_file_path),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # =========================================================
    # STAGE 8 — Navigation (Loaded last)
    # =========================================================
    waregv_navigation_launch_file_path = os.path.join(
        get_package_share_directory("waregv_navigation"), "launch", "navigation.launch.py"
    )
    waregv_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waregv_navigation_launch_file_path),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "max_linear_velocity": max_linear_velocity_conf,
            "max_angular_velocity": max_angular_velocity_conf,
        }.items(),
    )

    # =========================================================
    # Event-driven sequencing
    # =========================================================
    start_stage3 = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_states_gate,
            on_exit=[
                _banner(3, "/tf readiness barrier"),
                tf_gate,
            ],
        )
    )

    start_stage4 = RegisterEventHandler(
        OnProcessExit(
            target_action=tf_gate,
            on_exit=[
                _banner(4, "Robot-side autonomy (twist_mux, odometry, controller)"),
                twist_mux_node,
                waregv_odometry,
                waregv_controller,
                TimerAction(
                    period=3.0,
                    actions=[
                        _banner(5, "Mapping (isolated for CPU stability)"),
                        waregv_mapping,
                    ],
                ),
                TimerAction(
                    period=8.0,
                    actions=[
                        _banner(6, "Suite & vision"),
                        waregv_suite,
                        waregv_vision,
                    ],
                ),
                TimerAction(
                    period=13.0,
                    actions=[
                        _banner(7, "Bridges & user interfaces"),
                        rosbridge_node,
                        foxglove_bridge,
                        waregv_user_interfaces,
                    ],
                ),
                TimerAction(
                    period=18.0,
                    actions=[
                        _banner(8, "Navigation"),
                        waregv_navigation,
                    ],
                ),
            ],
        )
    )

    return LaunchDescription(
        [
            max_linear_velocity_arg,
            max_angular_velocity_arg,
            wheel_radius_arg,
            track_width_arg,
            map_name_arg,
            _banner(1, "Descriptions & hardware drivers"),
            waregv_description,
            waregv_hardware,
            _banner(2, "Hardware sensor readiness barrier (/joint_states)"),
            joint_states_gate,
            start_stage3,
            start_stage4,
        ]
    )