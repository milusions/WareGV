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
    Uses default (reliable) QoS — match this to the publisher; don't force best_effort.
    """
    return ExecuteProcess(
        cmd=["ros2", "topic", "echo", "--once", topic],
        output="log",
        name=f"wait_for_{topic.strip('/').replace('/', '_')}",
    )


def _wait_for_service(service, timeout=180.0):
    """
    Blocking readiness probe: polls `ros2 service list` until `service` appears.
    Exits 0 on success, 1 on timeout.
    """
    svc = service.strip("/")
    # Poll every 0.5s up to `timeout` seconds.
    script = (
        f"end=$(($(date +%s) + {int(timeout)})); "
        f"while [ $(date +%s) -lt $end ]; do "
        f"  if ros2 service list 2>/dev/null | grep -qx '/{svc}'; then "
        f"    echo 'service /{svc} is available'; exit 0; "
        f"  fi; "
        f"  sleep 0.5; "
        f"done; "
        f"echo 'timeout waiting for /{svc}'; exit 1"
    )
    return ExecuteProcess(
        cmd=["bash", "-c", script],
        output="log",
        name=f"wait_for_service_{svc.replace('/', '_')}",
    )


def generate_launch_description():
    waregv_bringup_dir = get_package_share_directory("waregv_bringup")
    rosbridge_dir = get_package_share_directory("rosbridge_server")

    # ----- Launch arguments -------------------------------------------------
    world_name_arg = DeclareLaunchArgument("world_name", default_value="small_warehouse")
    max_linear_velocity_arg = DeclareLaunchArgument("max_linear_velocity", default_value="0.11")
    max_angular_velocity_arg = DeclareLaunchArgument("max_angular_velocity", default_value="0.35")
    wheel_radius_arg = DeclareLaunchArgument("wheel_radius", default_value="0.035")
    track_width_arg = DeclareLaunchArgument("track_width", default_value="0.168")
    model_arg = DeclareLaunchArgument("model", default_value="waregv.urdf.xacro")
    robot_spawn_z_arg = DeclareLaunchArgument("spawn_z", default_value="0.5")

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

    # =========================================================
    # STAGE 1 — Descriptions & World
    # =========================================================
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
        launch_arguments={"world_name": world_name_conf, "use_sim_time": use_sim_time}.items(),
    )

    # =========================================================
    # STAGE 2 — Spawn entity (gated on /clock being published)
    # =========================================================
    clock_gate = _wait_for_topic("/clock", timeout=180.0)

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        name="gz_spawn_entity",
        arguments=[
            "-world", world_name_conf,
            "-topic", "robot_description",
            "-name", "waregv",
            "-z", robot_spawn_z,
        ],
    )

    # =========================================================
    # STAGE 3 — controller_manager service readiness gate
    # (was /tf — that was a circular dependency, see notes)
    # =========================================================
    controller_manager_gate = _wait_for_service(
        "/controller_manager/list_controllers", timeout=180.0
    )

    # =========================================================
    # STAGE 4 — Controllers (spawners)
    # =========================================================
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_velocity_controller",
        arguments=[
            "velocity_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # =========================================================
    # STAGE 5 — Robot-side autonomy (gated on /tf being alive,
    # which now only happens after joint_state_broadcaster runs)
    # =========================================================
    tf_gate = _wait_for_topic("/tf", timeout=180.0)

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
    # STAGE 6 — High-level autonomy (mapping, nav, suite)
    # =========================================================
    waregv_mapping_launch_file_path = os.path.join(
        get_package_share_directory("waregv_mapping"), "launch", "mapping.launch.py"
    )
    waregv_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waregv_mapping_launch_file_path),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

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

    waregv_suite_launch_file_path = os.path.join(
        get_package_share_directory("waregv_suite"), "launch", "suite.launch.py"
    )
    waregv_suite = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waregv_suite_launch_file_path),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # =========================================================
    # STAGE 7 — Bridges / UIs
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
    # Event-driven sequencing
    # =========================================================
    # Stage 1 -> 2: /clock is published by the parameter_bridge once Gazebo is ticking
    start_stage2 = RegisterEventHandler(
        OnProcessExit(
            target_action=clock_gate,
            on_exit=[
                _banner(2, "Spawn robot into Gazebo"),
                gz_spawn_entity,
            ],
        )
    )

    # Stage 2 -> 3: after spawn exits, wait for controller_manager services
    start_stage3 = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[
                _banner(3, "Controller manager service readiness barrier"),
                controller_manager_gate,
            ],
        )
    )

    # Stage 3 -> 4: spawners fire once controller_manager services are live
    start_stage4 = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_manager_gate,
            on_exit=[
                _banner(4, "Spawn controller_manager controllers"),
                joint_state_broadcaster_spawner,
                velocity_controller_spawner,
            ],
        )
    )

    # Stage 4 -> 5: /tf now genuinely exists (JSB is active -> /joint_states -> RSP -> /tf)
    start_stage5 = RegisterEventHandler(
        OnProcessExit(
            target_action=velocity_controller_spawner,
            on_exit=[
                _banner(5, "Wait for /tf, then start robot-side autonomy"),
                tf_gate,
            ],
        )
    )

    # Stage 5 body: fires once /tf has been observed
    start_stage5_body = RegisterEventHandler(
        OnProcessExit(
            target_action=tf_gate,
            on_exit=[
                LogInfo(msg="[STAGE 5] /tf is alive — starting twist_mux, odometry, controller"),
                twist_mux_node,
                waregv_odometry,
                waregv_controller,
                TimerAction(
                    period=5.0,
                    actions=[
                        _banner(6, "High-level autonomy (mapping, navigation, suite)"),
                        waregv_mapping,
                        waregv_navigation,
                        waregv_suite,
                    ],
                ),
                TimerAction(
                    period=10.0,
                    actions=[
                        _banner(7, "Bridges & user interfaces"),
                        rosbridge_node,
                        foxglove_bridge,
                        waregv_user_interfaces,
                    ],
                ),
            ],
        )
    )

    return LaunchDescription(
        [
            # Arguments
            robot_spawn_z_arg,
            world_name_arg,
            max_linear_velocity_arg,
            max_angular_velocity_arg,
            wheel_radius_arg,
            track_width_arg,
            model_arg,

            # Stage 1
            _banner(1, "Descriptions & Gazebo world"),
            waregv_urdf,
            gazebo_sim,

            # Stage 1 -> 2
            clock_gate,
            start_stage2,

            # Stage 2 -> 3
            start_stage3,

            # Stage 3 -> 4
            start_stage4,

            # Stage 4 -> 5 (two chained handlers: spawner exit -> tf gate -> stage 5 body)
            start_stage5,
            start_stage5_body,
        ]
    )