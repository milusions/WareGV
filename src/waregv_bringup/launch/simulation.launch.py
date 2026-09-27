import json
import os

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
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression


def generate_launch_description():
    waregv_bringup_dir = get_package_share_directory("waregv_bringup")
    rosbridge_dir = get_package_share_directory("rosbridge_server")

    # ---- Launch arguments ----
    mode_arg = DeclareLaunchArgument(
        name="mode",
        default_value="slam_only",
        description="slam_only | slam_with_nav2 | nav2_with_amcl",
    )
    mode = LaunchConfiguration("mode")

    world_name_arg = DeclareLaunchArgument(
        "world_name", default_value="small_warehouse"
    )
    map_name_arg = DeclareLaunchArgument(
        name="map_name", default_value="map",
        description="Loaded map name — used only in nav2_with_amcl mode.",
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
    map_name_conf = LaunchConfiguration("map_name")

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

    # ---- Robot description ----
    waregv_urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("waregv_description"),
                         "launch", "description.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time, "model": model_conf}.items(),
    )

    # ---- Gazebo ----
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("waregv_gazebo_sim"),
                         "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world_name": world_name_conf,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world", world_name_conf,
            "-topic", "robot_description",
            "-name", "waregv",
            "-z", robot_spawn_z,
        ],
    )

    controller_spawners = GroupAction(
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "60",
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
                    "--controller-manager-timeout", "60",
                ],
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
        ]
    )

    # ---- Twist mux ----
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[
            os.path.join(waregv_bringup_dir, "config", "twist_mux.yaml"),
            {"use_stamped": False},
        ],
        remappings=[("/cmd_vel_out", "/cmd_vel_unstamped")],
    )

    # ---- Odometry ----
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

    # ---- Controller ----
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

    # ---- rosbridge (for the dashboard) ----
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

    # ---- User interfaces ----
    waregv_user_interfaces = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("waregv_user_interfaces"),
                         "launch", "user_interfaces.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ---- SLAM — slam_only + slam_with_nav2 ----
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

    # ---- Nav2 — slam_with_nav2 + nav2_with_amcl ----
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

    return LaunchDescription([
        mode_arg,
        map_name_arg,
        robot_spawn_z_arg,
        world_name_arg,
        max_linear_velocity_arg,
        max_angular_velocity_arg,
        wheel_radius_arg,
        track_width_arg,
        model_arg,

        # Robot + Gazebo
        waregv_urdf,
        gazebo_sim,
        gz_spawn_entity,
        controller_spawners,

        # Control chain
        twist_mux_node,
        waregv_odometry,
        waregv_controller,

        # UI + bridge
        rosbridge_node,
        waregv_user_interfaces,

        # Mode-conditional
        waregv_mapping,
        waregv_navigation,
    ])