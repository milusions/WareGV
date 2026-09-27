import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources.frontend_launch_description_source import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression


def generate_launch_description():
    waregv_bringup_dir = get_package_share_directory("waregv_bringup")
    rosbridge_dir = get_package_share_directory("rosbridge_server")

    mode_arg = DeclareLaunchArgument(
        name="mode",
        default_value="slam_only",
        description="slam_only | slam_with_nav2 | nav2_with_amcl",
    )
    mode = LaunchConfiguration("mode")

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

    # ---- Core (always) ----
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
        output="log",
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

    # ---- SLAM: slam_only + slam_with_nav2 ----
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

    # ---- Nav2: slam_with_nav2 + nav2_with_amcl ----
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
        max_linear_velocity_arg,
        max_angular_velocity_arg,
        wheel_radius_arg,
        track_width_arg,
        map_name_arg,
        use_sim_time_arg,

        waregv_description,
        rosbridge_node,
        waregv_user_interfaces,
        waregv_hardware,
        twist_mux_node,
        waregv_odometry,
        waregv_controller,
        waregv_mapping,
        waregv_navigation,
    ])