import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value='false',
        description="Use simulation (Gazebo) clock if true"
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")

    urdf_file_path = PathJoinSubstitution([
        get_package_share_directory("waregv_description"),
        "urdf",
        "waregv.urdf.xacro"
    ])
    
    controllers_yaml_path = PathJoinSubstitution([
        get_package_share_directory("waregv_description"),
        "config",
        "controller.yaml"
    ])
    
    # Process Xacro into URDF string
    robot_description = ParameterValue(
        Command(["xacro ", urdf_file_path]),
        value_type=str
    )

    # 1. Robot State Publisher (Publishes /robot_description topic)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time
        }]
    ) 

    # 2. ros2_control_node (Hardware Interface + Controller Manager)
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time}, # ADDED: Match sim time parameter
            controllers_yaml_path
        ],
        output='screen'
    )

    # 3. Spawner node for joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # 4. Spawner node for diff_drive_controller
    diff_drive_broadcaster_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=[
        'differential_drive_controller',
        '--ros-args',
        '-r', '/differential_drive_controller/cmd_vel_unstamped:=/cmd_vel_unstamped',
        '-r', '/odom:=/odom/wheel'
    ],
    output='screen'
)

    # 5. Delay spawning joint_state_broadcaster until control_node is started
    delay_joint_state_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    # 6. Delay spawning diff_drive_controller until joint_state_broadcaster is started
    delay_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[diff_drive_broadcaster_spawner],
        )
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
        # control_node,
        # delay_joint_state_spawner,
        # delay_diff_drive_spawner
    ])