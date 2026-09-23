from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Node definitions
    headlight_node = Node(
        package='waregv_user_interfaces',
        executable='headlight_node',
        name='headlight_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    nav2_status_node = Node(
        package='waregv_user_interfaces',
        executable='nav2_status_node',
        name='nav2_status_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    sensor_health_node = Node(
        package='waregv_user_interfaces',
        executable='sensor_health_node',
        name='sensor_health_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    tts_node = Node(
        package='waregv_user_interfaces',
        executable='tts_node',
        name='tts_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        headlight_node,
        nav2_status_node,
        sensor_health_node,
        tts_node,
    ])