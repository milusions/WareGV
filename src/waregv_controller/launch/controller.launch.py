from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions.declare_launch_argument import DeclareLaunchArgument
import numpy as np
from launch.substitutions.launch_configuration import LaunchConfiguration


def generate_launch_description():
    
    max_linear_velocity_arg = DeclareLaunchArgument(name="max_linear_velocity", default_value="0.5")
    max_angular_velocity_arg = DeclareLaunchArgument(name="max_angular_velocity", default_value=str(np.pi))
    wheel_radius_arg = DeclareLaunchArgument(name="wheel_radius", default_value='0.0325')
    wheel_base_arg = DeclareLaunchArgument(name="wheel_base", default_value='0.176')
    
    
    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value='true')
    
    joystick_node = Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration("use_sim_time"),
                'dev': '/dev/input/js0',
                'deadzone': 0.05,
                'autorepeat_rate': 20.0
            }]
        )
    
    waregv_controller = Node(
        package="waregv_controller",
        executable="joystick_control",
         parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time"), "max_linear_vel":LaunchConfiguration('max_linear_velocity'), "max_angular_vel":LaunchConfiguration('max_angular_velocity')}],
        output="screen",
    )
    
    waregv_inverse_kinematics = Node(
        package="waregv_controller",
        executable="inverse_kinematics",
         parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time"), "wheel_radius":LaunchConfiguration('wheel_radius'), "wheel_base":LaunchConfiguration('wheel_base')}],
        output="screen",
    )

    return LaunchDescription([
                                use_sim_time_arg,
                                wheel_radius_arg,
                                wheel_base_arg,
                            max_linear_velocity_arg,
                            max_angular_velocity_arg,
                              joystick_node,
                              waregv_controller,
                              waregv_inverse_kinematics
                              
                             ])
