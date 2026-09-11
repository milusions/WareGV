from launch import LaunchDescription
from pathlib import Path
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    waregv_description_dir = get_package_share_directory("waregv_description")
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"
    
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch_path = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
    
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory("waregv_description"), "urdf", "waregv.urdf.xacro")
    )
    
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model"), " is_ignition:=", is_ignition]), value_type=str)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":robot_description}]
    )
    gazebo_resource_path = SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=[str(Path(waregv_description_dir).parent.resolve())])
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            
            os.path.join(
               get_package_share_directory("ros_gz_sim"),
                "launch"
            ), "/gz_sim.launch.py"
        ]),
        launch_arguments=[
            ("gz_args", " -v 4 -r empty.sdf")
        ]
    )
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-world", "empty", "-topic", "robot_description", "-name", "waregv"]
        
    )
    
    gz_ros_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
      parameters=[{
            "config_file": os.path.join(
                get_package_share_directory("waregv_description"),
                "config",
                "bridge.yaml"
            )
        }]
    )
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
   
    spawn_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    joystick_node = Node(
        package="joy",
        executable="joy_node",
       
        output="screen",
    )
    
    waregv_controller = Node(
        package="waregv_controller",
        executable="joystick_control",
        parameters=[{'use_sim_time': True}],
        output="screen",
    )
    
    waregv_odometry = Node(
        package="waregv_odometry",
        executable="odometry",
             parameters=[{'use_sim_time': True}],
        output="screen",
    )
    
    waregv_inverse_kinematics = Node(
        package="waregv_controller",
        executable="inverse_kinematics",
             parameters=[{'use_sim_time': True}],
        output="screen",
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("waregv_description"), "rviz", "display.rviz")]
       
    )
    
    odom_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_footprint"],
            parameters=[{'use_sim_time': True}]
       
    )
   
    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=os.path.join(waregv_description_dir, 'config', 'slam_toolbox.yaml')
    )
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
    
        launch_arguments={'use_sim_time': 'true', "slam_params_file":slam_params_file  }.items() 
    )
    nav2_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )
    
    nav2_params_file = os.path.join(waregv_description_dir, 'config', 'nav2_params.yaml')
    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file,
            'autostart': 'true'
        }.items()
    )
    
    twist_mux_node_config_filepath = os.path.join(
        waregv_description_dir,
        'config',
        'twist_mux.yaml'
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_node_config_filepath, {"use_stamped":False}],
        remappings=[
      
            ('/cmd_vel_out', '/cmd_vel_unstamped')
        ]
    )
    return LaunchDescription([model_arg,
                              robot_state_publisher,
                              gazebo_resource_path,
                              gazebo,
                              gz_spawn_entity,
                              gz_ros_bridge,
                              spawn_joint_state_broadcaster,
                              spawn_velocity_controller,
                               odom_frame,
                              joystick_node,
                              waregv_controller,
                              waregv_odometry,
                              waregv_inverse_kinematics,
                              twist_mux_node,
                             
                              slam_toolbox_node,
                              nav2_node,
                              rviz2,
                             ])
