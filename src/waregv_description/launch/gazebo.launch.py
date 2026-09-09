from launch import LaunchDescription
from pathlib import Path
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    waregv_description_dir = get_package_share_directory("waregv_description")
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"
    
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
       
        output="screen",
    )
  
    return LaunchDescription([model_arg, robot_state_publisher, gazebo_resource_path, gazebo, gz_spawn_entity, gz_ros_bridge, spawn_joint_state_broadcaster,spawn_velocity_controller,joystick_node])
