import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    
    waregv_gazebo_sim_dir = get_package_share_directory("waregv_gazebo_sim")
    waregv_description_dir = get_package_share_directory("waregv_description")
    
    world_name_arg = DeclareLaunchArgument("world_name", default_value="small_warehouse")
    world_name_conf = LaunchConfiguration("world_name")
    
    models_path = os.path.join(waregv_gazebo_sim_dir, "models")
    
    resource_paths = os.path.pathsep.join([
        str(Path(waregv_description_dir).parent.resolve()),
        str(Path(waregv_gazebo_sim_dir).parent.resolve()),
        models_path,
        waregv_gazebo_sim_dir
    ])
    gazebo_resource_path = SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=resource_paths)
    
    # Correct string concatenation for world file name
    world_filename = PythonExpression(["'", world_name_conf, ".world'"])
    
    world_path = PathJoinSubstitution([
        waregv_gazebo_sim_dir, 
        "worlds", 
        world_name_conf, 
        world_filename
    ])
  
    ros_gz_bridge_config_file = os.path.join(waregv_gazebo_sim_dir, "config", "bridge.yaml")
    
    gz_ros_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"config_file": ros_gz_bridge_config_file, "use_sim_time": True}]
    )
    
    # Proper string argument concatenation for gz_args
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-v 4 -r ", world_path]
        }.items()
    )

    return LaunchDescription([
        world_name_arg,
        gazebo_resource_path,
        gazebo,
        gz_ros_bridge
    ])