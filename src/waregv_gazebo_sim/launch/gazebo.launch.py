from launch import LaunchDescription
from pathlib import Path
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import  LaunchConfiguration
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions.python_expression import PythonExpression
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions.node import Node



def generate_launch_description():
    
    waregv_gazebo_sim_dir = get_package_share_directory("waregv_gazebo_sim")
    waregv_description_dir = get_package_share_directory("waregv_description")
    
    world_name_arg = DeclareLaunchArgument("world_name",default_value="small_warehouse")
    
    models_path = os.path.join(waregv_gazebo_sim_dir, "models")
    
    gazebo_resource_path = SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=[str(Path(waregv_description_dir).parent.resolve()),":",str(Path(waregv_gazebo_sim_dir).parent.resolve()), ":", models_path, ":", waregv_gazebo_sim_dir])
    
  
    
    world_path = PathJoinSubstitution([waregv_gazebo_sim_dir, "worlds", LaunchConfiguration('world_name'), [LaunchConfiguration('world_name'),".world"]])
  
    ros_gz_brige_config_file = os.path.join(waregv_gazebo_sim_dir, "config", "bridge.yaml")
    
    gz_ros_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"config_file": ros_gz_brige_config_file}]
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            
            os.path.join(
               get_package_share_directory("ros_gz_sim"),
                "launch"
            ), "/gz_sim.launch.py"
        ]),
        launch_arguments=[
            ("gz_args", [" -v 4 -r ", world_path])
        ]
    )
  

    return LaunchDescription([
        world_name_arg,
                              gazebo_resource_path,
                              gazebo,
                              gz_ros_bridge
                             ])
