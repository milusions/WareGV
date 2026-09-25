from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value='false')
    
    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')

    config_file_path = os.path.join(ydlidar_dir, 'params', 'X2.yaml')

    ydlidar_node  = Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
   parameters=[config_file_path, {"reversion": True, "inverted": True}],
        )
       

    imu_chasis_relay_node = Node(
            package="waregv_hardware",
            executable="imu_chasis_relay",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            output="screen",
        )
    
    
    motor_system_node = Node(
        package="waregv_hardware",
        executable="motor_system_node",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )
    
    camera_streamer = Node(
            package="waregv_hardware",
            executable="camera_streamer",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            output="screen",
        )
      
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ]),
        launch_arguments={
            'enable_accel': 'true',
            'enable_gyro': 'true',
            'unite_imu_method': '2',     
            'enable_infra1': 'true',      
            'enable_infra2': 'true',       
            'enable_sync': 'true',    
        }.items()
    )
    
    return LaunchDescription([
        use_sim_time_arg,  
        ydlidar_node,
        imu_chasis_relay_node,
        motor_system_node,
        realsense_launch,
        camera_streamer
    ])