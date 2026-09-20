from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value='false')
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    ydlidar_node = Node(
        package="waregv_driver",
        executable="ydlidar",
        parameters=[{
            "reverse_direction": True,
            "angle_offset_deg": 25.0,
            "use_sim_time": use_sim_time
        }],
        output="screen",
    )
    
    controller_link = Node(
        package="waregv_driver",
        executable="controller_link",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )
    
    motor_encoder = Node(
        package="waregv_driver",
        executable="motor_encoder",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )
    
    imu = Node(
        package="waregv_driver",
        executable="imu",
        parameters=[{"use_sim_time": use_sim_time}],
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
            'unite_imu_method': '2',        # Interpolate accel & gyro into single /camera/camera/imu topic
            'enable_infra1': 'true',       # Left infrared camera for stereo VIO
            'enable_infra2': 'true',       # Right infrared camera for stereo VIO
            'enable_sync': 'true',         # Hardware timestamp sync
        }.items()
    )
    
    return LaunchDescription([
        use_sim_time_arg,  
        ydlidar_node,
        # controller_link,
        # motor_encoder,
        # imu,
        realsense_launch
    ])