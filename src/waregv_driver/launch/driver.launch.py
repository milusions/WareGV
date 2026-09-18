from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions.launch_configuration import LaunchConfiguration


def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value='true')
    
    ydlidar_node = Node(
        package="waregv_driver",
        executable="ydlidar",
   
         parameters=[{"angle_offset_deg:":-25.0,'use_sim_time': LaunchConfiguration("use_sim_time")}],
        output="screen",
    )
    
    controller_link = Node(
        package="waregv_driver",
        executable="controller_link",
   
         parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        output="screen",
    )
    
    motor_encoder = Node(
        package="waregv_driver",
        executable="motor_encoder",
   
         parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        output="screen",
    )
    
    imu = Node(
        package="waregv_driver",
        executable="imu",
   
         parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        output="screen",
    )
    
    camera = Node(
        package="waregv_driver",
        executable="camera",
   
         parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    return LaunchDescription([
                              ydlidar_node,
                              controller_link,
                              motor_encoder,
                              imu,
                              camera
                              
                             ])
