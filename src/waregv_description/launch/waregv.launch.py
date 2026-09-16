from launch import LaunchDescription
from pathlib import Path
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction, GroupAction
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import UnlessCondition, IfCondition


def generate_launch_description():
    
    waregv_description_dir = get_package_share_directory("waregv_description")
    
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory("waregv_description"), "urdf", "waregv.urdf.xacro")
    )
    use_sim_time = False
    use_sim_time_string = "false"
    mapping_check_arg = DeclareLaunchArgument(name="mapping", default_value='true')
    nav2_check_arg = DeclareLaunchArgument(name="navigation", default_value='true')
    
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":robot_description}]
    )
    
 
    joystick_node = Node(
        package="joy",
        executable="joy_node",
       
        output="screen",
    )
    
    ydlidar_node = Node(
        package="waregv_driver",
        executable="ydlidar",
   
         parameters=[{'use_sim_time': use_sim_time}],
        output="screen",
    )
    
    controller_link = Node(
        package="waregv_driver",
        executable="controller_link",
   
         parameters=[{'use_sim_time': use_sim_time}],
        output="screen",
    )
    
    motor_encoder = Node(
        package="waregv_driver",
        executable="motor_encoder",
   
         parameters=[{'use_sim_time': use_sim_time}],
        output="screen",
    )
    
    imu = Node(
        package="waregv_driver",
        executable="imu",
   
         parameters=[{'use_sim_time': use_sim_time}],
        output="screen",
    )
    
    waregv_controller = Node(
        package="waregv_controller",
        executable="joystick_control",
   
         parameters=[{'use_sim_time': use_sim_time, "max_linear_vel":0.5, "max_angular_vel":3.14}],
        output="screen",
    )
    
    waregv_odometry = Node(
        package="waregv_odometry",
        executable="odometry",
             parameters=[{'use_sim_time': use_sim_time, "wheel_radius":0.0325}],
        output="screen",
    )
    
    waregv_inverse_kinematics = Node(
        package="waregv_controller",
        executable="inverse_kinematics",
             parameters=[{'use_sim_time': use_sim_time, "wheel_radius":0.0325, "wheel_base":0.176}],
        output="screen",
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
    delayed_rviz = TimerAction(
        period=10.0,
        actions=[
            Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
          parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(LaunchConfiguration("navigation")),
        arguments=["-d", os.path.join(get_package_share_directory("waregv_description"), "rviz", "mapping.rviz")]
       
       ),
             Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration("navigation")),
        arguments=["-d", os.path.join(get_package_share_directory("waregv_description"), "rviz", "display.rviz")]
       
       )
        ])
    
    odom_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_footprint"],
            parameters=[{'use_sim_time': use_sim_time}]
       
    )
   
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    slam_launch_path = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
    
    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=os.path.join(waregv_description_dir, 'config', 'slam_toolbox.yaml')
    )
    
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        condition=IfCondition(LaunchConfiguration("mapping")),
        launch_arguments={'use_sim_time': use_sim_time_string, "slam_params_file":slam_params_file }.items() 
    )
    
    map_name = os.environ.get('MAP_NAME', 'small_warehouse')
    
    map_file = os.path.join(waregv_description_dir, "maps", map_name, f"{map_name}.yaml")
   
    nav2_params_file = os.path.join(waregv_description_dir, 'config', 'nav2_params.yaml')
    
    nav_node = GroupAction(
            actions=[
                IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("mapping"), "' == 'false' and '",
            LaunchConfiguration("navigation"), "' == 'true'"
        ])),
        launch_arguments={
            'use_sim_time': use_sim_time_string,
            'params_file': nav2_params_file,
            'map':map_file,
            'autostart': 'true',
        
        }.items()
    ),
                IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )),
             condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("mapping"), "' == 'true' and '",
            LaunchConfiguration("navigation"), "' == 'true'"
        ])),
        launch_arguments={
            'use_sim_time': use_sim_time_string,
              'params_file': nav2_params_file,
         
            'autostart': 'true',
        
        }.items()
    )
            ]
        )

    return LaunchDescription([
                              model_arg,
                              mapping_check_arg,
                              nav2_check_arg,
                              robot_state_publisher,
                              ydlidar_node,
                              controller_link,
                              motor_encoder,
                              imu,
                              odom_frame,
                              joystick_node,
                              waregv_controller,
                              waregv_odometry,
                              waregv_inverse_kinematics,
                              twist_mux_node,
                            #   delayed_rviz,
                              slam_toolbox_node,
                              nav_node
                              
                             ])
