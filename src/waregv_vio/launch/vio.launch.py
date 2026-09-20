import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('waregv_vio'),
        'config',
        'rs_d435i',
        'estimator_config.yaml'
    )

    ov_node = Node(
        package='ov_msckf',
        executable='run_subscribe_msckf',
        name='run_subscribe_msckf',
        output='screen',
        parameters=[{
            'config_path': config_path,
            'verbosity': 'INFO',
            'publish_tf': True,
        }]
    )

    return LaunchDescription([ov_node])