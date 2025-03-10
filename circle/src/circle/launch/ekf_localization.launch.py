from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    ekf_config_path = os.path.join(
        get_package_share_directory('circle'), 'config', 'ekf_imu.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    return LaunchDescription([ekf_node])
