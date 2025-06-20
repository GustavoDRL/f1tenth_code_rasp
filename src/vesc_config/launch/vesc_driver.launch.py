"""VESC Driver Launch File for F1TENTH."""
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for VESC driver."""
    config = os.path.join(
        get_package_share_directory('vesc_config'),
        'config',
        'vesc_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver',
            parameters=[config],
            output='screen'
        )
    ])