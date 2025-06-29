#!/usr/bin/env python3
"""
Launch file simplificado - apenas LiDAR para teste
"""

from launch import LaunchDescription
from launch.actions import LogInfo
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate simplified launch for LiDAR testing only."""

    # Configuração do LiDAR (mesma que funciona)
    lidar_config = PathJoinSubstitution(
        [FindPackageShare("ydlidar_ros2_driver"), "params", "custom_x4.yaml"]
    )

    return LaunchDescription(
        [
            LogInfo(msg="Testing LiDAR only..."),
            # YDLiDAR X4 Driver
            Node(
                package="ydlidar_ros2_driver",
                executable="ydlidar_ros2_driver_node",
                name="ydlidar_node",
                parameters=[lidar_config],
                output="screen",
                respawn=True,
                respawn_delay=3.0,
            ),
            # Transform publisher (mesmo do comando que funciona)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser_tf",
                arguments=[
                    "0.0",
                    "0.0",
                    "0.02",  # x, y, z (mesmos valores do comando que funciona)
                    "0",
                    "0",
                    "0",  # roll, pitch, yaw
                    "base_link",
                    "laser_frame",
                ],
                output="screen",
            ),
            LogInfo(msg="LiDAR test ready!"),
        ]
    )
