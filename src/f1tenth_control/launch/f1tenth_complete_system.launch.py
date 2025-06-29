#!/usr/bin/env python3
"""
Launch file com parâmetros inline para garantir que sejam carregados
"""

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch with inline parameters to ensure they're loaded."""

    # Parâmetros extraídos diretamente do custom_x4.yaml
    lidar_params = {
        "port": "/dev/ydlidar",
        "frame_id": "laser_frame",
        "ignore_array": "",
        "baudrate": 128000,  # IMPORTANTE: mesmo valor do comando que funciona
        "lidar_type": 1,
        "device_type": 0,
        "sample_rate": 5,
        "abnormal_check_count": 4,
        "fixed_resolution": True,
        "reversion": True,
        "inverted": True,
        "auto_reconnect": True,
        "isSingleChannel": True,  # IMPORTANTE para o X4
        "intensity": False,
        "support_motor_dtr": False,
        "angle_max": 180.0,
        "angle_min": -180.0,
        "range_max": 12.0,
        "range_min": 0.1,
        "frequency": 10.0,
        "invalid_range_is_inf": False,
    }

    return LaunchDescription(
        [
            LogInfo(msg="Testing LiDAR with inline parameters..."),
            LogInfo(msg=f"Using baudrate: {lidar_params['baudrate']}"),
            # YDLiDAR X4 Driver com parâmetros inline
            Node(
                package="ydlidar_ros2_driver",
                executable="ydlidar_ros2_driver_node",
                name="ydlidar_ros2_driver_node",
                parameters=[lidar_params],
                output="screen",
            ),
            # Transform publisher
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_pub_laser",
                arguments=[
                    "0.0",
                    "0.0",
                    "0.02",
                    "0",
                    "0",
                    "0",
                    "base_link",
                    "laser_frame",
                ],
                output="screen",
            ),
            LogInfo(msg="LiDAR inline params test ready!"),
        ]
    )
