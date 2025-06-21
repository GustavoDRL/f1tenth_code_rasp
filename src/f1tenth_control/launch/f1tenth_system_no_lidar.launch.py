#!/usr/bin/env python3
"""
F1TENTH System (No LiDAR) Launch File
====================================
Sistema híbrido: VESC + Servo + Teclado (SEM LiDAR)
Para Raspberry Pi 4B com controle manual por teclado

Componentes:
- VESC Motor Controller (USB)
- Servo Steering (GPIO PWM)
- Keyboard Control Interface
- Real-time Odometry & TF

Uso:
    ros2 launch f1tenth_control f1tenth_system_no_lidar.launch.py

Controles de Teclado:
    W/S: Acelerar/Frear
    A/D: Esquerda/Direita
    Espaço: Parar
    Q: Sair
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate F1TENTH system without LiDAR launch description."""

    # ==========================================================================
    # LAUNCH ARGUMENTS
    # ==========================================================================
    declared_arguments = [
        DeclareLaunchArgument(
            "namespace", default_value="ego_racecar", description="Robot namespace"
        ),
        DeclareLaunchArgument(
            "debug_mode", default_value="false", description="Enable debug logging"
        ),
    ]

    # ==========================================================================
    # CONFIGURATION FILES
    # ==========================================================================
    control_config = PathJoinSubstitution(
        [FindPackageShare("f1tenth_control"), "config", "system_config.yaml"]
    )

    vesc_config = PathJoinSubstitution(
        [FindPackageShare("vesc_config"), "config", "vesc_config.yaml"]
    )

    # ==========================================================================
    # GLOBAL PARAMETERS
    # ==========================================================================
    global_parameters = [
        SetParameter(name="use_sim_time", value=False),
        SetParameter(name="robot_namespace", value=LaunchConfiguration("namespace")),
    ]

    # ==========================================================================
    # HARDWARE DRIVERS
    # ==========================================================================
    hardware_drivers = GroupAction(
        [
            LogInfo(msg="Launching F1TENTH hardware drivers (no LiDAR)..."),
            # VESC Motor Controller Driver
            Node(
                package="vesc_driver",
                executable="vesc_driver_node",
                name="vesc_driver",
                parameters=[vesc_config],
                output="screen",
                respawn=True,
                respawn_delay=2.0,
            ),
            # Servo Control Node (GPIO PWM)
            Node(
                package="f1tenth_control",
                executable="servo_control_node",
                name="servo_control_node",
                namespace=LaunchConfiguration("namespace"),
                parameters=[control_config],
                output="screen",
                emulate_tty=True,
                respawn=True,
                respawn_delay=2.0,
            ),
        ]
    )

    # ==========================================================================
    # CONVERSION NODES (DELAYED STARTUP)
    # ==========================================================================
    conversion_nodes = GroupAction(
        [
            LogInfo(msg="Launching Ackermann ↔ VESC converters..."),
            # Ackermann to VESC Converter
            Node(
                package="vesc_ackermann",
                executable="ackermann_to_vesc_node",
                name="ackermann_to_vesc_node",
                parameters=[
                    {"speed_to_erpm_gain": 4614.0, "speed_to_erpm_offset": 0.0}
                ],
                remappings=[
                    ("ackermann_cmd", "drive"),
                ],
                output="screen",
            ),
            # VESC to Odometry Converter
            Node(
                package="vesc_ackermann",
                executable="vesc_to_odom_node",
                name="vesc_to_odom_node",
                parameters=[
                    {
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                        "speed_to_erpm_gain": 4614.0,
                        "wheelbase": 0.33,
                        "publish_tf": False,
                    }
                ],
                remappings=[
                    ("odom", "/ego_racecar/odom"),
                ],
                output="screen",
            ),
        ]
    )

    # ==========================================================================
    # CONTROL INTERFACE
    # ==========================================================================
    control_interface = GroupAction(
        [
            LogInfo(msg="Launching keyboard control interface..."),
            # Keyboard Control Interface
            Node(
                package="joy_converter",
                executable="joy_keyboard",
                name="keyboard_converter",
                parameters=[control_config],
                remappings=[
                    ("drive", "/drive"),
                ],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )

    # ==========================================================================
    # TRANSFORM PUBLISHERS
    # ==========================================================================
    transform_publishers = GroupAction(
        [
            LogInfo(msg="Setting up coordinate transforms..."),
            # Base Link to Odom (since no LiDAR/SLAM)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_tf",
                arguments=[
                    "0",
                    "0",
                    "0",  # x, y, z
                    "0",
                    "0",
                    "0",  # roll, pitch, yaw
                    "map",
                    "odom",
                ],
                output="log",
            ),
        ]
    )

    # ==========================================================================
    # DELAYED STARTUP SEQUENCE
    # ==========================================================================
    delayed_conversion = TimerAction(period=3.0, actions=[conversion_nodes])

    delayed_control = TimerAction(period=5.0, actions=[control_interface])

    # ==========================================================================
    # LAUNCH DESCRIPTION ASSEMBLY
    # ==========================================================================
    return LaunchDescription(
        declared_arguments
        + global_parameters
        + [
            LogInfo(msg="========================================"),
            LogInfo(msg="F1TENTH SYSTEM (NO LIDAR) STARTING..."),
            LogInfo(msg="Hardware: VESC + Servo (no LiDAR)"),
            LogInfo(msg="Controls: W/S=Motor, A/D=Servo, Space=Stop"),
            LogInfo(msg="========================================"),
            # Immediate startup
            hardware_drivers,
            transform_publishers,
            # Delayed startup
            delayed_conversion,
            delayed_control,
            LogInfo(msg="F1TENTH System (No LiDAR) Ready!"),
        ]
    )
