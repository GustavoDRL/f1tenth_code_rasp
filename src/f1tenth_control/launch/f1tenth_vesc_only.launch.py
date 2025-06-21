#!/usr/bin/env python3
"""
F1TENTH VESC Only Launch File
============================
Sistema simplificado: Apenas VESC + Teclado (SEM Servo, SEM LiDAR)
Para testes do motor e desenvolvimento isolado

Componentes:
- VESC Motor Controller (USB)
- Keyboard Control Interface
- Basic Odometry

Uso:
    ros2 launch f1tenth_control f1tenth_vesc_only.launch.py

Controles de Teclado:
    W/S: Acelerar/Frear
    Espaço: Parar
    Q: Sair
    (A/D: Sem efeito - sem servo)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate F1TENTH VESC-only launch description."""

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
            LogInfo(msg="Launching VESC motor driver only..."),
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
        ]
    )

    # ==========================================================================
    # CONVERSION NODES (DELAYED STARTUP)
    # ==========================================================================
    conversion_nodes = GroupAction(
        [
            LogInfo(msg="Launching Ackermann → VESC converter..."),
            # Ackermann to VESC Converter (Motor only)
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
            # VESC to Odometry Converter (Basic odometry)
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
                        "publish_tf": True,  # VESC publishes basic TF
                        "use_servo_cmd_to_calc_angular_velocity": False,  # No servo
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
            LogInfo(msg="Launching keyboard control (motor only)..."),
            # Keyboard Control Interface (motor commands only)
            Node(
                package="joy_converter",
                executable="joy_keyboard",
                name="keyboard_converter",
                parameters=[
                    {
                        "max_speed": 2.0,  # Conservative speed for testing
                        "max_steering_angle": 0.0,  # No steering without servo
                        "speed_increment": 0.3,  # Smaller increments for safety
                        "steering_increment": 0.0,  # No steering
                    }
                ],
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
            LogInfo(msg="Setting up basic coordinate transforms..."),
            # Map to Odom (static since no SLAM)
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
            LogInfo(msg="F1TENTH VESC ONLY SYSTEM STARTING..."),
            LogInfo(msg="Hardware: VESC Motor Only"),
            LogInfo(msg="Controls: W/S=Motor, Space=Stop"),
            LogInfo(msg="Note: A/D keys have no effect (no servo)"),
            LogInfo(msg="========================================"),
            # Immediate startup
            hardware_drivers,
            transform_publishers,
            # Delayed startup
            delayed_conversion,
            delayed_control,
            LogInfo(msg="F1TENTH VESC Only System Ready!"),
        ]
    )
