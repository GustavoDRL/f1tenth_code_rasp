#!/usr/bin/env python3
"""
F1TENTH Servo Only Launch File
=============================
Sistema simplificado: Apenas Servo + Teclado (SEM VESC, SEM LiDAR)
Para testes de direção e calibração do servo

Componentes:
- Servo Steering (GPIO PWM)
- Keyboard Control Interface
- Static TF publishers

Uso:
    ros2 launch f1tenth_control f1tenth_servo_only.launch.py

Controles de Teclado:
    A/D: Esquerda/Direita
    Espaço: Centralizar servo
    Q: Sair
    (W/S: Sem efeito - sem motor)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate F1TENTH Servo-only launch description."""

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
            LogInfo(msg="Launching servo control only..."),
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
    # CONTROL INTERFACE
    # ==========================================================================
    control_interface = GroupAction(
        [
            LogInfo(msg="Launching keyboard control (servo only)..."),
            # Keyboard Control Interface (steering commands only)
            Node(
                package="joy_converter",
                executable="joy_keyboard",
                name="keyboard_converter",
                parameters=[
                    {
                        "max_speed": 0.0,  # No speed without motor
                        "max_steering_angle": 0.4,  # Full steering range
                        "speed_increment": 0.0,  # No speed control
                        "steering_increment": 0.05,  # Fine steering control
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
            LogInfo(msg="Setting up static coordinate transforms..."),
            # Map to Odom (static)
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
            # Odom to Base Link (static since no motion)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="odom_to_base_tf",
                arguments=[
                    "0",
                    "0",
                    "0",  # x, y, z
                    "0",
                    "0",
                    "0",  # roll, pitch, yaw
                    "odom",
                    "base_link",
                ],
                output="log",
            ),
        ]
    )

    # ==========================================================================
    # DELAYED STARTUP SEQUENCE
    # ==========================================================================
    delayed_control = TimerAction(period=3.0, actions=[control_interface])

    # ==========================================================================
    # LAUNCH DESCRIPTION ASSEMBLY
    # ==========================================================================
    return LaunchDescription(
        declared_arguments
        + global_parameters
        + [
            LogInfo(msg="========================================"),
            LogInfo(msg="F1TENTH SERVO ONLY SYSTEM STARTING..."),
            LogInfo(msg="Hardware: Servo Steering Only"),
            LogInfo(msg="Controls: A/D=Steering, Space=Center"),
            LogInfo(msg="Note: W/S keys have no effect (no motor)"),
            LogInfo(msg="========================================"),
            # Immediate startup
            hardware_drivers,
            transform_publishers,
            # Delayed startup
            delayed_control,
            LogInfo(msg="F1TENTH Servo Only System Ready!"),
        ]
    )
