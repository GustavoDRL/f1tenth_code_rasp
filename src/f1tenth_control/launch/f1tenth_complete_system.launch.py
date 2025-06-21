#!/usr/bin/env python3
"""
F1TENTH Complete System Launch File
==================================

Sistema híbrido completo para controle manual:
- VESC Motor Controller (USB serial)
- Servo Steering (GPIO PWM)
- Keyboard Control Interface
- Real-time Odometry

Soluciona problema de comunicação entre nós com contexto ROS2 unificado.

Uso:
    ros2 launch f1tenth_control f1tenth_complete_system.launch.py

Autor: F1TENTH Team
Data: 2025-06-21
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate complete F1TENTH system launch description."""

    # ==========================================================================
    # LAUNCH ARGUMENTS
    # ==========================================================================
    declared_arguments = [
        DeclareLaunchArgument(
            "namespace",
            default_value="ego_racecar",
            description="Robot namespace for F1TENTH standard",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time (false for real hardware)",
        ),
        DeclareLaunchArgument(
            "enable_keyboard",
            default_value="true",
            description="Enable keyboard control interface",
        ),
        DeclareLaunchArgument(
            "enable_servo",
            default_value="true",
            description="Enable GPIO servo control",
        ),
        DeclareLaunchArgument(
            "enable_safety",
            default_value="true",
            description="Enable safety monitoring and limits",
        ),
        DeclareLaunchArgument(
            "debug_mode",
            default_value="false",
            description="Enable debug logging and monitoring",
        ),
    ]

    # ==========================================================================
    # CONFIGURATION FILES
    # ==========================================================================
    config_file = PathJoinSubstitution(
        [FindPackageShare("f1tenth_control"), "config", "system_config.yaml"]
    )

    # ==========================================================================
    # GLOBAL PARAMETERS
    # ==========================================================================
    global_parameters = [
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
        SetParameter(name="robot_namespace", value=LaunchConfiguration("namespace")),
    ]

    # ==========================================================================
    # HARDWARE DRIVERS GROUP
    # ==========================================================================
    hardware_drivers = GroupAction(
        [
            LogInfo(msg="Launching F1TENTH hardware drivers..."),
            # VESC Motor Controller Driver
            Node(
                package="vesc_driver",
                executable="vesc_driver_node",
                name="vesc_driver",
                namespace=LaunchConfiguration("namespace"),
                parameters=[config_file],
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                arguments=["--ros-args", "--log-level", "INFO"],
            ),
            # Servo Control Node (GPIO)
            Node(
                package="f1tenth_control",
                executable="servo_control_node",
                name="servo_control_node",
                namespace=LaunchConfiguration("namespace"),
                parameters=[config_file],
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_servo")),
                respawn=True,
                respawn_delay=2.0,
            ),
        ]
    )

    # ==========================================================================
    # CONVERSION NODES GROUP
    # ==========================================================================
    conversion_nodes = GroupAction(
        [
            LogInfo(msg="Launching Ackermann ↔ VESC converters..."),
            # Ackermann to VESC Converter (CRITICAL FOR MOTOR CONTROL)
            Node(
                package="vesc_ackermann",
                executable="ackermann_to_vesc_node",
                name="ackermann_to_vesc_node",
                namespace=LaunchConfiguration("namespace"),
                parameters=[config_file],
                output="screen",
                respawn=True,
                respawn_delay=1.0,
                remappings=[
                    ("ackermann_cmd", "drive"),  # Subscribe to /drive topic
                ],
            ),
            # VESC to Odometry Converter
            Node(
                package="vesc_ackermann",
                executable="vesc_to_odom_node",
                name="vesc_to_odom_node",
                namespace=LaunchConfiguration("namespace"),
                parameters=[config_file],
                output="screen",
                respawn=True,
                respawn_delay=1.0,
                remappings=[
                    ("odom", "vesc/odom"),  # Publish to namespaced odom
                ],
            ),
        ]
    )

    # ==========================================================================
    # CONTROL INTERFACE GROUP
    # ==========================================================================
    control_interface = GroupAction(
        [
            LogInfo(msg="Launching control interfaces..."),
            # Keyboard Control Interface
            Node(
                package="joy_converter",
                executable="joy_keyboard",
                name="joy_keyboard_converter",
                parameters=[config_file],
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_keyboard")),
                remappings=[
                    ("drive", "/ego_racecar/drive"),  # Publish to namespaced drive
                ],
            ),
        ]
    )

    # ==========================================================================
    # TRANSFORM PUBLISHERS GROUP
    # ==========================================================================
    transform_publishers = GroupAction(
        [
            LogInfo(msg="Setting up coordinate frame transforms..."),
            # Base Link to Laser Frame Transform
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_base_to_laser",
                namespace=LaunchConfiguration("namespace"),
                arguments=[
                    "0.1",
                    "0.0",
                    "0.2",  # x, y, z translation (10cm forward, 20cm up)
                    "0",
                    "0",
                    "0",  # roll, pitch, yaw rotation
                    "base_link",  # parent frame
                    "laser_frame",  # child frame
                ],
                output="log",
            ),
        ]
    )

    # ==========================================================================
    # DELAYED STARTUP SEQUENCE
    # ==========================================================================
    # Hardware drivers start immediately
    # Conversion nodes start after 3 seconds (allow hardware initialization)
    # Control interface starts after 5 seconds (allow full system ready)

    delayed_conversions = TimerAction(period=3.0, actions=[conversion_nodes])

    delayed_control = TimerAction(period=5.0, actions=[control_interface])

    # ==========================================================================
    # SYSTEM MONITORING (DEBUG MODE)
    # ==========================================================================
    debug_nodes = GroupAction(
        [
            LogInfo(msg="Debug mode enabled - launching monitoring tools..."),
            # Topic Monitor (if debug enabled)
            Node(
                package="rqt_topic",
                executable="rqt_topic",
                name="topic_monitor",
                condition=IfCondition(LaunchConfiguration("debug_mode")),
                output="log",
            ),
        ],
        condition=IfCondition(LaunchConfiguration("debug_mode")),
    )

    # ==========================================================================
    # LAUNCH DESCRIPTION ASSEMBLY
    # ==========================================================================
    return LaunchDescription(
        declared_arguments
        + global_parameters
        + [
            LogInfo(msg="========================================"),
            LogInfo(msg="F1TENTH Complete System Starting..."),
            LogInfo(msg="Hardware: Raspberry Pi 4B + VESC 6.2"),
            LogInfo(msg="Controls: W/S=Motor, A/D=Servo, Space=Stop"),
            LogInfo(msg="========================================"),
            # Immediate startup
            hardware_drivers,
            transform_publishers,
            # Delayed startup for proper initialization
            delayed_conversions,
            delayed_control,
            # Optional debug tools
            debug_nodes,
            LogInfo(msg="F1TENTH System Launch Complete!"),
        ]
    )


# =============================================================================
# LAUNCH FILE VALIDATION
# =============================================================================
if __name__ == "__main__":
    # This section runs when launch file is executed directly
    print("F1TENTH Complete System Launch File")
    print("===================================")
    print("This file launches the complete F1TENTH hybrid control system.")
    print("Usage: ros2 launch f1tenth_control f1tenth_complete_system.launch.py")
    print("")
    print("System Components:")
    print("- VESC Motor Controller (USB)")
    print("- Servo Steering Control (GPIO)")
    print("- Keyboard Control Interface")
    print("- Ackermann ↔ VESC Conversion")
    print("- Real-time Odometry")
    print("- Coordinate Frame Transforms")
    print("")
    print("Expected Result:")
    print("- W/S keys control motor speed")
    print("- A/D keys control steering servo")
    print("- Space key stops the vehicle")
    print("- Q key exits the control interface")
