#!/usr/bin/env python3
"""
Launch file completo F1TENTH - Servo + VESC + Conversores
Integra todo o sistema de controle hardware
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Argumentos configuráveis
    vesc_config_arg = DeclareLaunchArgument(
        "vesc_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("vesc_config"), "config", "vesc_config.yaml"]
        ),
        description="Arquivo de configuração VESC",
    )

    servo_config_arg = DeclareLaunchArgument(
        "servo_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("f1tenth_control"), "config", "control_params.yaml"]
        ),
        description="Arquivo de configuração servo",
    )

    # 1. Servo Control Node
    servo_node = Node(
        package="f1tenth_control",
        executable="servo_control_node",
        name="servo_control_node",
        parameters=[LaunchConfiguration("servo_config")],
        output="screen",
        emulate_tty=True,
    )

    # 2. VESC Driver
    vesc_driver_node = TimerAction(
        period=2.0,  # Aguardar servo inicializar
        actions=[
            Node(
                package="vesc_driver",
                executable="vesc_driver_node",
                name="vesc_driver",
                parameters=[LaunchConfiguration("vesc_config")],
                output="screen",
                emulate_tty=True,
            )
        ],
    )

    # 3. Ackermann to VESC converter
    ackermann_to_vesc = TimerAction(
        period=4.0,  # Aguardar VESC driver inicializar
        actions=[
            Node(
                package="vesc_ackermann",
                executable="ackermann_to_vesc_node",
                name="ackermann_to_vesc",
                parameters=[LaunchConfiguration("vesc_config")],
                remappings=[
                    ("ackermann_cmd", "/drive"),
                ],
                output="screen",
                emulate_tty=True,
            )
        ],
    )

    # 4. VESC to Odometry converter
    vesc_to_odom = TimerAction(
        period=5.0,  # Aguardar conversores
        actions=[
            Node(
                package="vesc_ackermann",
                executable="vesc_to_odom_node",
                name="vesc_to_odom",
                parameters=[LaunchConfiguration("vesc_config")],
                remappings=[
                    ("odom", "/odom"),
                ],
                output="screen",
                emulate_tty=True,
            )
        ],
    )

    # 5. Static Transform Publisher (base_link -> laser_frame)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_laser",
        arguments=["0.1", "0.0", "0.2", "0", "0", "0", "base_link", "laser_frame"],
        output="screen",
    )

    return LaunchDescription(
        [
            vesc_config_arg,
            servo_config_arg,
            servo_node,
            static_tf,
            vesc_driver_node,
            ackermann_to_vesc,
            vesc_to_odom,
        ]
    )
