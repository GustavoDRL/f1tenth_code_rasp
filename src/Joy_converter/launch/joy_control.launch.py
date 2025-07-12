#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    """Generates the launch description for starting joystick control."""

    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'vesc_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('vesc_config'), 'config', 'vesc_config.yaml'
            ]),
            description='Path to the VESC configuration file.'
        ),
    ]

    # VESC Driver Node
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver',
        parameters=[LaunchConfiguration('vesc_config')],
        output='screen'
    )

    # Ackermann to VESC Node
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc',
        parameters=[LaunchConfiguration('vesc_config')],
        remappings=[('ackermann_cmd_in', '/drive')]
    )

    # VESC to Odometry Node
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    
    # Joy Node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }]
    )

    # Joy Ackerman Converter Node
    joy_ackerman_node = Node(
        package='joy_converter',
        executable='joy_ackerman',
        name='joy_ackerman_converter',
        output='screen'
    )

    return LaunchDescription(
        declared_arguments + 
        [
            vesc_driver_node,
            ackermann_to_vesc_node,
            vesc_to_odom_node,
            joy_node,
            joy_ackerman_node
        ]
    ) 