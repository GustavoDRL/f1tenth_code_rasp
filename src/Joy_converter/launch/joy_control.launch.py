#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    """Generates the launch description for starting joystick control."""
   
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
            joy_node,
            joy_ackerman_node
        ]
    ) 