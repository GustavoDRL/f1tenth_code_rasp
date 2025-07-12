#!/usr/bin/env python3
"""
F1TENTH Complete System Launch File
==================================
Sistema completo: VESC + Servo + LiDAR + Teclado
Para Raspberry Pi 4B com controle manual por teclado

Componentes:
- VESC Motor Controller (USB)
- Servo Steering (GPIO PWM)
- YDLiDAR X4 (USB) com configuração personalizada (`custom_x4.yaml`)
- Keyboard Control Interface
- Real-time Odometry & TF

Uso:
    ros2 launch f1tenth_control f1tenth_complete_system.launch.py

Detalhes da Configuração do LiDAR:
- O arquivo `custom_x4.yaml` é usado para definir `isSingleChannel: true`,
  essencial para a operação correta de alguns modelos YDLIDAR X4.
- A frequência do scan está configurada para 10Hz no mesmo arquivo.

Controles de Teclado:
    W/S: Acelerar/Frear
    A/D: Esquerda/Direita
    Espaço: Parar
    Q: Sair
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    TimerAction,
    LogInfo,
    ExecuteProcess,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix


def generate_launch_description():
    """Generate complete F1TENTH system launch description."""

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

    # Parâmetros do VESC - A porta é definida diretamente para maior robustez
    vesc_params = {
        "port": "/dev/sensors/vesc",
        "speed_max": 20000.0,
    }

    # Parâmetros do LiDAR definidos inline para garantir funcionamento
    lidar_params = {
        "port": "/dev/ydlidar",
        "frame_id": "laser_frame",
        "ignore_array": "",
        "baudrate": 128000,
        "lidar_type": 1,
        "device_type": 0,
        "sample_rate": 5,
        "abnormal_check_count": 4,
        "fixed_resolution": True,
        "reversion": True,
        "inverted": True,
        "auto_reconnect": True,
        "isSingleChannel": True,
        "intensity": False,
        "support_motor_dtr": False,
        "angle_max": 180.0,
        "angle_min": -180.0,
        "range_max": 12.0,
        "range_min": 0.1,
        "frequency": 10.0,
        "invalid_range_is_inf": False,
    }

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
            LogInfo(msg="Launching F1TENTH hardware drivers..."),
            # VESC Motor Controller Driver
            Node(
                package="vesc_driver",
                executable="vesc_driver_node",
                name="vesc_driver",
                parameters=[vesc_params],  # Usa os parâmetros definidos aqui
                output="screen",
                respawn=True,
                respawn_delay=2.0,
            ),
            # Servo Control Node (GPIO PWM)
            ExecuteProcess(
                cmd=[
                    PathJoinSubstitution(
                        [
                            FindPackagePrefix("f1tenth_control"),
                            "bin",
                            "servo_control_node",
                        ]
                    ),
                    "--ros-args",
                    "--params-file",
                    control_config,
                    "-r",
                    ["__ns:=/", LaunchConfiguration("namespace")],
                    "-r",
                    # Renomeia para carregar os parâmetros corretos do YAML
                    "__node:=enhanced_servo_control_node",
                ],
                output="screen",
                emulate_tty=True,
            ),
            # YDLiDAR X4 Driver
            Node(
                package="ydlidar_ros2_driver",
                executable="ydlidar_ros2_driver_node",
                name="ydlidar_node",
                parameters=[lidar_params],  # Usa os parâmetros inline
                output="screen",
                respawn=True,
                respawn_delay=3.0,
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
    # TRANSFORM PUBLISHERS
    # ==========================================================================
    transform_publishers = GroupAction(
        [
            LogInfo(msg="Setting up coordinate transforms..."),
            # Base Link to Laser Frame
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser_tf",
                arguments=[
                    "0.1",
                    "0.0",
                    "0.2",  # x, y, z (10cm forward, 20cm up)
                    "0",
                    "0",
                    "0",  # roll, pitch, yaw
                    "base_link",
                    "laser_frame",
                ],
                output="log",
            ),
        ]
    )

    # ==========================================================================
    # DELAYED STARTUP SEQUENCE
    # ==========================================================================
    delayed_conversion = TimerAction(period=3.0, actions=[conversion_nodes])

    # ==========================================================================
    # LAUNCH DESCRIPTION ASSEMBLY
    # ==========================================================================
    return LaunchDescription(
        declared_arguments
        + global_parameters
        + [
            LogInfo(msg="========================================"),
            LogInfo(msg="F1TENTH COMPLETE SYSTEM STARTING..."),
            LogInfo(msg="Hardware: VESC + Servo + LiDAR"),
            LogInfo(msg="Controls: W/S=Motor, A/D=Servo, Space=Stop"),
            LogInfo(msg="========================================"),
            # Immediate startup
            hardware_drivers,
            transform_publishers,
            # Delayed startup
            delayed_conversion,
            LogInfo(msg="F1TENTH Complete System Ready!"),
            LogInfo(
                msg="Para controle manual, execute: "
                "ros2 run f1tenth_control keyboard_control"
            ),
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
    print("NOTE: Keyboard control node must be started in a separate terminal:")
    print("ros2 run f1tenth_control keyboard_control")
    print("")
    print("System Components:")
    print("- VESC Motor Controller (USB)")
    print("- Servo Steering Control (GPIO)")
    print("- YDLiDAR X4 (USB)")
    print("- Ackermann ↔ VESC Conversion")
    print("- Real-time Odometry")
    print("- Coordinate Frame Transforms")
