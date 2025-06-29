#!/usr/bin/env python3
"""
F1TENTH Complete System Launch File
==================================
Sistema completo compatível com simulador F1TENTH: VESC + Servo + LiDAR + Map
Para Raspberry Pi 4B com controle manual por teclado

COMPATIBILIDADE F1TENTH SIMULATOR:
- Publica todos os tópicos requeridos: /scan, /ego_racecar/odom, /map
- Subscreve aos tópicos padrão: /drive, /initialpose  
- Integra teleop_twist_keyboard para compatibilidade total
- Mantém árvore TF completa: map -> odom -> base_link -> laser_frame

Componentes:
- VESC Motor Controller (USB)
- Servo Steering (GPIO PWM) 
- YDLiDAR X4 (USB) com configuração personalizada
- Map Server (para compatibilidade com códigos do simulador)
- Keyboard Control Interface (teleop_twist_keyboard)
- Real-time Odometry & TF

Uso:
    ros2 launch f1tenth_control f1tenth_complete_system.launch.py

Modos de operação:
    # Modo hardware (padrão)
    ros2 launch f1tenth_control f1tenth_complete_system.launch.py
    
    # Modo compatibilidade com simulador
    ros2 launch f1tenth_control f1tenth_complete_system.launch.py \
        enable_map_server:=true \
        enable_teleop_twist:=true \
        enable_initialpose:=true

Detalhes da Configuração do LiDAR:
- O arquivo inline params garante `isSingleChannel: true`
- A frequência do scan está configurada para 10Hz
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
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
            "namespace", default_value="ego_racecar", description="Robot namespace"
        ),
        DeclareLaunchArgument(
            "debug_mode", default_value="false", description="Enable debug logging"
        ),
        DeclareLaunchArgument(
            "enable_map_server",
            default_value="true",
            description="Enable map server for /map topic",
        ),
        DeclareLaunchArgument(
            "enable_teleop_twist",
            default_value="false",
            description="Enable teleop_twist_keyboard",
        ),
        DeclareLaunchArgument(
            "enable_initialpose",
            default_value="true",
            description="Enable /initialpose topic support",
        ),
        DeclareLaunchArgument(
            "map_file",
            default_value="/opt/ros/humble/share/nav2_map_server/maps/turtlebot3_world.yaml",
            description="Path to map file (for simulator compatibility)",
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
                parameters=[vesc_config],
                output="screen",
                respawn=True,
                respawn_delay=2.0,
            ),
            # Servo Control Node (GPIO PWM)
            Node(
                package="f1tenth_control",
                executable="enhanced_servo_control_node",
                name="enhanced_servo_control_node",
                namespace=LaunchConfiguration("namespace"),
                parameters=[control_config],
                output="screen",
            ),
            # YDLiDAR X4 Driver com parâmetros inline
            Node(
                package="ydlidar_ros2_driver",
                executable="ydlidar_ros2_driver_node",
                name="ydlidar_node",
                parameters=[lidar_params],
                output="screen",
                respawn=True,
                respawn_delay=3.0,
            ),
        ]
    )

    # ==========================================================================
    # F1TENTH SIMULATOR COMPATIBILITY NODES
    # ==========================================================================
    simulator_compatibility = GroupAction(
        [
            LogInfo(msg="Setting up F1TENTH simulator compatibility..."),
            # Map Server (publica o tópico /map requerido pelo simulador)
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[
                    {"yaml_filename": LaunchConfiguration("map_file")},
                    {"topic_name": "map"},
                    {"frame_id": "map"},
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_map_server")),
            ),
            # Map Server Lifecycle Manager
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                parameters=[
                    {"autostart": True},
                    {"node_names": ["map_server"]},
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_map_server")),
            ),
            # Teleop Twist Keyboard (compatibilidade com simulador)
            Node(
                package="teleop_twist_keyboard",
                executable="teleop_twist_keyboard",
                name="teleop_twist_keyboard",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_teleop_twist")),
                remappings=[
                    ("cmd_vel", "/cmd_vel_keyboard"),
                ],
            ),
            # Twist to Ackermann Converter (para teleop_twist_keyboard)
            Node(
                package="f1tenth_control",
                executable="twist_to_ackermann_node",
                name="twist_to_ackermann",
                parameters=[
                    {"max_speed": 2.0},
                    {"max_steering_angle": 0.4},
                    {"wheelbase": 0.32},
                ],
                remappings=[
                    ("cmd_vel", "/cmd_vel_keyboard"),
                    ("ackermann_cmd", "/drive"),
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_teleop_twist")),
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
    # TRANSFORM PUBLISHERS (F1TENTH STANDARD)
    # ==========================================================================
    transform_publishers = GroupAction(
        [
            LogInfo(msg="Setting up F1TENTH coordinate transforms..."),
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
            # Map to Odom Transform (para compatibilidade sem SLAM)
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
    # INITIALPOSE SUPPORT (F1TENTH SIMULATOR COMPATIBILITY)
    # ==========================================================================
    initialpose_support = Node(
        package="f1tenth_control",
        executable="initialpose_handler_node",
        name="initialpose_handler",
        parameters=[
            {"base_frame": "base_link"},
            {"odom_frame": "odom"},
        ],
        remappings=[
            ("initialpose", "/initialpose"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_initialpose")),
    )

    # ==========================================================================
    # DELAYED STARTUP SEQUENCE
    # ==========================================================================
    delayed_conversion = TimerAction(period=3.0, actions=[conversion_nodes])
    delayed_simulator_compat = TimerAction(
        period=5.0, actions=[simulator_compatibility]
    )

    # ==========================================================================
    # LAUNCH DESCRIPTION ASSEMBLY
    # ==========================================================================
    return LaunchDescription(
        declared_arguments
        + global_parameters
        + [
            LogInfo(msg="========================================"),
            LogInfo(msg="F1TENTH COMPLETE SYSTEM STARTING..."),
            LogInfo(msg="Hardware: VESC + Servo + LiDAR + Map"),
            LogInfo(msg="Mode: F1TENTH Simulator Compatible"),
            LogInfo(msg="Topics: /scan, /ego_racecar/odom, /map, /drive"),
            LogInfo(msg="Teleop: W/S=Motor, A/D=Servo, Space=Stop"),
            LogInfo(msg="========================================"),
            # Immediate startup
            hardware_drivers,
            transform_publishers,
            initialpose_support,
            # Delayed startup
            delayed_conversion,
            delayed_simulator_compat,
            LogInfo(msg="F1TENTH Complete System Ready!"),
            LogInfo(msg="Para controle: ros2 run f1tenth_control keyboard_control"),
            LogInfo(
                msg="Teleop twist: ros2 run teleop_twist_keyboard "
                "teleop_twist_keyboard"
            ),
            LogInfo(
                msg="Verifique tópicos: ros2 topic list | grep -E "
                "'/(scan|ego_racecar/odom|map|drive)'"
            ),
        ]
    )


# =============================================================================
# LAUNCH FILE VALIDATION
# =============================================================================
if __name__ == "__main__":
    # Esta seção executa quando o launch file é executado diretamente
    print("F1TENTH Complete System Launch File")
    print("===================================")
    print("Sistema compatível com simulador F1TENTH")
    print("Usage: ros2 launch f1tenth_control " "f1tenth_complete_system.launch.py")
    print("")
    print("MODO COMPATIBILIDADE SIMULADOR:")
    print("ros2 launch f1tenth_control f1tenth_complete_system.launch.py \\")
    print("    enable_map_server:=true \\")
    print("    enable_teleop_twist:=true \\")
    print("    enable_initialpose:=true")
    print("")
    print("TÓPICOS F1TENTH PUBLICADOS:")
    print("- /scan: LaserScan data")
    print("- /ego_racecar/odom: Odometry data")
    print("- /map: Map data (com map_server)")
    print("- TF tree: map->odom->base_link->laser_frame")
    print("")
    print("TÓPICOS F1TENTH SUBSCRITOS:")
    print("- /drive: AckermannDriveStamped commands")
    print("- /initialpose: Pose reset via RViz")
    print("")
    print("System Components:")
    print("- VESC Motor Controller (USB)")
    print("- Servo Steering Control (GPIO)")
    print("- YDLiDAR X4 (USB)")
    print("- Map Server (Nav2)")
    print("- Ackermann ↔ VESC Conversion")
    print("- Real-time Odometry & TF")
    print("- F1TENTH Simulator Compatibility Layer")
