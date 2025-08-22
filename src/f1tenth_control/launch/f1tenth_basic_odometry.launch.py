#!/usr/bin/env python3
"""
F1TENTH Basic Odometry Launch File
==================================
Sistema de odometria básica otimizada para F1TENTH.

Componentes:
- VESC Motor Controller (USB)  
- Servo Steering (GPIO PWM)
- Servo Feedback Publisher (sincronização)
- VESC to Odometry Enhanced (filtros + validação)
- Ackermann Converter (mantido)
- YDLiDAR X4 (opcional)

Melhorias implementadas:
- Sincronização servo-motor via tópico real
- Filtros de ruído simples
- Validação de dados e outliers
- Covariância adaptativa
- Integração numérica melhorada

Uso:
    ros2 launch f1tenth_control f1tenth_basic_odometry.launch.py

Author: F1TENTH Team
Date: 2025-01-26
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
    """Generate F1TENTH basic odometry launch description."""

    # ==========================================================================
    # LAUNCH ARGUMENTS
    # ==========================================================================
    declared_arguments = [
        DeclareLaunchArgument(
            "namespace", 
            default_value="ego_racecar", 
            description="Robot namespace"
        ),
        DeclareLaunchArgument(
            "debug_odometry", 
            default_value="false",
            description="Enable odometry debug output"
        ),
        DeclareLaunchArgument(
            "enable_lidar", 
            default_value="true",
            description="Enable YDLiDAR X4"
        ),
        DeclareLaunchArgument(
            "config_file", 
            default_value="basic_odometry_config.yaml",
            description="Configuration file name"
        ),
    ]

    # ==========================================================================
    # CONFIGURATION FILES
    # ==========================================================================
    config_file = PathJoinSubstitution([
        FindPackageShare("f1tenth_control"), 
        "config", 
        LaunchConfiguration("config_file")
    ])

    # ==========================================================================
    # GLOBAL PARAMETERS
    # ==========================================================================
    global_parameters = [
        SetParameter(name="use_sim_time", value=False),
        SetParameter(name="robot_namespace", value=LaunchConfiguration("namespace")),
    ]

    # ==========================================================================
    # HARDWARE DRIVERS (IMEDIATO)
    # ==========================================================================
    hardware_drivers = GroupAction([
        LogInfo(msg="🚀 Iniciando drivers de hardware F1TENTH..."),
        
        # VESC Motor Controller Driver
        Node(
            package="vesc_driver",
            executable="vesc_driver_node",
            name="vesc_driver",
            parameters=[{
                "port": "/dev/sensors/vesc",
                "speed_max": 20000.0,
                "speed_min": -20000.0,
                "use_servo_cmd": False,  # Servo via GPIO
                "publish_odom": False,   # Odometria via vesc_to_odom
            }],
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        ),
        
        # Enhanced Servo Control Node (GPIO PWM)
        Node(
            package="f1tenth_control",
            executable="enhanced_servo_control_node",
            name="enhanced_servo_control",
            parameters=[config_file],
            output="screen",
            respawn=True,
            respawn_delay=1.0,
        ),
        
        # Servo Feedback Publisher (NOVO - sincronização)
        Node(
            package="f1tenth_control",
            executable="servo_feedback_publisher",
            name="servo_feedback_publisher",
            parameters=[config_file],
            output="log",  # Log level baixo para não poluir
        ),
        
        # YDLiDAR X4 Driver (condicional)
        Node(
            package="ydlidar_ros2_driver",
            executable="ydlidar_ros2_driver_node",
            name="ydlidar_node",
            parameters=[{
                "port": "/dev/ydlidar",
                "frame_id": "laser_frame",
                "baudrate": 128000,
                "lidar_type": 1,
                "device_type": 0,
                "sample_rate": 5,
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
            }],
            output="screen",
            respawn=True,
            respawn_delay=3.0,
            condition=lambda context: LaunchConfiguration("enable_lidar").perform(context) == "true"
        ),
    ])

    # ==========================================================================
    # CONVERSION NODES (STARTUP ATRASADO)
    # ==========================================================================
    conversion_nodes = GroupAction([
        LogInfo(msg="🔄 Iniciando conversores Ackermann ↔ VESC..."),
        
        # Ackermann to VESC Converter (mantido)
        Node(
            package="vesc_ackermann",
            executable="ackermann_to_vesc_node",
            name="ackermann_to_vesc_node",
            parameters=[config_file],
            remappings=[
                ("ackermann_cmd", "drive"),
            ],
            output="screen",
        ),
        
        # VESC to Odometry Enhanced (SUBSTITUÍDO)
        Node(
            package="vesc_ackermann",
            executable="vesc_to_odom_node",
            name="vesc_to_odom_enhanced",
            parameters=[config_file],
            output="screen",
            respawn=True,
            respawn_delay=1.0,
        ),
    ])

    # ==========================================================================
    # TRANSFORM PUBLISHERS
    # ==========================================================================
    transform_publishers = GroupAction([
        LogInfo(msg="🗺️ Configurando transformações de coordenadas..."),
        
        # Base Link to Laser Frame
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_laser_tf",
            arguments=[
                "0.1", "0.0", "0.2",  # x, y, z (10cm forward, 20cm up)
                "0", "0", "0",        # roll, pitch, yaw
                "base_link",
                "laser_frame",
            ],
            output="log",
        ),
    ])

    # ==========================================================================
    # DELAYED STARTUP SEQUENCE
    # ==========================================================================
    # Dar tempo para hardware drivers inicializarem
    delayed_conversion = TimerAction(period=3.0, actions=[conversion_nodes])

    # ==========================================================================
    # SISTEMA DE MONITORAMENTO (OPCIONAL)
    # ==========================================================================
    monitoring_nodes = GroupAction([
        LogInfo(msg="📊 Iniciando monitoramento do sistema..."),
        
        # Monitor de tópicos críticos
        ExecuteProcess(
            cmd=[
                "bash", "-c", 
                "sleep 5 && ros2 topic hz /odom --window 10 --timeout 30 || echo '⚠️ Odometria não disponível'"
            ],
            output="screen",
            condition=lambda context: LaunchConfiguration("debug_odometry").perform(context) == "true"
        ),
    ])

    # ==========================================================================
    # LAUNCH DESCRIPTION ASSEMBLY  
    # ==========================================================================
    return LaunchDescription(
        declared_arguments
        + global_parameters
        + [
            LogInfo(msg="========================================"),
            LogInfo(msg="🏁 F1TENTH BASIC ODOMETRY STARTING..."),
            LogInfo(msg="🚀 Hardware: VESC + Servo GPIO + LiDAR"),
            LogInfo(msg="🔄 Odometry: Enhanced with filters"),
            LogInfo(msg="📊 Features: Real-time sync + validation"),
            LogInfo(msg="========================================"),
            
            # Startup sequencial
            hardware_drivers,           # t=0s: Hardware
            transform_publishers,       # t=0s: TF estático
            delayed_conversion,         # t=3s: Conversion layers
            monitoring_nodes,           # t=0s: Monitoring (se debug)
            
            LogInfo(msg="✅ F1TENTH Basic Odometry System Ready!"),
            LogInfo(msg="📝 Logs: ros2 topic echo /odom"),
            LogInfo(msg="📊 Stats: ros2 topic hz /odom --window 10"),
            LogInfo(msg="🎮 Control: ros2 run f1tenth_control keyboard_control"),
            LogInfo(msg="========================================"),
        ]
    )


# =============================================================================
# LAUNCH FILE VALIDATION
# =============================================================================
if __name__ == "__main__":
    # Validation quando executado diretamente
    print("🏁 F1TENTH Basic Odometry Launch File")
    print("=====================================")
    print("✅ Sistema de odometria básica otimizada para F1TENTH")
    print("📋 Componentes:")
    print("   - VESC Motor Controller (USB)")
    print("   - Enhanced Servo Control (GPIO)")
    print("   - Servo Feedback Sync (NEW)")
    print("   - VESC to Odom Enhanced (NEW)")
    print("   - YDLiDAR X4 (opcional)")
    print("")
    print("🚀 Uso:")
    print("   ros2 launch f1tenth_control f1tenth_basic_odometry.launch.py")
    print("")
    print("🔧 Opções:")
    print("   debug_odometry:=true   # Habilitar debug de odometria")
    print("   enable_lidar:=false    # Desabilitar LiDAR")
    print("   config_file:=custom.yaml # Usar configuração customizada")
    print("")
    print("📊 Melhorias implementadas:")
    print("   ✅ Sincronização servo-motor real")
    print("   ✅ Filtros de ruído e outliers")
    print("   ✅ Validação de dados")
    print("   ✅ Covariância adaptativa")
    print("   ✅ Integração numérica melhorada")
    print("   ✅ Estatísticas em tempo real")
    print("")
    print("⚠️  IMPORTANTE: Execute sempre o pigpio daemon primeiro:")
    print("   sudo systemctl start pigpiod")
    print("=====================================")