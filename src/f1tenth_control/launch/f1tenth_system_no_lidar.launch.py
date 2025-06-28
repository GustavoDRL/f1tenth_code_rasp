#!/usr/bin/env python3
"""
F1TENTH System (No LiDAR) Launch File
====================================
Sistema híbrido: VESC + Servo + Teclado (SEM LiDAR)
Para Raspberry Pi 4B com controle manual por teclado.

Este arquivo de lançamento orquestra os nós essenciais para a operação
manual do F1TENTH, carregando toda a configuração de um arquivo YAML
centralizado para máxima flexibilidade e manutenibilidade.

Componentes:
- VESC Motor Controller (USB)
- Enhanced Servo Steering (GPIO PWM com Failsafe)
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
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    TimerAction,
    LogInfo,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix


def generate_launch_description():
    """Gera a descrição de lançamento do sistema F1TENTH sem LiDAR."""

    # ==========================================================================
    # ARGUMENTOS DE LANÇAMENTO
    # ==========================================================================
    declared_arguments = [
        DeclareLaunchArgument(
            "namespace", default_value="ego_racecar", description="Namespace do robô."
        ),
        DeclareLaunchArgument(
            "debug_mode",
            default_value="false",
            description="Habilita logs de depuração.",
        ),
    ]

    # ==========================================================================
    # ARQUIVO DE CONFIGURAÇÃO CENTRAL
    # ==========================================================================
    control_config = PathJoinSubstitution(
        [FindPackageShare("f1tenth_control"), "config", "system_config.yaml"]
    )

    # ==========================================================================
    # PARÂMETROS GLOBAIS
    # ==========================================================================
    global_parameters = [
        SetParameter(name="use_sim_time", value=False),
        SetParameter(name="robot_namespace", value=LaunchConfiguration("namespace")),
    ]

    # ==========================================================================
    # DRIVERS DE HARDWARE
    # ==========================================================================
    hardware_drivers = GroupAction(
        [
            LogInfo(msg="Lançando drivers de hardware F1TENTH (sem LiDAR)..."),
            # Driver do VESC Motor Controller
            Node(
                package="vesc_driver",
                executable="vesc_driver_node",
                name="vesc_driver",
                parameters=[control_config],  # Carrega de system_config.yaml
                output="screen",
                respawn=True,
                respawn_delay=2.0,
            ),
            # Nó de Controle do Servo (GPIO PWM) - VERSÃO AVANÇADA
            # WORKAROUND: Usando ExecuteProcess para chamar o executável diretamente,
            # pois ros2 launch não o encontra no ambiente ARM64/Humble.
            ExecuteProcess(
                cmd=[
                    PathJoinSubstitution(
                        [
                            FindPackagePrefix("f1tenth_control"),
                            "bin",
                            "enhanced_servo_control_node",
                        ]
                    ),
                    "--ros-args",
                    "--params-file",
                    control_config,
                    "-r",
                    ["__ns:=/", LaunchConfiguration("namespace")],
                    "-r",
                    "__node:=servo_control_node",
                ],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )

    # ==========================================================================
    # NÓS DE CONVERSÃO (INICIALIZAÇÃO ATRASADA)
    # ==========================================================================
    conversion_nodes = GroupAction(
        [
            LogInfo(msg="Lançando conversores Ackermann <=> VESC..."),
            # Conversor Ackermann para VESC
            Node(
                package="vesc_ackermann",
                executable="ackermann_to_vesc_node",
                name="ackermann_to_vesc_node",
                parameters=[control_config],  # Remove hardcoded params
                remappings=[("ackermann_cmd", "drive")],
                output="screen",
            ),
            # Conversor VESC para Odometria
            Node(
                package="vesc_ackermann",
                executable="vesc_to_odom_node",
                name="vesc_to_odom_node",
                parameters=[control_config],  # Remove hardcoded params
                remappings=[("odom", "/ego_racecar/odom")],
                output="screen",
            ),
        ]
    )

    # ==========================================================================
    # INTERFACE DE CONTROLE
    # ==========================================================================
    control_interface = GroupAction(
        [
            LogInfo(msg="Lançando interface de controle por teclado..."),
            # Interface de Controle por Teclado
            Node(
                package="joy_converter",
                executable="joy_keyboard",
                name="joy_keyboard_converter",  # Nome correto para carregar params
                parameters=[control_config],
                remappings=[("drive", "/drive")],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )

    # ==========================================================================
    # PUBLICADORES DE TRANSFORMAÇÕES (TF)
    # ==========================================================================
    transform_publishers = GroupAction(
        [
            LogInfo(msg="Configurando transformações de coordenadas..."),
            # Transformação estática Map -> Odom
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_tf",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
                output="log",
            ),
        ]
    )

    # ==========================================================================
    # SEQUÊNCIA DE INICIALIZAÇÃO ATRASADA
    # ==========================================================================
    # Atraso para garantir que os drivers de hardware estejam prontos
    delayed_conversion = TimerAction(period=3.0, actions=[conversion_nodes])
    delayed_control = TimerAction(period=4.0, actions=[control_interface])

    # ==========================================================================
    # MONTAGEM DA DESCRIÇÃO DE LANÇAMENTO
    # ==========================================================================
    return LaunchDescription(
        declared_arguments
        + global_parameters
        + [
            LogInfo(msg="========================================="),
            LogInfo(msg="F1TENTH SYSTEM (NO LIDAR) STARTING..."),
            LogInfo(msg="Hardware: VESC + Enhanced Servo"),
            LogInfo(msg="Controls: W/S=Motor, A/D=Servo, Space=Stop"),
            LogInfo(msg="========================================="),
            # Inicialização imediata
            hardware_drivers,
            transform_publishers,
            # Inicialização com atraso
            delayed_conversion,
            delayed_control,
            LogInfo(msg="F1TENTH System (No LiDAR) Ready!"),
        ]
    )
