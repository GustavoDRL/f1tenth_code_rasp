#!/usr/bin/env python3
"""
Launch corrigido para 8BitDo Ultimate 2C Wireless Controller
Resolve problemas de detecção e configuração do joystick

Melhorias:
- Parâmetros específicos para 8BitDo
- Detecção automática de dispositivo
- Configuração robusta de deadzone
- Delay para inicialização sequencial
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Argumentos configuráveis
    device_id_arg = DeclareLaunchArgument(
        "device_id",
        default_value="0",
        description="ID do dispositivo joystick (0, 1, 2...)",
    )

    autorepeat_rate_arg = DeclareLaunchArgument(
        "autorepeat_rate",
        default_value="20.0",
        description="Taxa de repetição automática (Hz)",
    )

    deadzone_arg = DeclareLaunchArgument(
        "deadzone", default_value="0.05", description="Zona morta do joystick (0.0-1.0)"
    )

    debug_mode_arg = DeclareLaunchArgument(
        "debug_mode", default_value="true", description="Habilitar logs detalhados"
    )

    max_speed_arg = DeclareLaunchArgument(
        "max_speed",
        default_value="3.0",
        description="Velocidade máxima (m/s) - reduzida para testes",
    )

    max_angle_arg = DeclareLaunchArgument(
        "max_angle",
        default_value="0.25",
        description="Ângulo máximo direção (rad) - reduzido para segurança",
    )

    # Comando de diagnóstico inicial (opcional)
    diagnostic_cmd = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            'echo "🎮 Verificando dispositivos joystick..." && '
            "ls -la /dev/input/js* 2>/dev/null || "
            'echo "⚠️  Nenhum /dev/input/js* encontrado" && '
            'echo "📋 Dispositivos event disponíveis:" && '
            "ls -la /dev/input/event* | head -3",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("debug_mode")),
    )

    # Nó joy_node com configuração robusta para 8BitDo
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[
            {
                # Configuração básica
                "device_id": LaunchConfiguration("device_id"),
                "deadzone": LaunchConfiguration("deadzone"),
                "autorepeat_rate": LaunchConfiguration("autorepeat_rate"),
                "coalesce_interval": 0.01,  # Reduzir latência
                # Configurações específicas para 8BitDo
                "dev_name": "",  # Deixar vazio para auto-detecção
                "publish_period": 0.05,  # 20Hz de publicação
                "sticky_buttons": False,  # Desabilitar botões grudados
                "default_trig_val": False,  # Triggers como botões
            }
        ],
        output="screen",
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
    )

    # Nó conversor joy_ackerman com delay para aguardar joy_node
    joy_converter = TimerAction(
        period=3.0,  # 3 segundos de delay para inicialização
        actions=[
            Node(
                package="joy_converter",
                executable="joy_ackerman",
                name="joy_ackerman",
                parameters=[
                    {
                        # Limites de segurança reduzidos para testes
                        "max_speed": LaunchConfiguration("max_speed"),
                        "max_angle": LaunchConfiguration("max_angle"),
                        "controller_error": 0.1,  # Dead zone no converter
                        # Configurações específicas
                        "emergency_button": 10,  # Botão PS/Xbox para reset
                        "enable_safety": True,
                        "timeout_safety": 1.0,  # Timeout de 1 segundo
                    }
                ],
                remappings=[("joy", "/joy"), ("drive", "/drive")],
                output="screen",
                emulate_tty=True,
                respawn=True,
                respawn_delay=2.0,
            )
        ],
    )

    # Nó de monitoramento (opcional para debug)
    monitor_node = TimerAction(
        period=5.0,  # 5 segundos após inicialização
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-c",
                    'echo "📊 STATUS JOYSTICK:" && '
                    'ros2 topic list | grep -E "joy|drive" && '
                    'echo "🔍 Verificando publicação /joy..." && '
                    'timeout 2s ros2 topic echo /joy --once 2>/dev/null || echo "❌ Tópico /joy não está publicando"',
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("debug_mode")),
            )
        ],
    )

    return LaunchDescription(
        [
            # Argumentos
            device_id_arg,
            autorepeat_rate_arg,
            deadzone_arg,
            debug_mode_arg,
            max_speed_arg,
            max_angle_arg,
            # Diagnóstico inicial
            diagnostic_cmd,
            # Nós principais
            joy_node,
            joy_converter,
            monitor_node,
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
