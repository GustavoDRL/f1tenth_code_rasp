#!/usr/bin/env python3
"""
Launch file corrigido para sistema F1TENTH híbrido
Arquitetura: VESC (motor) + GPIO Raspberry (servo)

CORREÇÃO: Remapeamento correto /drive -> /ackermann_cmd
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Gerar descrição do launch corrigido"""

    # Argumentos de configuração
    declared_arguments = [
        DeclareLaunchArgument(
            'namespace',
            default_value='ego_racecar',
            description='Namespace do robô'
        ),
        DeclareLaunchArgument(
            'debug_mode',
            default_value='false',
            description='Habilitar modo debug'
        ),
        DeclareLaunchArgument(
            'enable_joystick',
            default_value='true',
            description='Habilitar controle por joystick'
        )
    ]

    # Caminhos dos pacotes
    f1tenth_control_dir = get_package_share_directory('f1tenth_control')
    vesc_driver_dir = get_package_share_directory('vesc_driver')
    vesc_ackermann_dir = get_package_share_directory('vesc_ackermann')
    vesc_config_dir = get_package_share_directory('vesc_config')
    joy_converter_dir = get_package_share_directory('joy_converter')

    # Arquivos de configuração
    control_config = os.path.join(f1tenth_control_dir, 'config', 'control_params.yaml')
    vesc_config = os.path.join(vesc_config_dir, 'config', 'vesc_config.yaml')

    # ===== 1. DRIVER VESC =====
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver',
        parameters=[vesc_config],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    # ===== 2. CONVERSOR ACKERMANN -> VESC (COM REMAPEAMENTO CORRIGIDO) =====
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[
            {'speed_to_erpm_gain': 4614.0},
            {'speed_to_erpm_offset': 0.0},
            {'steering_angle_to_servo_gain': -1.2135},
            {'steering_angle_to_servo_offset': 0.5304}
        ],
        remappings=[
            ('ackermann_cmd', 'drive'),  # ← CORREÇÃO CRÍTICA!
        ],
        output='screen'
    )

    # ===== 3. CONVERSOR VESC -> ODOMETRIA =====
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[
            {'odom_frame': 'odom'},
            {'base_frame': 'base_link'},
            {'speed_to_erpm_gain': 4614.0},
            {'speed_to_erpm_offset': 0.0},
            {'use_servo_cmd_to_calc_angular_velocity': True},
            {'steering_angle_to_servo_gain': -1.2135},
            {'steering_angle_to_servo_offset': 0.5304},
            {'wheelbase': 0.33},
            {'publish_tf': False}  # servo_control_node publica TF
        ],
        output='screen'
    )

    # ===== 4. CONTROLE SERVO GPIO =====
    servo_control_node = Node(
        package='f1tenth_control',
        executable='servo_control_node',
        name='servo_control_node',
        parameters=[control_config],
        output='screen',
        emulate_tty=True
    )

    # ===== 5. TRANSFORMAÇÃO ESTÁTICA =====
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=[
            '0.1', '0.0', '0.2',    # x, y, z
            '0.0', '0.0', '0.0',    # roll, pitch, yaw
            'base_link',
            'laser_frame'
        ],
        output='screen'
    )

    # ===== 6. CONTROLE JOYSTICK (OPCIONAL) =====
    joystick_nodes = GroupAction([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0
            }],
            condition=IfCondition(LaunchConfiguration('enable_joystick'))
        ),

        Node(
            package='joy_converter',
            executable='joy_ackerman',
            name='joy_ackerman',
            remappings=[
                ('joy', '/joy'),
                ('/drive', '/drive')  # Já mapeado corretamente
            ],
            condition=IfCondition(LaunchConfiguration('enable_joystick'))
        )
    ])

    # Inicialização escalonada para evitar conflitos
    delayed_components = TimerAction(
        period=2.0,  # 2 segundos de delay
        actions=[
            ackermann_to_vesc_node,
            vesc_to_odom_node,
            servo_control_node
        ]
    )

    return LaunchDescription(
        declared_arguments +
        [
            # Driver VESC primeiro
            vesc_driver_node,
            
            # Componentes principais (com delay)
            delayed_components,
            
            # Transformações e joystick
            static_tf_base_to_laser,
            joystick_nodes
        ]
    ) 