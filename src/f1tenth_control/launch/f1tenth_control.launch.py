#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Launch file simplificado para teste do sistema F1TENTH
    """
    
    # Argumentos configuráveis
    use_enhanced_control_arg = DeclareLaunchArgument(
        'use_enhanced_control',
        default_value='false',
        description='Usar o nó de controle avançado com PID'
    )
    
    # Obter caminhos dos pacotes
    f1tenth_control_dir = get_package_share_directory('f1tenth_control')
    
    # Arquivos de configuração
    basic_config_file = os.path.join(f1tenth_control_dir, 'config', 'control_params.yaml')
    enhanced_config_file = os.path.join(f1tenth_control_dir, 'config', 'enhanced_control_params.yaml')
    
    # ===== CONTROLE BÁSICO DO SERVO =====
    basic_servo_control_node = Node(
        package='f1tenth_control',
        executable='servo_control_node',
        name='servo_control_node',
        parameters=[basic_config_file],
        output='screen',
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration('use_enhanced_control'))
    )
    
    # ===== CONTROLE AVANÇADO DO SERVO =====
    enhanced_servo_control_node = Node(
        package='f1tenth_control',
        executable='enhanced_servo_control_node',
        name='enhanced_servo_control_node',
        parameters=[enhanced_config_file],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('use_enhanced_control'))
    )
    
    # ===== TRANSFORMAÇÃO ESTÁTICA BASE -> LASER =====
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=[
            '0.1', '0.0', '0.2',    # x, y, z (metros)
            '0.0', '0.0', '0.0',    # roll, pitch, yaw (radianos)
            'base_link',            # frame pai
            'laser_frame'           # frame filho
        ],
        output='screen'
    )
    
    # ===== DESCRIÇÃO COMPLETA =====
    return LaunchDescription([
        # Argumentos
        use_enhanced_control_arg,
        
        # Componentes
        basic_servo_control_node,      # Controle básico (padrão)
        enhanced_servo_control_node,   # Controle avançado (opcional)
        static_tf_base_to_laser,       # TF estática
    ]) 