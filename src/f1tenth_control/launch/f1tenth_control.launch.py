#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Obter diretório do pacote
    pkg_dir = get_package_share_directory('f1tenth_control')
    
    # Caminho completo para o arquivo de configuração
    config_file = os.path.join(pkg_dir, 'config', 'control_params.yaml')
    
    # Verificar se o arquivo de configuração existe
    if not os.path.exists(config_file):
        print(f"Erro: Arquivo de configuração não encontrado em {config_file}")
        # Você pode decidir lançar uma exceção ou retornar uma LaunchDescription vazia
        return LaunchDescription([]) 
        
    # Nó de controle do servo
    servo_control_node = Node(
        package='f1tenth_control',
        executable='servo_control_node', # Nome definido no setup.py entry_points
        name='servo_control_node',
        parameters=[config_file],
        output='screen'
    )
    
    # Launch file completo
    return LaunchDescription([
        servo_control_node,
    ]) 