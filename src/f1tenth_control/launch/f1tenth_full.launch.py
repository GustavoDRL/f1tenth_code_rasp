#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Obter diretórios dos pacotes
    f1tenth_control_dir = get_package_share_directory('f1tenth_control')
    vesc_driver_dir = get_package_share_directory('vesc_driver')
    vesc_ackermann_dir = get_package_share_directory('vesc_ackermann')
    joy_converter_dir = get_package_share_directory('joy_converter')
    # ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver') # Para quando o Lidar for adicionado

    # --- Componentes Individuais ---

    # 1. Driver VESC (usando o launch file do pacote vesc_driver)
    vesc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vesc_driver_dir, 'launch', 'vesc_driver_node.launch.py')
        )
        # Nota: O vesc_driver_node.launch.py deve carregar o config YAML apropriado.
        # Você pode precisar passar parâmetros para ele se o nome do YAML não for padrão.
    )

    # 2. VESC Ackermann (Nó de odometria: vesc_to_odom)
    # Este nó lê dados do VESC (/sensors/core) e publica em /odom
    # Certifique-se que os parâmetros no launch file correspondam à sua configuração!
    vesc_to_odom_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource( # Usar Any para suportar XML
            os.path.join(vesc_ackermann_dir, 'launch', 'vesc_to_odom_node.launch.xml')
        ),
        # Você pode sobrescrever parâmetros aqui se necessário, por exemplo:
        # launch_arguments={'publish_tf': 'true', 'wheelbase': '0.33'}.items()
    )
    
    # 3. VESC Ackermann (Nó de comando: ackermann_to_vesc)
    # Este nó converte /drive (Ackermann) para comandos VESC (/commands/motor/speed)
    # O servo é tratado pelo nosso nó `servo_control_node` diretamente
    ackermann_to_vesc_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource( # Usar Any para suportar XML
            os.path.join(vesc_ackermann_dir, 'launch', 'ackermann_to_vesc_node.launch.xml')
        )
    )

    # 4. Conversor de Joystick
    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(joy_converter_dir, 'launch', 'launch_joy_ackerman.py')
        )
    )
    
    # 5. Lidar (Descomentar quando pronto)
    # lidar_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(ydlidar_dir, 'launch', 'ydlidar_launch.py') # Verifique o nome do launch file
    #     )
    # )
    
    # 6. Nó de Controle do Servo (que também republica odometria)
    servo_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_control_dir, 'launch', 'f1tenth_control.launch.py')
        )
    )
    
    # --- Launch File Completo ---
    return LaunchDescription([
        # Iniciar o driver VESC primeiro
        vesc_launch,
        
        # Nós de conversão Ackermann/VESC
        vesc_to_odom_launch, # Para calcular a odometria inicial em /odom
        ackermann_to_vesc_launch, # Para enviar comandos de velocidade ao VESC
        
        # Interface com o usuário (Joystick)
        joy_launch,
        
        # Lidar (quando adicionado)
        # lidar_launch,  
        
        # Nosso nó customizado para controle do servo e republicação de odometria
        servo_control_launch,
    ]) 