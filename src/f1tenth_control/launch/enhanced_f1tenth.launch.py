#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Launch file otimizado para o sistema F1TENTH com controle avançado
    """
    
    # Argumentos configuráveis
    use_enhanced_control_arg = DeclareLaunchArgument(
        'use_enhanced_control',
        default_value='true',
        description='Usar o nó de controle avançado com PID e failsafe'
    )
    
    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='false',
        description='Habilitar o driver do YDLidar'
    )
    
    enable_diagnostics_arg = DeclareLaunchArgument(
        'enable_diagnostics',
        default_value='true',
        description='Habilitar publicação de diagnósticos'
    )
    
    # Obter caminhos dos pacotes
    f1tenth_control_dir = get_package_share_directory('f1tenth_control')
    vesc_driver_dir = get_package_share_directory('vesc_driver')
    vesc_ackermann_dir = get_package_share_directory('vesc_ackermann')
    joy_converter_dir = get_package_share_directory('joy_converter')
    
    # Arquivos de configuração
    enhanced_config_file = os.path.join(f1tenth_control_dir, 'config', 'enhanced_control_params.yaml')
    
    # ===== 1. VESC DRIVER =====
    vesc_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vesc_driver_dir, 'launch', 'vesc_driver_node.launch.py')
        ),
        launch_arguments={
            'namespace': '',
            'use_sim_time': 'false'
        }.items()
    )
    
    # ===== 2. VESC ACKERMANN - ODOMETRIA =====
    vesc_to_odom_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(vesc_ackermann_dir, 'launch', 'vesc_to_odom_node.launch.xml')
        ),
        launch_arguments={
            'publish_tf': 'false',  # O enhanced_servo_control_node publica TF
            'wheelbase': '0.33',    # Ajustar conforme veículo
            'speed_to_erpm_gain': '4600',
            'speed_to_erpm_offset': '0'
        }.items()
    )
    
    # ===== 3. VESC ACKERMANN - CONTROLE VELOCIDADE =====
    ackermann_to_vesc_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(vesc_ackermann_dir, 'launch', 'ackermann_to_vesc_node.launch.xml')
        ),
        launch_arguments={
            'speed_to_erpm_gain': '4600',
            'speed_to_erpm_offset': '0',
            'steering_angle_to_servo_gain': '1.0',
            'steering_angle_to_servo_offset': '0.5'
        }.items()
    )
    
    # ===== 4. CONTROLE DE JOYSTICK =====
    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(joy_converter_dir, 'launch', 'launch_joy_ackerman.py')
        )
    )
    
    # ===== 5. CONTROLE AVANÇADO DO SERVO =====
    enhanced_servo_control_node = Node(
        package='f1tenth_control',
        executable='enhanced_servo_control_node',
        name='enhanced_servo_control_node',
        parameters=[enhanced_config_file],
        output='screen',
        emulate_tty=True,
        condition=LaunchConfiguration('use_enhanced_control')
    )
    
    # ===== 6. CONTROLE BÁSICO DO SERVO (FALLBACK) =====
    basic_servo_control_node = Node(
        package='f1tenth_control',
        executable='servo_control_node',
        name='servo_control_node',
        parameters=[os.path.join(f1tenth_control_dir, 'config', 'control_params.yaml')],
        output='screen',
        emulate_tty=True,
        condition='unless $(var use_enhanced_control)'
    )
    
    # ===== 7. TRANSFORMAÇÃO ESTÁTICA BASE -> LASER =====
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
    
    # ===== 8. LIDAR (CONDICIONAL) =====
    # Nota: Descomente quando o YDLidar estiver disponível
    # lidar_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'ydlidar_launch.py')
    #     ),
    #     condition=LaunchConfiguration('enable_lidar')
    # )
    
    # ===== 9. DIAGNÓSTICOS =====
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': False,
            'publish_frequency': 30.0
        }],
        condition=LaunchConfiguration('enable_diagnostics')
    )
    
    # ===== 10. MONITOR DE SISTEMA =====
    system_monitor = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_robot_monitor', 'rqt_robot_monitor'],
        output='screen',
        condition=LaunchConfiguration('enable_diagnostics')
    )
    
    # ===== DESCRIÇÃO COMPLETA =====
    return LaunchDescription([
        # Argumentos
        use_enhanced_control_arg,
        enable_lidar_arg,
        enable_diagnostics_arg,
        
        # Componentes principais (ordem de inicialização)
        vesc_driver_launch,              # 1. Driver VESC
        vesc_to_odom_launch,             # 2. Odometria
        ackermann_to_vesc_launch,        # 3. Controle velocidade
        joy_launch,                      # 4. Interface joystick
        
        # Controle de direção (escolha entre básico e avançado)
        enhanced_servo_control_node,     # 5a. Controle avançado
        basic_servo_control_node,        # 5b. Controle básico
        
        # Transformações espaciais
        static_tf_base_to_laser,         # 6. TF estática
        
        # Percepção (quando disponível)
        # lidar_launch,                  # 7. Lidar
        
        # Diagnósticos e monitoramento
        robot_state_publisher,           # 8. Publisher de estado
        # system_monitor,                # 9. Monitor de sistema
    ]) 