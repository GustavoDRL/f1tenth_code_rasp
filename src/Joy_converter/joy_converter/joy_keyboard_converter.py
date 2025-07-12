#!/usr/bin/env python3
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseWithCovarianceStamped

class JoyToAckermannImproved(Node):
    def __init__(self):
        super().__init__('joy_ackermann_improved')
        
        # Configurações principais
        self.max_speed = 7.0
        self.max_angle = 0.32
        self.deadzone = 0.05  # Deadzone menor
        
        # Configurações de mapeamento (podem ser ajustadas)
        self.speed_axis = 1      # Eixo vertical do analógico esquerdo
        self.steering_axis = 0   # Eixo horizontal do analógico esquerdo
        self.invert_speed = False    # Se True, inverte o eixo de velocidade
        self.invert_steering = False # Se True, inverte o eixo de direção
        
        # Modo de controle
        self.control_mode = "direct"  # "direct" ou "incremental"
        self.speed_increment = 0.2
        self.angle_increment = 0.05
        
        # Estado atual (para modo incremental)
        self.current_speed = 0.0
        self.current_angle = 0.0
        
        # Estado dos botões (para evitar múltiplos triggers)
        self.last_buttons = []
        
        # Subscribers e Publishers
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)
        
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10)
        
        # Timer para debug
        self.debug_timer = self.create_timer(1.0, self.debug_callback)
        self.last_joy_msg = None
        
        # Publicar pose inicial
        self.publish_initial_pose()
        
        self.get_logger().info('=== JoyToAckermann Improved Iniciado ===')
        self.get_logger().info(f'Modo de controle: {self.control_mode}')
        self.get_logger().info(f'Deadzone: {self.deadzone}')
        self.get_logger().info(f'Eixo velocidade: {self.speed_axis} (invertido: {self.invert_speed})')
        self.get_logger().info(f'Eixo direção: {self.steering_axis} (invertido: {self.invert_steering})')
        self.get_logger().info('Controles:')
        if self.control_mode == "direct":
            self.get_logger().info('  Analógico esquerdo: Direção e velocidade')
        else:
            self.get_logger().info('  Botões X/Circle: Acelerar/Frear')
            self.get_logger().info('  Analógico esquerdo horizontal: Direção')
        self.get_logger().info('  Botão PS: Reset posição')
        self.get_logger().info('  Botão Triangle: Trocar modo de controle')
        self.get_logger().info('==========================================')

    def publish_initial_pose(self):
        """Publica pose inicial do robô"""
        msg_map = PoseWithCovarianceStamped()
        msg_map.header.stamp = self.get_clock().now().to_msg()
        msg_map.header.frame_id = 'map'
        
        # Pose inicial
        msg_map.pose.pose.position.x = 0.0
        msg_map.pose.pose.position.y = 0.0
        msg_map.pose.pose.position.z = 0.0
        msg_map.pose.pose.orientation.x = 0.0
        msg_map.pose.pose.orientation.y = 0.0
        msg_map.pose.pose.orientation.z = 0.0
        msg_map.pose.pose.orientation.w = 1.0
        
        # Covariância
        msg_map.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]
        
        self.pose_publisher.publish(msg_map)
        self.get_logger().info('Pose inicial publicada')

    def debug_callback(self):
        """Callback de debug para mostrar valores dos eixos"""
        if self.last_joy_msg and len(self.last_joy_msg.axes) >= 4:
            axes = self.last_joy_msg.axes
            self.get_logger().info(
                f'Debug - Eixos RAW: [0]={axes[0]:.3f}, [1]={axes[1]:.3f}, '
                f'[2]={axes[2]:.3f}, [3]={axes[3]:.3f}'
            )

    def apply_deadzone(self, value):
        """Aplica deadzone ao valor"""
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def button_pressed(self, msg, button_index):
        """Verifica se um botão foi pressionado (edge detection)"""
        if len(msg.buttons) <= button_index:
            return False
        
        # Inicializar last_buttons se necessário
        while len(self.last_buttons) <= button_index:
            self.last_buttons.append(0)
        
        current_state = msg.buttons[button_index]
        last_state = self.last_buttons[button_index]
        self.last_buttons[button_index] = current_state
        
        # Retorna True apenas na transição de 0 para 1
        return current_state == 1 and last_state == 0

    def joy_callback(self, msg):
        """Callback principal do joystick"""
        self.last_joy_msg = msg
        
        # Verificar se temos eixos suficientes
        if len(msg.axes) < 4:
            self.get_logger().warn('Joystick não tem eixos suficientes')
            return
        
        # === BOTÕES ESPECIAIS ===
        
        # Botão PS (reset posição) - geralmente botão 10 ou 12
        for ps_button in [10, 11, 12]:
            if self.button_pressed(msg, ps_button):
                self.get_logger().info('Botão PS pressionado - Reset posição')
                self.publish_initial_pose()
                self.current_speed = 0.0
                self.current_angle = 0.0
                break
        
        # Botão Triangle (trocar modo) - geralmente botão 3
        if self.button_pressed(msg, 3):
            self.control_mode = "incremental" if self.control_mode == "direct" else "direct"
            self.get_logger().info(f'Modo alterado para: {self.control_mode}')
            self.current_speed = 0.0
            self.current_angle = 0.0
        
        # === CONTROLE DE MOVIMENTO ===
        
        if self.control_mode == "direct":
            self.direct_control(msg)
        else:
            self.incremental_control(msg)
        
        # Publicar comando
        self.publish_ackermann_command()

    def direct_control(self, msg):
        """Controle direto pelos analógicos"""
        # Ler valores dos eixos
        speed_raw = msg.axes[self.speed_axis]
        steering_raw = msg.axes[self.steering_axis]
        
        # Aplicar inversão se necessário
        if self.invert_speed:
            speed_raw = -speed_raw
        if self.invert_steering:
            steering_raw = -steering_raw
        
        # Aplicar deadzone
        speed_axis = self.apply_deadzone(speed_raw)
        steering_axis = self.apply_deadzone(steering_raw)
        
        # Calcular velocidade e ângulo
        self.current_speed = self.max_speed * speed_axis
        self.current_angle = self.max_angle * steering_axis
        
        # Debug detalhado
        if abs(speed_raw) > 0.01 or abs(steering_raw) > 0.01:
            self.get_logger().info(
                f'Direct - Speed: RAW={speed_raw:.3f} -> PROCESSED={speed_axis:.3f} -> FINAL={self.current_speed:.2f} | '
                f'Steering: RAW={steering_raw:.3f} -> PROCESSED={steering_axis:.3f} -> FINAL={self.current_angle:.2f}'
            )

    def incremental_control(self, msg):
        """Controle incremental por botões"""
        # Velocidade por botões
        if len(msg.buttons) > 0 and msg.buttons[0] == 1:  # Botão X - acelerar
            self.current_speed = min(self.max_speed, self.current_speed + self.speed_increment)
            
        if len(msg.buttons) > 1 and msg.buttons[1] == 1:  # Botão Circle - frear
            self.current_speed = max(-self.max_speed, self.current_speed - self.speed_increment)
        
        # Botão Square para parar
        if len(msg.buttons) > 2 and msg.buttons[2] == 1:
            self.current_speed = 0.0
        
        # Direção pelo analógico horizontal
        steering_raw = msg.axes[self.steering_axis]
        if self.invert_steering:
            steering_raw = -steering_raw
        
        steering_axis = self.apply_deadzone(steering_raw)
        self.current_angle = self.max_angle * steering_axis
        
        # Debug
        if any(msg.buttons[:3]) or abs(steering_raw) > 0.01:
            self.get_logger().info(
                f'Incremental - Speed: {self.current_speed:.2f} | Steering: {self.current_angle:.2f}'
            )

    def publish_ackermann_command(self):
        """Publica comando Ackermann"""
        ackermann_cmd = AckermannDriveStamped()
        ackermann_cmd.header.stamp = self.get_clock().now().to_msg()
        ackermann_cmd.header.frame_id = 'base_link'
        ackermann_cmd.drive.speed = self.current_speed
        ackermann_cmd.drive.steering_angle = self.current_angle
        
        self.drive_publisher.publish(ackermann_cmd)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        joy_ackermann = JoyToAckermannImproved()
        print("JoyToAckermann Improved Initialized")
        rclpy.spin(joy_ackermann)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        try:
            joy_ackermann.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()