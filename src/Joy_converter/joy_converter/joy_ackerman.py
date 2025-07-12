#!/usr/bin/env python3
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseWithCovarianceStamped

class JoyToAckerman(Node):
    def __init__(self):
        super().__init__('joy_ackerman')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',  # Substitua 'joy_topic' pelo tópico correto onde os dados do joystick são publicados
            self.joy_callback,
            10)
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',  # Substitua 'ackermann_cmd' pelo tópico onde os comandos ackermann serão publicados
            10)
        self.max_speed = 7.0  # Limite de velocidade em m/s
        self.max_acceleration = 0
        self.max_angle = 0.32 # Limite de vangulo em rad
        self.max_angular_acceleration = 0.01
        self.controller_error = 0.1
        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10)
       
        self.publish_initial_pose()
        
        # Debug: Criar timer para mostrar valores dos eixos
        self.debug_timer = self.create_timer(1.0, self.debug_callback)
        self.last_joy_msg = None
   
    def publish_initial_pose(self):
        msg_map = PoseWithCovarianceStamped()
        # Fill header
        msg_map.header.stamp = self.get_clock().now().to_msg()
        msg_map.header.frame_id = 'map'
        # Fill pose
        msg_map.pose.pose.position.x = 0.0
        msg_map.pose.pose.position.y = 0.0
        msg_map.pose.pose.position.z = 0.0
        msg_map.pose.pose.orientation.x = 0.0
        msg_map.pose.pose.orientation.y = 0.0
        msg_map.pose.pose.orientation.z = 0.0
        msg_map.pose.pose.orientation.w = 1.0  # Corrigido: quaternion válido
        # Fill covariance
        msg_map.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]
        self.publisher_.publish(msg_map)
        self.get_logger().info('Initial pose published')
    
    def debug_callback(self):
        if self.last_joy_msg:
            self.get_logger().info(f'Joy axes: {self.last_joy_msg.axes}')
     
    def joy_callback(self, msg):
        self.last_joy_msg = msg
        
        # Verificar se temos eixos suficientes
        if len(msg.axes) < 4:
            self.get_logger().warn('Joystick não tem eixos suficientes')
            return
        
        # CORREÇÃO 1: Testar diferentes mapeamentos de eixos
        # Configuração padrão (pode precisar ajustar):
        # axes[0] = Analógico esquerdo horizontal (esquerda/direita)
        # axes[1] = Analógico esquerdo vertical (frente/trás)
        # axes[2] = Analógico direito horizontal
        # axes[3] = Analógico direito vertical
        
        # Teste com analógico esquerdo para direção
        speed_axis = msg.axes[1]      # Analógico esquerdo vertical
        steering_axis = msg.axes[0]   # Analógico esquerdo horizontal
        
        # Eliminar valores muito próximos a zero para evitar movimentos indesejados
        if abs(speed_axis) < self.controller_error:
            speed_axis = 0.0
        if abs(steering_axis) < self.controller_error:
            steering_axis = 0.0
        
        # Return the car to initial position when PS button is pressed
        if len(msg.buttons) > 10 and msg.buttons[10] == 1:
            self.publish_initial_pose()
        
        # CORREÇÃO 2: Inverter sinal se necessário
        # Se o carro não vai para trás, tente inverter o sinal da velocidade
        speed = self.max_speed * speed_axis  # Remover o sinal negativo se estava presente
        
        # CORREÇÃO 3: Ajustar direção
        # Se a direção está invertida, inverter o sinal
        steering_angle = -self.max_angle * steering_axis  # Negativo para corrigir inversão
        
        # Publicar os comandos ackermann
        ackermann_cmd = AckermannDriveStamped()
        ackermann_cmd.header.stamp = self.get_clock().now().to_msg()
        ackermann_cmd.header.frame_id = 'base_link'
        ackermann_cmd.drive.speed = speed
        ackermann_cmd.drive.steering_angle = steering_angle
        
        self.publisher.publish(ackermann_cmd)
        
        # Debug log
        if abs(speed) > 0.1 or abs(steering_angle) > 0.1:
            self.get_logger().info(f'Speed: {speed:.2f}, Steering: {steering_angle:.2f}')

def main(args=None):
    rclpy.init(args=args)
    joy_ackerman = JoyToAckerman()
    print("JoyToAckerman Initialized")
    rclpy.spin(joy_ackerman)
    joy_ackerman.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()