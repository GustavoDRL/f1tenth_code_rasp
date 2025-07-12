#!/usr/bin/env python3
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyToAckerman(Node):
    def __init__(self):
        super().__init__('joy_ackerman_converter')
        
        # Parâmetros com valores padrão seguros
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('max_steering_angle', 0.34) # ~20 degrees
        self.declare_parameter('deadman_button', 4) # Botão LB (Left Bumper) como padrão
        self.declare_parameter('speed_axis', 1) # Analógico esquerdo vertical
        self.declare_parameter('steer_axis', 0) # Analógico esquerdo horizontal

        # Leitura dos parâmetros
        self.max_speed = self.get_parameter('max_speed').get_value()
        self.max_steering_angle = self.get_parameter('max_steering_angle').get_value()
        self.deadman_button = self.get_parameter('deadman_button').get_value()
        self.speed_axis = self.get_parameter('speed_axis').get_value()
        self.steer_axis = self.get_parameter('steer_axis').get_value()

        self.get_logger().info(f"Usando 'deadman' no botão: {self.deadman_button}")
        self.get_logger().info(f"Eixo de velocidade: {self.speed_axis}, Eixo de direção: {self.steer_axis}")
        self.get_logger().info(f"Velocidade máxima: {self.max_speed} m/s, Ângulo máximo: {self.max_steering_angle} rad")

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)

    def joy_callback(self, msg: Joy):
        # Cria a mensagem de comando
        ackermann_cmd = AckermannDriveStamped()
        
        # Botão de segurança (deadman switch)
        # Se o botão não estiver pressionado, envia comando de parada
        if msg.buttons[self.deadman_button] == 0:
            ackermann_cmd.drive.speed = 0.0
            ackermann_cmd.drive.steering_angle = 0.0
        else:
            # Mapeia os eixos para velocidade e direção
            # Inverte o eixo de direção para que esquerda seja positivo e direita negativo (padrão ROS)
            speed = msg.axes[self.speed_axis] * self.max_speed
            steering_angle = msg.axes[self.steer_axis] * self.max_steering_angle
            
            ackermann_cmd.drive.speed = speed
            ackermann_cmd.drive.steering_angle = steering_angle

        # Publica o comando
        self.publisher.publish(ackermann_cmd)

def main(args=None):
    rclpy.init(args=args)
    joy_ackerman = JoyToAckerman()
    print("JoyToAckerman Converter Node Initialized")
    print("Segure o botão LB (ou o botão 4) para ativar os comandos.")
    rclpy.spin(joy_ackerman)
    joy_ackerman.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()