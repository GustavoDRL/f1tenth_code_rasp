#!/usr/bin/env python3
"""
Twist to Ackermann Converter Node
=================================
Converte mensagens geometry_msgs/Twist (do teleop_twist_keyboard)
para ackermann_msgs/AckermannDriveStamped para compatibilidade
total com o simulador F1TENTH.

Este nó permite usar o teleop_twist_keyboard padrão do ROS2
em vez do nosso keyboard_control customizado, garantindo que
códigos testados no simulador F1TENTH funcionem no hardware real.

Conversão:
- cmd_vel.linear.x  -> drive.speed
- cmd_vel.angular.z -> drive.steering_angle (usando modelo cinemático)

Autor: F1TENTH Team
Data: 2025-06-28
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
import math


class TwistToAckermannNode(Node):
    """
    Node para converter Twist para AckermannDriveStamped.

    Características:
    - Converte velocidade linear diretamente
    - Calcula ângulo de direção usando modelo cinemático
    - Aplica limites de segurança
    - Mantém compatibilidade com simulador F1TENTH
    """

    def __init__(self):
        super().__init__("twist_to_ackermann_node")

        # Declarar parâmetros do veículo
        self.declare_parameter("max_speed", 2.0)
        self.declare_parameter("max_steering_angle", 0.4)  # ~23 graus
        self.declare_parameter("wheelbase", 0.32)  # metros
        self.declare_parameter("publish_rate", 50.0)  # Hz

        # Obter parâmetros com valores seguros
        self.max_speed = self.get_parameter("max_speed").value or 2.0
        max_steer_param = self.get_parameter("max_steering_angle").value
        self.max_steering_angle = max_steer_param or 0.4
        self.wheelbase = self.get_parameter("wheelbase").value or 0.32
        self.publish_rate = self.get_parameter("publish_rate").value or 50.0

        # Subscriber para cmd_vel (Twist)
        self.twist_subscription = self.create_subscription(
            Twist, "cmd_vel", self.twist_callback, 10
        )

        # Publisher para ackermann_cmd
        self.ackermann_publisher = self.create_publisher(
            AckermannDriveStamped, "ackermann_cmd", 10
        )

        # Timer para publicação periódica (manter último comando)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        # Estado interno
        self.last_twist = Twist()
        self.last_ackermann = AckermannDriveStamped()

        self.get_logger().info("Twist to Ackermann converter initialized")
        self.get_logger().info(
            f"Parameters: max_speed={self.max_speed}, "
            f"max_steering={self.max_steering_angle}, "
            f"wheelbase={self.wheelbase}"
        )

    def twist_callback(self, msg: Twist):
        """
        Callback para processar mensagens Twist.

        Args:
            msg: Mensagem Twist recebida
        """
        self.last_twist = msg

        # Converter para Ackermann
        ackermann_msg = self.twist_to_ackermann(msg)
        self.last_ackermann = ackermann_msg

        # Publicar imediatamente
        self.ackermann_publisher.publish(ackermann_msg)

    def twist_to_ackermann(self, twist: Twist) -> AckermannDriveStamped:
        """
        Converte Twist para AckermannDriveStamped.

        Args:
            twist: Mensagem Twist de entrada

        Returns:
            AckermannDriveStamped: Mensagem convertida
        """
        ackermann_msg = AckermannDriveStamped()

        # Header
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        ackermann_msg.header.frame_id = "base_link"

        # Velocidade linear (aplicar limites)
        speed = twist.linear.x
        speed = max(-self.max_speed, min(self.max_speed, speed))
        ackermann_msg.drive.speed = speed

        # Ângulo de direção a partir da velocidade angular
        # Usando modelo cinemático:
        # angular_velocity = speed * tan(steering_angle) / wheelbase
        if abs(speed) > 0.01:  # Evitar divisão por zero
            # steering_angle = atan(angular_velocity * wheelbase / speed)
            steering_angle = math.atan(twist.angular.z * self.wheelbase / speed)
        else:
            # Se velocidade é zero, usar relação simplificada
            # Para baixas velocidades, angular.z é proporcional
            steering_angle = twist.angular.z * 0.5  # Fator empírico

        # Aplicar limites de ângulo de direção
        steering_angle = max(
            -self.max_steering_angle, min(self.max_steering_angle, steering_angle)
        )
        ackermann_msg.drive.steering_angle = steering_angle

        # Outros campos (opcional)
        ackermann_msg.drive.steering_angle_velocity = 0.0
        ackermann_msg.drive.acceleration = 0.0
        ackermann_msg.drive.jerk = 0.0

        return ackermann_msg

    def timer_callback(self):
        """
        Timer callback para manter publicação periódica.
        Republicar último comando para manter controle ativo.
        """
        # Verificar se recebemos comandos recentemente
        if hasattr(self, "last_ackermann"):
            # Atualizar timestamp
            self.last_ackermann.header.stamp = self.get_clock().now().to_msg()
            self.ackermann_publisher.publish(self.last_ackermann)

    def destroy_node(self):
        """Cleanup ao destruir o nó."""
        # Publicar comando de parada
        stop_msg = AckermannDriveStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = "base_link"
        stop_msg.drive.speed = 0.0
        stop_msg.drive.steering_angle = 0.0

        self.ackermann_publisher.publish(stop_msg)

        super().destroy_node()


def main(args=None):
    """Função principal."""
    rclpy.init(args=args)

    node = TwistToAckermannNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down twist_to_ackermann_node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
