#!/usr/bin/env python3
"""
Conversor Joy Keyboard para F1TENTH
Converte eventos de teclado do controle 8BitDo para comandos Ackermann
Para quando o controle está em modo keyboard ao invés de joystick
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
import threading
import sys
import select
import termios
import tty


class JoyKeyboardConverter(Node):
    def __init__(self):
        super().__init__("joy_keyboard_converter")

        # Parâmetros
        self.max_speed = 3.0
        self.max_angle = 0.32
        self.speed_increment = 0.5
        self.angle_increment = 0.1

        # Estado atual
        self.current_speed = 0.0
        self.current_angle = 0.0

        # Publisher
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)

        # Timer para publicação contínua
        self.timer = self.create_timer(0.1, self.publish_drive)

        # Configurar terminal para entrada de teclado
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.get_logger().info("Conversor teclado iniciado!")
        self.get_logger().info("Controles:")
        self.get_logger().info("  W/S: Acelerar/Frear")
        self.get_logger().info("  A/D: Esquerda/Direita")
        self.get_logger().info("  Espaço: Parar")
        self.get_logger().info("  Q: Sair")

        # Thread para ler teclado
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_reader)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def keyboard_reader(self):
        """Thread para ler entradas do teclado"""
        while self.running:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1).lower()
                self.process_key(key)

    def process_key(self, key):
        """Processa tecla pressionada"""
        if key == "w":  # Acelerar
            self.current_speed = min(
                self.max_speed, self.current_speed + self.speed_increment
            )
            self.get_logger().info(f"Acelerando: {self.current_speed:.1f} m/s")

        elif key == "s":  # Frear
            self.current_speed = max(
                -self.max_speed, self.current_speed - self.speed_increment
            )
            self.get_logger().info(f"Freando: {self.current_speed:.1f} m/s")

        elif key == "a":  # Esquerda
            self.current_angle = min(
                self.max_angle, self.current_angle + self.angle_increment
            )
            self.get_logger().info(f"Virando esquerda: {self.current_angle:.2f} rad")

        elif key == "d":  # Direita
            self.current_angle = max(
                -self.max_angle, self.current_angle - self.angle_increment
            )
            self.get_logger().info(f"Virando direita: {self.current_angle:.2f} rad")

        elif key == " ":  # Parar
            self.current_speed = 0.0
            self.current_angle = 0.0
            self.get_logger().info("Parando e centralizando")

        elif key == "q":  # Sair
            self.get_logger().info("Saindo...")
            self.running = False
            self.restore_terminal()
            rclpy.shutdown()

    def publish_drive(self):
        """Publica comando de direção"""
        msg = AckermannDriveStamped()
        msg.drive.speed = self.current_speed
        msg.drive.steering_angle = self.current_angle

        self.drive_pub.publish(msg)

    def restore_terminal(self):
        """Restaura configurações do terminal"""
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        except:
            pass

    def __del__(self):
        self.restore_terminal()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = JoyKeyboardConverter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.restore_terminal()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
