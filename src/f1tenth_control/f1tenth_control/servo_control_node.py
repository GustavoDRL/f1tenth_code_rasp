#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped
# Tentativa de importação segura para compatibilidade com Windows
try:
    import pigpio
except ImportError:
    print("\n*** Atenção: Biblioteca pigpio não encontrada. O controle de GPIO não funcionará. ***")
    print("*** Instale em sistemas baseados em Debian/Ubuntu com: sudo apt install python3-pigpio ***")
    print("*** Lembre-se de iniciar o daemon: sudo systemctl start pigpiod ***\n")
    pigpio = None # Define como None se não puder ser importado

import math
import time
from tf2_ros import TransformBroadcaster
import signal
import contextlib

# Context manager para timeout na conexão GPIO
@contextlib.contextmanager
def gpio_timeout(timeout_sec):
    """Context manager que gera TimeoutError se ultrapassar timeout."""
    def _handler(signum, frame):
        raise TimeoutError(f"GPIO connection timeout after {timeout_sec}s")
    old_handler = signal.signal(signal.SIGALRM, _handler)
    signal.alarm(int(timeout_sec))
    try:
        yield
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, old_handler)

class ServoControlNode(Node):
    """
    Nó ROS2 para controle integrado do VESC e servo em um Raspberry Pi
    para veículos F1TENTH.
    """

    def __init__(self):
        super().__init__('servo_control_node')

        # Declarar e obter parâmetros
        self.declare_parameters(
            namespace='',
            parameters=[
                ('servo_gpio_pin', 18),
                ('servo_pwm_frequency', 50),
                ('servo_min_pulse_width', 1000),
                ('servo_max_pulse_width', 2000),
                ('gpio_connection_timeout', 5.0),  # Timeout (s) para pigpio
                ('max_steering_angle', 0.4),
                ('min_steering_angle', -0.4),
                ('odom_frame', 'odom'),
                ('base_frame', 'base_link'),
                ('drive_topic', '/drive'),
                ('odom_topic', '/ego_racecar/odom'),
                ('use_vesc_odom', True),
                ('vesc_odom_topic', '/odom')
            ]
        )

        # Obter parâmetros
        self.servo_gpio_pin = self.get_parameter('servo_gpio_pin').value
        self.servo_pwm_frequency = self.get_parameter('servo_pwm_frequency').value
        self.servo_min_pulse_width = self.get_parameter('servo_min_pulse_width').value
        self.servo_max_pulse_width = self.get_parameter('servo_max_pulse_width').value
        self.gpio_connection_timeout = self.get_parameter('gpio_connection_timeout').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.min_steering_angle = self.get_parameter('min_steering_angle').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.use_vesc_odom = self.get_parameter('use_vesc_odom').value
        self.vesc_odom_topic = self.get_parameter('vesc_odom_topic').value

        # Conectar ao daemon pigpio apenas se a biblioteca foi importada
        self.pi = None
        if pigpio:
            try:
                # Conectar com timeout para evitar travamentos
                with gpio_timeout(self.gpio_connection_timeout):
                    self.pi = pigpio.pi()
                if not self.pi.connected:
                    raise ConnectionError('pigpio daemon not responding')
                if not self.pi.connected:
                    self.get_logger().error('Não foi possível conectar ao daemon pigpio. Está instalado e executando?')
                    self.get_logger().error('Execute: sudo systemctl start pigpiod')
                    self.pi = None # Falha na conexão
                else:
                    # Configurar o pino GPIO para o servo
                    self.pi.set_mode(self.servo_gpio_pin, pigpio.OUTPUT)
                    self.pi.set_PWM_frequency(self.servo_gpio_pin, self.servo_pwm_frequency)

                    # Centralizar o servo na inicialização
                    self.set_servo_angle(0.0)

                    self.get_logger().info(f'Servo inicializado no pino GPIO {self.servo_gpio_pin}')
            except Exception as e:
                self.get_logger().error(f'Erro ao inicializar pigpio: {e}')
                self.pi = None
        else:
             self.get_logger().warn('Biblioteca pigpio não disponível. O controle de servo via GPIO está desativado.')

        # Criar subscription para o tópico de drive (comandos Ackermann)
        self.drive_subscription = self.create_subscription(
            AckermannDriveStamped,
            self.drive_topic,
            self.drive_callback,
            10)

        # Se estamos usando a odometria do VESC, subscrever-se ao tópico
        if self.use_vesc_odom:
            self.odom_subscription = self.create_subscription(
                Odometry,
                self.vesc_odom_topic,
                self.odom_callback,
                10)
            self.get_logger().info(f'Subscrevendo a odometria VESC em: {self.vesc_odom_topic}')
        else:
             self.get_logger().info('Cálculo de odometria VESC desativado.')

        # Publisher para odometria no formato F1TENTH
        self.odom_publisher = self.create_publisher(
            Odometry,
            self.odom_topic,
            10)

        # Broadcaster TF para transformações
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f'Nó de controle inicializado. Publicando odometria em: {self.odom_topic}')

    def set_servo_angle(self, angle):
        """
        Converte um ângulo de direção (em radianos) para um valor de PWM e envia para o servo.

        Args:
            angle (float): Ângulo de direção em radianos
        """
        # Verifica se pigpio está disponível
        if not self.pi:
            # self.get_logger().warn('Tentativa de definir ângulo do servo, mas pigpio não está disponível.', throttle_duration_sec=5)
            return

        # Limitar o ângulo aos valores mínimo e máximo
        angle = min(max(angle, self.min_steering_angle), self.max_steering_angle)

        # Mapear o ângulo para a faixa de largura de pulso
        # Supõe que o ângulo 0 está no centro do servo (meio da faixa de PWM)
        angle_range = self.max_steering_angle - self.min_steering_angle
        pulse_range = self.servo_max_pulse_width - self.servo_min_pulse_width

        # Normalizar o ângulo para um valor entre 0 e 1, considerando min_angle como 0
        # e max_angle como 1
        if angle_range == 0:
             normalized_angle = 0.5 # Evita divisão por zero se min==max
        else:
            normalized_angle = (angle - self.min_steering_angle) / angle_range

        # Converter para largura de pulso em microssegundos
        pulse_width = self.servo_min_pulse_width + normalized_angle * pulse_range

        # Garantir que a largura de pulso esteja dentro dos limites absolutos
        pulse_width = int(min(max(pulse_width, self.servo_min_pulse_width), self.servo_max_pulse_width))

        # Aplicar ao pino GPIO usando a função de largura de pulso do pigpio
        try:
            self.pi.set_servo_pulsewidth(self.servo_gpio_pin, pulse_width)
            # self.get_logger().debug(f'Servo PWM set to: {pulse_width} µs for angle {angle:.2f} rad')
        except Exception as e:
             self.get_logger().error(f'Erro ao definir a largura de pulso do servo: {e}')

    def drive_callback(self, msg):
        """
        Processa mensagens AckermannDriveStamped do tópico /drive

        Args:
            msg (AckermannDriveStamped): Mensagem com comandos de velocidade e ângulo de direção
        """
        # O controle de velocidade já é tratado pelo vesc_ackermann
        # Aqui apenas controlamos o servo diretamente
        steering_angle = msg.drive.steering_angle

        # Ajustar o ângulo do servo via PWM
        self.set_servo_angle(steering_angle)

        # self.get_logger().debug(f'Recebido comando de direção: {steering_angle} rad')

    def odom_callback(self, msg):
        """
        Recebe a odometria do vesc_to_odom e a republica no formato F1TENTH

        Args:
            msg (Odometry): Mensagem de odometria do vesc_to_odom
        """
        # Criar uma nova mensagem de odometria no formato esperado pela F1TENTH
        f1tenth_odom = Odometry()

        # Copiar os dados da mensagem original, ajustando o timestamp e frames
        f1tenth_odom.header.stamp = self.get_clock().now().to_msg() # Usar timestamp atual
        f1tenth_odom.header.frame_id = self.odom_frame
        f1tenth_odom.child_frame_id = self.base_frame # Frame padrão F1TENTH

        # Copiar pose e twist
        f1tenth_odom.pose = msg.pose
        f1tenth_odom.twist = msg.twist

        # Publicar a odometria no tópico F1TENTH
        self.odom_publisher.publish(f1tenth_odom)

        # Publicar transformação tf correspondente
        transform = TransformStamped()
        transform.header.stamp = f1tenth_odom.header.stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame

        # Posição
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z

        # Orientação
        transform.transform.rotation = msg.pose.pose.orientation

        # Publicar transformação
        self.tf_broadcaster.sendTransform(transform)

    def destroy_node(self):
        """Libera recursos GPIO ao desligar."""
        if self.pi and self.pi.connected:
             self.get_logger().info('Desligando PWM e liberando GPIO.')
             self.pi.set_servo_pulsewidth(self.servo_gpio_pin, 0) # Desliga PWM
             self.pi.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    servo_control_node = None
    try:
        servo_control_node = ServoControlNode()
        # Só entra no loop spin se a inicialização (incluindo pigpio) foi bem sucedida
        if servo_control_node.pi is not None or not pigpio:
             rclpy.spin(servo_control_node)
        else:
             servo_control_node.get_logger().error("Falha na inicialização do nó devido a problema com pigpio.")

    except KeyboardInterrupt:
         print("Nó interrompido pelo usuário.")
    except Exception as e:
        print(f'Erro inesperado no nó de controle do servo: {e}')
    finally:
        # Desligar corretamente
        if servo_control_node:
             servo_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
