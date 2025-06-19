#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import TransformStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# Importação condicional do pigpio
try:
    import pigpio
    PIGPIO_AVAILABLE = True
except ImportError:
    print("\n*** Aviso: pigpio não disponível. Controle GPIO desativado. ***")
    pigpio = None
    PIGPIO_AVAILABLE = False

import math
import time
import threading
import signal
import contextlib
import collections  # re-adicionado para uso de deque e outros
from enum import Enum
from dataclasses import dataclass
from concurrent.futures import ThreadPoolExecutor
from tf2_ros import TransformBroadcaster

class VehicleState(Enum):
    """Estados do veículo para máquina de estados"""
    INITIALIZING = "initializing"
    READY = "ready"
    DRIVING = "driving"
    EMERGENCY_STOP = "emergency_stop"
    ERROR = "error"

@dataclass
class PIDController:
    """Controlador PID para servo motor"""
    kp: float = 0.8
    ki: float = 0.1
    kd: float = 0.05
    integral: float = 0.0
    prev_error: float = 0.0
    max_integral: float = 0.5  # Anti-windup

    def compute(self, error: float, dt: float) -> float:
        """Calcula saída do controlador PID"""
        if dt <= 0:
            return 0.0

        # Termo integral com anti-windup
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)

        # Termo derivativo
        derivative = (error - self.prev_error) / dt

        # Saída PID
        output = (self.kp * error +
                 self.ki * self.integral +
                 self.kd * derivative)

        self.prev_error = error
        return output

    def reset(self):
        """Reset do controlador"""
        self.integral = 0.0
        self.prev_error = 0.0

class StateManager:
    """Gerenciador de estados do veículo com failsafe"""

    def __init__(self, node):
        self.state = VehicleState.INITIALIZING
        self.node = node
        self.last_command_time = time.time()
        self.command_timeout = 1.0  # segundos
        self.emergency_stop_active = False
        self.lock = threading.RLock()

    def update_command_timestamp(self):
        """Atualiza timestamp do último comando recebido"""
        self.last_command_time = time.time()

    def check_heartbeat(self):
        """Verifica timeout de comandos e ativa emergência se necessário"""
        if (time.time() - self.last_command_time) > self.command_timeout:
            if not self.emergency_stop_active:
                self.transition_to_emergency_stop()

    def transition_to_emergency_stop(self):
        """Ativa modo de parada de emergência"""
        with self.lock:
            old_state = self.state
            self.state = VehicleState.EMERGENCY_STOP
            self.emergency_stop_active = True

        self.node.get_logger().warn(f'Estado alterado: {old_state.value} -> {self.state.value}')

        # Centralizar servo e parar
        self.node.emergency_stop()

    def transition_to_ready(self):
        """Transição para estado pronto"""
        with self.lock:
            old_state = self.state
            self.state = VehicleState.READY
            self.emergency_stop_active = False
        self.node.get_logger().info(f'Estado alterado: {old_state.value} -> {self.state.value}')

    def can_drive(self) -> bool:
        """Verifica se o veículo pode dirigir"""
        with self.lock:
            return self.state in [VehicleState.READY, VehicleState.DRIVING]

# Adicionar após imports: contexto para timeout GPIO
@contextlib.contextmanager
def gpio_timeout(timeout_sec):
    """Context manager para timeout na conexão GPIO com pigpio"""
    def _handler(signum, frame):
        raise TimeoutError(f"GPIO connection timeout after {timeout_sec}s")
    old_handler = signal.signal(signal.SIGALRM, _handler)
    signal.alarm(int(timeout_sec))
    try:
        yield
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, old_handler)

class EnhancedServoControlNode(Node):
    """
    Nó avançado para controle integrado VESC/Servo com:
    - Controle PID suavizado
    - Máquina de estados com failsafe
    - Processamento assíncrono
    - Diagnósticos avançados
    """

    def __init__(self):
        super().__init__('enhanced_servo_control_node')

        # Configuração de QoS para baixa latência
        self.low_latency_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Declarar parâmetros avançados
        self._declare_parameters()
        self._load_parameters()

        # Estado interno thread-safe
        self.angle_lock = threading.RLock()
        self._current_angle = 0.0
        self._target_angle = 0.0
        self.last_control_time = time.time()

        # Controlador PID
        self.pid_controller = PIDController(
            kp=self.get_parameter('pid_kp').value,
            ki=self.get_parameter('pid_ki').value,
            kd=self.get_parameter('pid_kd').value
        )

        # Gerenciador de estados
        self.state_manager = StateManager(self)

        # Buffer de comandos thread-safe (dimensão aumentada para evitar overflow em 100Hz)
        self.command_buffer = collections.deque(maxlen=150)
        self.buffer_lock = threading.Lock()

        # Pool de threads para processamento
        self.executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="servo_ctrl")

        # Inicializar pigpio se disponível
        self.pi = None
        self._initialize_gpio()

        # Configurar publishers/subscribers
        self._setup_communication()

        # Timers
        self.control_timer = self.create_timer(1.0/self.control_frequency, self.control_loop)
        self.diagnostic_timer = self.create_timer(1.0, self.publish_diagnostics)
        self.heartbeat_timer = self.create_timer(0.1, self.state_manager.check_heartbeat)

        # Estatísticas
        self.stats = {
            'commands_received': 0,
            'control_cycles': 0,
            'pid_corrections': 0,
            'emergency_stops': 0
        }

        self.get_logger().info('Nó de controle avançado inicializado')
        self.state_manager.transition_to_ready()

    def _declare_parameters(self):
        """Declarar todos os parâmetros do nó"""
        parameters = [
            # GPIO e PWM
            ('servo_gpio_pin', 18),
            ('servo_pwm_frequency', 50),
            ('servo_min_pulse_width', 1000),
            ('servo_max_pulse_width', 2000),
            ('gpio_connection_timeout', 5.0),  # Timeout (s) para conexão pigpio

            # Controle
            ('max_steering_angle', 0.4),
            ('min_steering_angle', -0.4),
            ('control_frequency', 100.0),  # Hz
            ('angle_tolerance', 0.01),     # rad

            # PID
            ('pid_kp', 0.8),
            ('pid_ki', 0.1),
            ('pid_kd', 0.05),
            ('enable_pid', True),

            # Frames e tópicos
            ('odom_frame', 'odom'),
            ('base_frame', 'base_link'),
            ('drive_topic', '/drive'),
            ('odom_topic', '/ego_racecar/odom'),
            ('use_vesc_odom', True),
            ('vesc_odom_topic', '/odom'),

            # Segurança
            ('command_timeout', 1.0),
            ('max_angular_velocity', 2.0),  # rad/s
            ('enable_safety_limits', True)
        ]

        self.declare_parameters(namespace='', parameters=parameters)

    def _load_parameters(self):
        """Carregar parâmetros para variáveis locais"""
        # GPIO
        self.servo_gpio_pin = self.get_parameter('servo_gpio_pin').value
        self.servo_pwm_frequency = self.get_parameter('servo_pwm_frequency').value
        self.servo_min_pulse_width = self.get_parameter('servo_min_pulse_width').value
        self.servo_max_pulse_width = self.get_parameter('servo_max_pulse_width').value
        # GPIO timeout
        self.gpio_connection_timeout = self.get_parameter('gpio_connection_timeout').value

        # Controle
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.min_steering_angle = self.get_parameter('min_steering_angle').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value

        # Frames
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.use_vesc_odom = self.get_parameter('use_vesc_odom').value
        self.vesc_odom_topic = self.get_parameter('vesc_odom_topic').value

        # Segurança
        self.command_timeout = self.get_parameter('command_timeout').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.enable_safety_limits = self.get_parameter('enable_safety_limits').value
        self.enable_pid = self.get_parameter('enable_pid').value

    def _initialize_gpio(self):
        """Inicializar conexão pigpio com tratamento robusto de erros"""
        if not PIGPIO_AVAILABLE:
            self.get_logger().warn('pigpio não disponível - controle GPIO desativado')
            return

        try:
            # Conectar com timeout
            with gpio_timeout(self.gpio_connection_timeout):
                self.pi = pigpio.pi()
            # Verificação de conexão
            if not self.pi or not self.pi.connected:
                self.get_logger().error('Falha na conexão pigpio - daemon não está rodando?')
                self.pi = None
                return

            # Configurar GPIO
            self.pi.set_mode(self.servo_gpio_pin, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(self.servo_gpio_pin, self.servo_pwm_frequency)

            # Centralizar servo
            self.set_servo_angle(0.0, force=True)

            self.get_logger().info(f'GPIO inicializado - pino {self.servo_gpio_pin}')

        except Exception as e:
            self.get_logger().error(f'Erro na inicialização GPIO: {e}')
            self.pi = None

    def _setup_communication(self):
        """Configurar publishers e subscribers com QoS otimizado"""
        # Subscriber para comandos de direção
        self.drive_subscription = self.create_subscription(
            AckermannDriveStamped,
            self.drive_topic,
            self.drive_callback,
            qos_profile=self.low_latency_qos
        )

        # Subscriber para odometria VESC (se habilitado)
        if self.use_vesc_odom:
            self.odom_subscription = self.create_subscription(
                Odometry,
                self.vesc_odom_topic,
                self.odom_callback,
                10
            )

        # Publishers
        self.odom_publisher = self.create_publisher(Odometry, self.odom_topic, 10)
        self.diagnostic_publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.emergency_stop_publisher = self.create_publisher(Bool, '/emergency_stop', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def set_servo_angle(self, angle: float, force: bool = False):
        """
        Definir ângulo do servo com validação e limites de segurança

        Args:
            angle: Ângulo em radianos
            force: Ignorar verificações de estado (para emergência)
        """
        if not self.pi:
            return False

        # Verificar estado (exceto em força/emergência)
        if not force and not self.state_manager.can_drive():
            return False

        # Aplicar limites de segurança
        if self.enable_safety_limits:
            angle = max(min(angle, self.max_steering_angle), self.min_steering_angle)

            # Verificar velocidade angular máxima
            dt = time.time() - self.last_control_time
            if dt > 0:
                angular_velocity = abs(angle - self.current_angle) / dt
                if angular_velocity > self.max_angular_velocity:
                    # Limitar mudança de ângulo
                    max_delta = self.max_angular_velocity * dt
                    if angle > self.current_angle:
                        angle = self.current_angle + max_delta
                    else:
                        angle = self.current_angle - max_delta

        # Converter para PWM
        angle_range = self.max_steering_angle - self.min_steering_angle
        if angle_range == 0:
            normalized_angle = 0.5
        else:
            normalized_angle = (angle - self.min_steering_angle) / angle_range

        pulse_width = (self.servo_min_pulse_width +
                      normalized_angle * (self.servo_max_pulse_width - self.servo_min_pulse_width))
        pulse_width = int(max(min(pulse_width, self.servo_max_pulse_width), self.servo_min_pulse_width))

        # Aplicar PWM
        try:
            self.pi.set_servo_pulsewidth(self.servo_gpio_pin, pulse_width)
            with self.angle_lock:
                self._current_angle = angle
            self.last_control_time = time.time()
            return True
        except Exception as e:
            self.get_logger().error(f'Erro ao definir PWM: {e}')
            return False

    def drive_callback(self, msg: AckermannDriveStamped):
        """Callback otimizado para comandos de direção"""
        self.stats['commands_received'] += 1
        self.state_manager.update_command_timestamp()

        # Adicionar ao buffer thread-safe
        with self.buffer_lock:
            if len(self.command_buffer) > self.command_buffer.maxlen * 0.9:
                self.get_logger().warn(f"Command buffer near capacity: {len(self.command_buffer)}/{self.command_buffer.maxlen}")
            self.command_buffer.append(msg)

        # Atualizar estado
        if self.state_manager.state == VehicleState.READY:
            self.state_manager.state = VehicleState.DRIVING

    def control_loop(self):
        """Loop de controle de alta frequência"""
        self.stats['control_cycles'] += 1

        # Processar comandos do buffer
        if not self.command_buffer:
            return

        with self.buffer_lock:
            if self.command_buffer:
                latest_cmd = self.command_buffer[-1]
                self.command_buffer.clear()
            else:
                return

        # Processar comando assincronamente
        self.executor.submit(self._process_command, latest_cmd)

    def _process_command(self, msg: AckermannDriveStamped):
        """Processar comando de direção com controle PID"""
        target_angle = msg.drive.steering_angle

        if self.enable_pid:
            # Usar controlador PID para movimento suave
            error = target_angle - self.current_angle

            if abs(error) > self.angle_tolerance:
                dt = time.time() - self.last_control_time
                pid_output = self.pid_controller.compute(error, dt)

                # Aplicar correção PID
                corrected_angle = self.current_angle + pid_output
                self.set_servo_angle(corrected_angle)
                self.stats['pid_corrections'] += 1
            else:
                # Erro dentro da tolerância
                with self.angle_lock:
                    self._target_angle = target_angle
        else:
            # Controle direto sem PID
            self.set_servo_angle(target_angle)

    def emergency_stop(self):
        """Parada de emergência"""
        self.stats['emergency_stops'] += 1
        self.get_logger().warn('PARADA DE EMERGÊNCIA ATIVADA')

        # Centralizar servo
        self.set_servo_angle(0.0, force=True)

        # Publicar sinal de emergência
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_stop_publisher.publish(emergency_msg)

        # Reset PID
        self.pid_controller.reset()

    def odom_callback(self, msg: Odometry):
        """Republicar odometria no formato F1TENTH"""
        f1tenth_odom = Odometry()

        # Header
        f1tenth_odom.header.stamp = self.get_clock().now().to_msg()
        f1tenth_odom.header.frame_id = self.odom_frame
        f1tenth_odom.child_frame_id = self.base_frame

        # Dados
        f1tenth_odom.pose = msg.pose
        f1tenth_odom.twist = msg.twist

        # Publicar
        self.odom_publisher.publish(f1tenth_odom)

        # TF
        transform = TransformStamped()
        transform.header = f1tenth_odom.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)

    def publish_diagnostics(self):
        """Publicar informações de diagnóstico"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # Status do servo
        servo_status = DiagnosticStatus()
        servo_status.name = "servo_control"
        servo_status.hardware_id = f"gpio_{self.servo_gpio_pin}"

        if self.pi and self.pi.connected:
            servo_status.level = DiagnosticStatus.OK
            servo_status.message = "Operacional"
        else:
            servo_status.level = DiagnosticStatus.ERROR
            servo_status.message = "GPIO não disponível"

        servo_status.values = [
            KeyValue(key="current_angle", value=str(self.current_angle)),
            KeyValue(key="target_angle", value=str(self.target_angle)),
            KeyValue(key="state", value=self.state_manager.state.value),
            KeyValue(key="commands_received", value=str(self.stats['commands_received'])),
            KeyValue(key="pid_corrections", value=str(self.stats['pid_corrections']))
        ]

        diag_array.status.append(servo_status)
        self.diagnostic_publisher.publish(diag_array)

    def destroy_node(self):
        """Cleanup com shutdown graceful"""
        self.get_logger().info('Desligando nó...')

        # Parar executor
        self.executor.shutdown(wait=True)

        # Liberar GPIO
        if self.pi and self.pi.connected:
            self.pi.set_servo_pulsewidth(self.servo_gpio_pin, 0)
            self.pi.stop()

        super().destroy_node()

    @property
    def current_angle(self):
        """Getter thread-safe para current_angle"""
        with self.angle_lock:
            return self._current_angle

    @current_angle.setter
    def current_angle(self, value):
        """Setter thread-safe para current_angle"""
        with self.angle_lock:
            self._current_angle = value

    @property
    def target_angle(self):
        """Getter thread-safe para target_angle"""
        with self.angle_lock:
            return self._target_angle

    @target_angle.setter
    def target_angle(self, value):
        """Setter thread-safe para target_angle"""
        with self.angle_lock:
            self._target_angle = value

def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = EnhancedServoControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Nó interrompido pelo usuário")
    except Exception as e:
        print(f"Erro inesperado: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
