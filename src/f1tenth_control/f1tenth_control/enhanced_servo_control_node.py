#!/usr/bin/env python3
"""
Nó avançado para controle integrado VESC/Servo.

- Controle PID suavizado
- Máquina de estados com failsafe
- Processamento assíncrono
- Diagnósticos avançados
- Recuperação de falhas
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.exceptions import ParameterNotDeclaredException
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import TransformStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from enum import Enum
from dataclasses import dataclass
from concurrent.futures import ThreadPoolExecutor
from typing import Optional, TYPE_CHECKING, cast
from tf2_ros import TransformBroadcaster

if TYPE_CHECKING:
    from enhanced_servo_control_node import EnhancedServoControlNode

# Importação condicional do pigpio
try:
    import pigpio

    PIGPIO_AVAILABLE = True
except ImportError:
    print("\n*** Aviso: pigpio não disponível. Controle GPIO desativado. ***")
    pigpio = None
    PIGPIO_AVAILABLE = False

import time
import threading
import signal
import contextlib
import collections
from enum import Enum
from dataclasses import dataclass
from concurrent.futures import ThreadPoolExecutor
from tf2_ros import TransformBroadcaster


class VehicleState(Enum):
    """Estados do veículo para máquina de estados."""

    INITIALIZING = "initializing"
    READY = "ready"
    DRIVING = "driving"
    EMERGENCY_STOP = "emergency_stop"
    ERROR = "error"


@dataclass
class PIDController:
    """Controlador PID para servo motor."""

    kp: float = 0.8
    ki: float = 0.1
    kd: float = 0.05
    integral: float = 0.0
    prev_error: float = 0.0
    max_integral: float = 0.5  # Anti-windup

    def compute(self, error: float, dt: float) -> float:
        """
        Calcula a saída do controlador PID.

        Args:
            error: Diferença entre o valor desejado e o atual.
            dt: Intervalo de tempo desde o último cálculo.

        Returns:
            Valor de correção calculado.
        """
        if dt <= 0:
            return 0.0

        # Termo integral com anti-windup
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)

        # Termo derivativo
        derivative = (error - self.prev_error) / dt

        # Saída PID
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.prev_error = error
        return output

    def reset(self):
        """Reseta o estado do controlador PID."""
        self.integral = 0.0
        self.prev_error = 0.0


class StateManager:
    """Gerenciador de estados do veículo com failsafe."""

    def __init__(self, node: "EnhancedServoControlNode", command_timeout: float):
        self.state = VehicleState.INITIALIZING
        self.node = node
        self.last_command_time = time.time()
        self.command_timeout = command_timeout
        self.emergency_stop_active = False
        self.lock = threading.RLock()

    def update_command_timestamp(self):
        """Atualiza timestamp do último comando recebido."""
        self.last_command_time = time.time()

    def check_heartbeat(self):
        """Verifica timeout de comandos e ativa emergência se necessário."""
        if (time.time() - self.last_command_time) > self.command_timeout:
            if not self.emergency_stop_active:
                self.node.get_logger().warn(
                    f"Timeout de comando ({self.command_timeout}s). "
                    "Ativando parada de emergência."
                )
                self.transition_to_emergency_stop()

    def transition_to_emergency_stop(self):
        """Ativa modo de parada de emergência."""
        with self.lock:
            if self.state == VehicleState.EMERGENCY_STOP:
                return
            old_state = self.state
            self.state = VehicleState.EMERGENCY_STOP
            self.emergency_stop_active = True

        self.node.get_logger().warn(
            f"Estado alterado: {old_state.value} -> {self.state.value}"
        )

        # Centralizar servo e parar
        self.node.emergency_stop()

    def transition_to_ready(self):
        """Transição para estado pronto."""
        with self.lock:
            old_state = self.state
            self.state = VehicleState.READY
            self.emergency_stop_active = False
        self.node.get_logger().info(
            f"Estado alterado: {old_state.value} -> {self.state.value}"
        )

    def can_drive(self) -> bool:
        """Verifica se o veículo pode dirigir."""
        with self.lock:
            return self.state in [VehicleState.READY, VehicleState.DRIVING]


@contextlib.contextmanager
def gpio_timeout(timeout_sec: float):
    """Context manager para timeout na conexão GPIO com pigpio."""

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
        super().__init__("enhanced_servo_control_node")

        # Configuração de QoS para baixa latência
        self.low_latency_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Declarar parâmetros e carregá-los em variáveis tipadas
        self._declare_parameters()
        self._load_parameters()  # Falha se os parâmetros estiverem ausentes

        # Estado interno thread-safe
        self.angle_lock = threading.RLock()
        self._current_angle = 0.0
        self._target_angle = 0.0
        self.last_control_time = time.time()

        # Controlador PID
        self.pid_controller = PIDController(
            kp=self.pid_kp, ki=self.pid_ki, kd=self.pid_kd
        )

        # Gerenciador de estados
        self.state_manager = StateManager(self, self.command_timeout)

        # Buffer de comandos thread-safe
        self.command_buffer = collections.deque(maxlen=150)
        self.buffer_lock = threading.Lock()

        # Pool de threads para processamento  
        self.thread_executor = ThreadPoolExecutor(  # type: ignore
            max_workers=2, thread_name_prefix="servo_ctrl"
        )

        # Inicializar pigpio se disponível
        self.pi: Optional["pigpio.pi"] = None
        self._initialize_gpio()

        # Configurar publishers/subscribers
        self._setup_communication()

        # Timers
        timer_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(timer_period, self.control_loop)
        self.diagnostic_timer = self.create_timer(1.0, self.publish_diagnostics)
        self.heartbeat_timer = self.create_timer(
            0.1, self.state_manager.check_heartbeat
        )

        # Estatísticas
        self.stats = {
            "commands_received": 0,
            "control_cycles": 0,
            "pid_corrections": 0,
            "emergency_stops": 0,
            "buffer_overflows": 0,
        }

        self.get_logger().info("Nó de controle avançado inicializado")
        self.state_manager.transition_to_ready()

    def _declare_parameters(self):
        """Declarar todos os parâmetros do nó."""
        self.get_logger().info("Declarando parâmetros...")
        params = [
            # GPIO e PWM
            ("servo_gpio_pin", 18),
            ("servo_pwm_frequency", 50),
            ("servo_min_pulse_width", 1000),
            ("servo_max_pulse_width", 2000),
            ("gpio_connection_timeout", 5.0),
            # Controle
            ("max_steering_angle", 0.4),
            ("min_steering_angle", -0.4),
            ("control_frequency", 100.0),
            ("angle_tolerance", 0.01),
            # PID
            ("pid_kp", 0.8),
            ("pid_ki", 0.1),
            ("pid_kd", 0.05),
            ("enable_pid", True),
            # Frames e tópicos
            ("odom_frame", "odom"),
            ("base_frame", "base_link"),
            ("drive_topic", "/drive"),
            ("odom_topic", "/ego_racecar/odom"),
            ("use_vesc_odom", True),
            ("vesc_odom_topic", "/odom"),
            # Segurança
            ("command_timeout", 1.0),
            ("max_angular_velocity", 2.0),
            ("enable_safety_limits", True),
            # TF Publishing Control
            ("publish_tf", False),
        ]
        self.declare_parameters(namespace="", parameters=params)

    def _load_parameters(self):
        """
        Carrega os parâmetros em variáveis de membro tipadas.
        Falha rapidamente se um parâmetro não estiver definido.
        """
        self.get_logger().info("Carregando e validando parâmetros...")
        try:
            # GPIO
            self.servo_gpio_pin: int = cast(
                int, self.get_parameter("servo_gpio_pin").value
            )
            self.servo_pwm_frequency: int = cast(
                int, self.get_parameter("servo_pwm_frequency").value
            )
            self.servo_min_pulse_width: int = cast(
                int, self.get_parameter("servo_min_pulse_width").value
            )
            self.servo_max_pulse_width: int = cast(
                int, self.get_parameter("servo_max_pulse_width").value
            )
            self.gpio_connection_timeout: float = cast(
                float, self.get_parameter("gpio_connection_timeout").value
            )

            # Controle
            self.max_steering_angle: float = cast(
                float, self.get_parameter("max_steering_angle").value
            )
            self.min_steering_angle: float = cast(
                float, self.get_parameter("min_steering_angle").value
            )
            self.control_frequency: float = cast(
                float, self.get_parameter("control_frequency").value
            )
            self.angle_tolerance: float = cast(
                float, self.get_parameter("angle_tolerance").value
            )

            # PID
            self.pid_kp: float = cast(float, self.get_parameter("pid_kp").value)
            self.pid_ki: float = cast(float, self.get_parameter("pid_ki").value)
            self.pid_kd: float = cast(float, self.get_parameter("pid_kd").value)
            self.enable_pid: bool = cast(bool, self.get_parameter("enable_pid").value)

            # Frames e Tópicos
            self.odom_frame: str = cast(str, self.get_parameter("odom_frame").value)
            self.base_frame: str = cast(str, self.get_parameter("base_frame").value)
            self.drive_topic: str = cast(str, self.get_parameter("drive_topic").value)
            self.odom_topic: str = cast(str, self.get_parameter("odom_topic").value)
            self.use_vesc_odom: bool = cast(
                bool, self.get_parameter("use_vesc_odom").value
            )
            self.vesc_odom_topic: str = cast(
                str, self.get_parameter("vesc_odom_topic").value
            )

            # Segurança
            self.command_timeout: float = cast(
                float, self.get_parameter("command_timeout").value
            )
            self.max_angular_velocity: float = cast(
                float, self.get_parameter("max_angular_velocity").value
            )
            self.enable_safety_limits: bool = cast(
                bool, self.get_parameter("enable_safety_limits").value
            )

            # TF Publishing Control
            self.publish_tf: bool = cast(
                bool, self.get_parameter("publish_tf").value
            )

        except ParameterNotDeclaredException as e:
            self.get_logger().fatal(f"Parâmetro crítico não declarado: {e}")
            raise e

    def _initialize_gpio(self):
        """Inicializar conexão pigpio com tratamento robusto de erros."""
        if not PIGPIO_AVAILABLE:
            self.get_logger().warn("pigpio não disponível - controle GPIO desativado")
            return

        try:
            self.get_logger().info("Tentando conectar ao daemon pigpio...")
            with gpio_timeout(self.gpio_connection_timeout):
                self.pi = pigpio.pi()  # type: ignore

            if not self.pi or not self.pi.connected:
                self.get_logger().error(
                    "Falha na conexão pigpio - daemon não está rodando?"
                )
                self.pi = None
                return

            self.pi.set_mode(self.servo_gpio_pin, pigpio.OUTPUT)  # type: ignore
            self.pi.set_PWM_frequency(self.servo_gpio_pin, self.servo_pwm_frequency)
            self.set_servo_angle(0.0, force=True)
            self.get_logger().info(f"GPIO inicializado - pino {self.servo_gpio_pin}")

        except Exception as e:
            self.get_logger().error(f"Erro na inicialização GPIO: {e}")
            self.pi = None

    def _setup_communication(self):
        """Configurar publishers, subscribers e services."""
        # Subscribers
        self.drive_subscription = self.create_subscription(
            AckermannDriveStamped,
            self.drive_topic,
            self.drive_callback,
            qos_profile=self.low_latency_qos,
        )
        if self.use_vesc_odom:
            self.odom_subscription = self.create_subscription(
                Odometry, self.vesc_odom_topic, self.odom_callback, 10
            )

        # Publishers
        self.odom_publisher = self.create_publisher(Odometry, self.odom_topic, 10)
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self.emergency_stop_publisher = self.create_publisher(
            Bool, "/emergency_stop", 10
        )

        # Services
        self.reset_service = self.create_service(
            Trigger, "~/reset_emergency_stop", self.reset_emergency_callback
        )

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def set_servo_angle(self, angle: float, force: bool = False) -> bool:
        """
        Define o ângulo do servo com validação e limites de segurança.

        Args:
            angle: Ângulo em radianos.
            force: Ignora verificações de estado (para emergência/inicialização).

        Returns:
            True se o ângulo foi definido com sucesso, False caso contrário.
        """
        if not self.pi or not self.pi.connected:
            return False

        if not force and not self.state_manager.can_drive():
            return False

        # Aplicar limites de segurança
        if self.enable_safety_limits:
            angle = max(min(angle, self.max_steering_angle), self.min_steering_angle)

            # Limitar velocidade angular máxima
            dt = time.time() - self.last_control_time
            if dt > 0.001:  # Evitar divisão por zero
                angular_velocity = abs(angle - self.current_angle) / dt
                if angular_velocity > self.max_angular_velocity:
                    max_delta = self.max_angular_velocity * dt
                    angle = (
                        self.current_angle + max_delta
                        if angle > self.current_angle
                        else self.current_angle - max_delta
                    )

        # Converter para PWM
        angle_range = self.max_steering_angle - self.min_steering_angle
        normalized_angle = (
            (angle - self.min_steering_angle) / angle_range if angle_range != 0 else 0.5
        )

        pulse_width = self.servo_min_pulse_width + (
            normalized_angle * (self.servo_max_pulse_width - self.servo_min_pulse_width)
        )
        pulse_width = int(
            max(
                min(pulse_width, self.servo_max_pulse_width), self.servo_min_pulse_width
            )
        )

        # Aplicar PWM
        try:
            self.pi.set_servo_pulsewidth(self.servo_gpio_pin, pulse_width)
            with self.angle_lock:
                self._current_angle = angle
            self.last_control_time = time.time()
            return True
        except Exception as e:
            self.get_logger().error(f"Erro ao definir PWM: {e}")
            return False

    def drive_callback(self, msg: AckermannDriveStamped):
        """
        Callback otimizado para comandos de direção, armazena no buffer.
        """
        self.stats["commands_received"] += 1
        self.state_manager.update_command_timestamp()

        with self.buffer_lock:
            if (
                self.command_buffer.maxlen is not None
                and len(self.command_buffer) >= self.command_buffer.maxlen
            ):
                self.get_logger().warn(
                    "Buffer de comando cheio, descartando comando antigo."
                )
                self.stats["buffer_overflows"] += 1
            else:
                self.command_buffer.append(msg)

        if self.state_manager.state == VehicleState.READY:
            self.state_manager.state = VehicleState.DRIVING

    def control_loop(self):
        """
        Loop de controle de alta frequência.
        Pega o comando mais recente do buffer e o processa.
        """
        self.stats["control_cycles"] += 1

        if not self.command_buffer:
            return

        with self.buffer_lock:
            if self.command_buffer:
                latest_cmd = self.command_buffer[-1]
                self.command_buffer.clear()
            else:
                return

        self.thread_executor.submit(self._process_command, latest_cmd)  # type: ignore

    def _process_command(self, msg: AckermannDriveStamped):
        """
        Processa um comando de direção em uma thread separada, usando PID.
        """
        target_angle = msg.drive.steering_angle

        if self.enable_pid:
            error = target_angle - self.current_angle
            if abs(error) > self.angle_tolerance:
                dt = time.time() - self.last_control_time
                pid_output = self.pid_controller.compute(error, dt)
                corrected_angle = self.current_angle + pid_output
                self.set_servo_angle(corrected_angle)
                self.stats["pid_corrections"] += 1
        else:
            self.set_servo_angle(target_angle)

    def emergency_stop(self):
        """Ativa a parada de emergência: centraliza servo e publica estado."""
        self.stats["emergency_stops"] += 1
        self.get_logger().error("PARADA DE EMERGÊNCIA ATIVADA")
        self.set_servo_angle(0.0, force=True)
        self.emergency_stop_publisher.publish(Bool(data=True))
        self.pid_controller.reset()

    def reset_emergency_callback(self, request, response):
        """Callback para o serviço de reset da parada de emergência."""
        self.get_logger().info("Pedido para resetar parada de emergência.")
        with self.state_manager.lock:
            if self.state_manager.state == VehicleState.EMERGENCY_STOP:
                self.state_manager.transition_to_ready()
                self.set_servo_angle(0.0, force=True)  # Re-centraliza
                response.success = True
                response.message = "Parada de emergência resetada. Sistema PRONTO."
            else:
                response.success = False
                response.message = (
                    f"O sistema não está em emergência "
                    f"(estado: {self.state_manager.state.value})."
                )
        return response

    def odom_callback(self, msg: Odometry):
        """Republica odometria no formato F1TENTH e envia TF condicionalmente."""
        f1tenth_odom = Odometry()
        f1tenth_odom.header.stamp = self.get_clock().now().to_msg()
        f1tenth_odom.header.frame_id = self.odom_frame
        f1tenth_odom.child_frame_id = self.base_frame
        f1tenth_odom.pose = msg.pose
        f1tenth_odom.twist = msg.twist
        self.odom_publisher.publish(f1tenth_odom)

        # ✅ CORREÇÃO TF_SELF_TRANSFORM: Condicionar TF publishing
        if self.publish_tf:
            transform = TransformStamped()
            transform.header = f1tenth_odom.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = msg.pose.pose.position.x
            transform.transform.translation.y = msg.pose.pose.position.y
            transform.transform.translation.z = msg.pose.pose.position.z
            transform.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def publish_diagnostics(self):
        """Publica um array de diagnósticos sobre o estado do nó."""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        servo_status = DiagnosticStatus(name="servo_control")
        servo_status.hardware_id = f"gpio_{self.servo_gpio_pin}"

        if self.pi and self.pi.connected:
            servo_status.level = DiagnosticStatus.OK
            servo_status.message = "Operacional"
        else:
            servo_status.level = DiagnosticStatus.ERROR
            servo_status.message = "GPIO não disponível ou desconectado"
            if self.state_manager.state != VehicleState.EMERGENCY_STOP:
                self.get_logger().error(
                    "Conexão pigpio perdida! Ativando parada de emergência."
                )
                self.state_manager.transition_to_emergency_stop()

        servo_status.values = [
            KeyValue(key="state", value=self.state_manager.state.value),
            KeyValue(key="current_angle_rad", value=f"{self.current_angle:.3f}"),
            KeyValue(key="target_angle_rad", value=f"{self.target_angle:.3f}"),
            KeyValue(
                key="commands_received", value=str(self.stats["commands_received"])
            ),
            KeyValue(key="pid_corrections", value=str(self.stats["pid_corrections"])),
            KeyValue(key="emergency_stops", value=str(self.stats["emergency_stops"])),
            KeyValue(key="buffer_overflows", value=str(self.stats["buffer_overflows"])),
        ]

        diag_array.status.append(servo_status)  # type: ignore
        self.diagnostic_publisher.publish(diag_array)

    def destroy_node(self):
        """Realiza cleanup antes de desligar."""
        self.get_logger().info("Desligando nó de controle do servo...")
        self.thread_executor.shutdown(wait=True)
        if self.pi and self.pi.connected:
            self.pi.set_servo_pulsewidth(self.servo_gpio_pin, 0)
            self.pi.stop()
        super().destroy_node()

    @property
    def current_angle(self) -> float:
        """Getter thread-safe para _current_angle."""
        with self.angle_lock:
            return self._current_angle

    @property
    def target_angle(self) -> float:
        """Getter thread-safe para _target_angle (lido pelo PID)."""
        with self.angle_lock:
            return self._target_angle


def main(args=None):
    """Função principal para iniciar o nó."""
    rclpy.init(args=args)
    node = None
    try:
        node = EnhancedServoControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Nó interrompido pelo usuário.")
    except Exception as e:
        if node:
            node.get_logger().fatal(f"Erro fatal não tratado: {e}")
        else:
            print(f"Erro fatal antes da inicialização do logger: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
