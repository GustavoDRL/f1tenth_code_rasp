#!/usr/bin/env python3
"""
Fixtures centralizadas para testes do sistema F1tenth.

Este módulo contém todas as fixtures reutilizáveis para mocking
de componentes ROS2, hardware e dados de teste.

Autor: Professor PhD em Engenharia Robótica
"""

import pytest
import time
import threading
from unittest.mock import MagicMock, patch, Mock
from typing import Dict, Any, List, Optional
from dataclasses import dataclass

# Imports ROS2 (com fallback para testes sem ROS)
try:
    import rclpy
    from rclpy.node import Node
    from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
    from sensor_msgs.msg import Joy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

from .mock_pigpio import MockPigpio, create_mock_pi


@dataclass
class TestConfig:
    """Configuração padrão para testes."""
    servo_gpio_pin: int = 18
    servo_pwm_frequency: int = 50
    servo_min_pulse_width: int = 1000
    servo_max_pulse_width: int = 2000
    max_steering_angle: float = 0.4
    min_steering_angle: float = -0.4
    control_frequency: float = 100.0
    max_speed: float = 7.0
    max_angle: float = 0.32


@pytest.fixture
def test_config():
    """Fixture com configuração padrão de teste."""
    return TestConfig()


@pytest.fixture
def mock_pigpio():
    """Fixture para mock do pigpio conectado."""
    return create_mock_pi(connected=True)


@pytest.fixture
def mock_pigpio_disconnected():
    """Fixture para mock do pigpio desconectado."""
    return create_mock_pi(connected=False)


@pytest.fixture
def patch_pigpio_import():
    """Fixture para substituir importação do pigpio."""
    with patch.dict('sys.modules', {'pigpio': MagicMock()}):
        yield


@pytest.fixture
def mock_ros_node():
    """Fixture para mock de um nó ROS2."""
    node = MagicMock(spec=Node if ROS_AVAILABLE else object)
    node.get_logger.return_value = MagicMock()
    node.create_subscription.return_value = MagicMock()
    node.create_publisher.return_value = MagicMock()
    node.create_timer.return_value = MagicMock()
    node.declare_parameters.return_value = None
    node.get_parameter.side_effect = lambda name: MockParameter(name)
    return node


class MockParameter:
    """Mock para parâmetros ROS2."""

    def __init__(self, name: str):
        self.name = name
        # Valores padrão baseados no nome do parâmetro
        self.value = self._get_default_value(name)

    def _get_default_value(self, name: str):
        """Retorna valor padrão baseado no nome do parâmetro."""
        defaults = {
            'servo_gpio_pin': 18,
            'servo_pwm_frequency': 50,
            'servo_min_pulse_width': 1000,
            'servo_max_pulse_width': 2000,
            'max_steering_angle': 0.4,
            'min_steering_angle': -0.4,
            'control_frequency': 100.0,
            'max_speed': 7.0,
            'max_angle': 0.32,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'drive_topic': '/drive',
            'use_vesc_odom': True,
            'pid_kp': 0.8,
            'pid_ki': 0.1,
            'pid_kd': 0.05
        }
        return defaults.get(name, 0.0)


@pytest.fixture
def sample_ackermann_msg():
    """Fixture com mensagem Ackermann de exemplo."""
    if not ROS_AVAILABLE:
        # Mock para quando ROS não está disponível
        msg = MagicMock()
        msg.drive.speed = 2.0
        msg.drive.steering_angle = 0.2
        return msg

    msg = AckermannDriveStamped()
    msg.drive.speed = 2.0
    msg.drive.steering_angle = 0.2
    msg.drive.acceleration = 0.0
    msg.drive.jerk = 0.0
    msg.drive.steering_angle_velocity = 0.0
    return msg


@pytest.fixture
def sample_joy_msg():
    """Fixture com mensagem Joy de exemplo."""
    if not ROS_AVAILABLE:
        msg = MagicMock()
        msg.axes = [0.0, 0.5, 0.0, -0.3, 0.0, 0.0]  # Forward + left turn
        msg.buttons = [0] * 11
        return msg

    msg = Joy()
    msg.axes = [0.0, 0.5, 0.0, -0.3, 0.0, 0.0]  # Forward + left turn
    msg.buttons = [0] * 11
    return msg


@pytest.fixture
def sample_twist_msg():
    """Fixture com mensagem Twist de exemplo."""
    if not ROS_AVAILABLE:
        msg = MagicMock()
        msg.linear.x = 1.5
        msg.angular.z = 0.3
        return msg

    msg = Twist()
    msg.linear.x = 1.5
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.3
    return msg


@pytest.fixture
def sample_odom_msg():
    """Fixture com mensagem Odometry de exemplo."""
    if not ROS_AVAILABLE:
        msg = MagicMock()
        msg.pose.pose.position.x = 1.0
        msg.pose.pose.position.y = 0.5
        msg.twist.twist.linear.x = 2.0
        return msg

    msg = Odometry()
    msg.pose.pose.position.x = 1.0
    msg.pose.pose.position.y = 0.5
    msg.pose.pose.position.z = 0.0
    msg.twist.twist.linear.x = 2.0
    msg.twist.twist.angular.z = 0.1
    return msg


class MockPublisher:
    """Mock para publishers ROS2."""

    def __init__(self):
        self.published_messages = []
        self.publish_count = 0

    def publish(self, msg):
        """Simula publicação de mensagem."""
        self.published_messages.append(msg)
        self.publish_count += 1

    def get_last_message(self):
        """Retorna última mensagem publicada."""
        return self.published_messages[-1] if self.published_messages else None

    def get_all_messages(self):
        """Retorna todas as mensagens publicadas."""
        return self.published_messages.copy()

    def clear_messages(self):
        """Limpa histórico de mensagens."""
        self.published_messages.clear()
        self.publish_count = 0


@pytest.fixture
def mock_publisher():
    """Fixture para mock publisher."""
    return MockPublisher()


class MockTimer:
    """Mock para timers ROS2."""

    def __init__(self, timer_period: float, callback):
        self.timer_period = timer_period
        self.callback = callback
        self.is_canceled = False
        self.call_count = 0

    def cancel(self):
        """Simula cancelamento do timer."""
        self.is_canceled = True

    def trigger(self):
        """Dispara o callback manualmente."""
        if not self.is_canceled:
            self.callback()
            self.call_count += 1


@pytest.fixture
def mock_timer():
    """Fixture para mock timer."""
    return MockTimer


class ServoTestData:
    """Dados de teste para validação do servo."""

    @staticmethod
    def angle_to_pulsewidth_cases():
        """Casos de teste para conversão ângulo->pulsewidth."""
        return [
            # (angle, expected_pulsewidth, config)
            (0.0, 1500, TestConfig()),      # Centro
            (0.4, 2000, TestConfig()),      # Máximo direita
            (-0.4, 1000, TestConfig()),     # Máximo esquerda
            (0.2, 1750, TestConfig()),      # Meio direita
            (-0.2, 1250, TestConfig()),     # Meio esquerda
            (0.8, 2000, TestConfig()),      # Saturação direita
            (-0.8, 1000, TestConfig()),     # Saturação esquerda
        ]

    @staticmethod
    def invalid_pulsewidth_cases():
        """Casos com larguras de pulso inválidas."""
        return [
            (400,),   # Muito baixo
            (3000,),  # Muito alto
            (-100,),  # Negativo
        ]


@pytest.fixture
def servo_test_data():
    """Fixture com dados de teste do servo."""
    return ServoTestData


class JoyTestData:
    """Dados de teste para conversores de joystick."""

    @staticmethod
    def joy_to_ackermann_cases():
        """Casos para conversão Joy->Ackermann."""
        return [
            # (joy_axes, expected_speed, expected_angle)
            ([0.0, 1.0, 0.0, 0.0], 7.0, 0.0),     # Máximo frente
            ([0.0, -1.0, 0.0, 0.0], -7.0, 0.0),   # Máximo trás
            ([0.0, 0.0, 0.0, 1.0], 0.0, 0.32),    # Máximo direita
            ([0.0, 0.0, 0.0, -1.0], 0.0, -0.32),  # Máximo esquerda
            ([0.0, 0.5, 0.0, 0.5], 3.5, 0.16),    # Combinado
            ([0.0, 0.05, 0.0, 0.05], 0.0, 0.0),   # Deadzone
        ]

    @staticmethod
    def joy_to_twist_cases():
        """Casos para conversão Joy->Twist."""
        return [
            # (joy_axes, expected_linear, expected_angular)
            ([0.0, 1.0, 0.0, 0.0], 3.0, 0.0),     # Máximo frente
            ([0.0, -1.0, 0.0, 0.0], -3.0, 0.0),   # Máximo trás
            ([0.0, 0.0, 0.0, 1.0], 0.0, 1.5),     # Máximo direita
            ([0.0, 0.0, 0.0, -1.0], 0.0, -1.5),   # Máximo esquerda
            ([0.0, 0.5, 0.0, 0.5], 1.5, 0.75),    # Combinado
        ]


@pytest.fixture
def joy_test_data():
    """Fixture com dados de teste do joystick."""
    return JoyTestData


class PerformanceTestHelper:
    """Helper para testes de performance."""

    def __init__(self):
        self.measurements = []

    def measure_execution_time(self, func, *args, **kwargs):
        """Mede tempo de execução de uma função."""
        start_time = time.perf_counter()
        result = func(*args, **kwargs)
        end_time = time.perf_counter()

        execution_time = end_time - start_time
        self.measurements.append(execution_time)

        return result, execution_time

    def get_average_time(self):
        """Retorna tempo médio de execução."""
        return sum(self.measurements) / len(self.measurements) if self.measurements else 0

    def get_max_time(self):
        """Retorna tempo máximo de execução."""
        return max(self.measurements) if self.measurements else 0

    def clear_measurements(self):
        """Limpa medições."""
        self.measurements.clear()


@pytest.fixture
def performance_helper():
    """Fixture para helper de performance."""
    return PerformanceTestHelper()


@pytest.fixture(autouse=True)
def setup_test_environment():
    """Fixture automática para configurar ambiente de teste."""
    # Configurar variáveis de ambiente para testes
    import os
    os.environ['F1TENTH_TEST_MODE'] = 'simulation'
    os.environ['ROS_DOMAIN_ID'] = '0'

    yield

    # Cleanup após o teste
    # Remover variáveis específicas se necessário
    pass


@pytest.fixture
def threaded_test_helper():
    """Helper para testes com threading."""

    class ThreadedTestHelper:
        def __init__(self):
            self.threads = []
            self.exceptions = []

        def run_in_thread(self, target, *args, **kwargs):
            """Executa função em thread separada."""
            def wrapper():
                try:
                    target(*args, **kwargs)
                except Exception as e:
                    self.exceptions.append(e)

            thread = threading.Thread(target=wrapper)
            self.threads.append(thread)
            thread.start()
            return thread

        def wait_all(self, timeout=5.0):
            """Aguarda todas as threads terminarem."""
            for thread in self.threads:
                thread.join(timeout=timeout)

        def has_exceptions(self):
            """Verifica se houve exceções nas threads."""
            return len(self.exceptions) > 0

        def get_exceptions(self):
            """Retorna lista de exceções."""
            return self.exceptions.copy()

    return ThreadedTestHelper()
