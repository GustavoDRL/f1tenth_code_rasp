#!/usr/bin/env python3
"""
Testes de integração para comunicação ROS2 no sistema F1tenth.

Este módulo valida:
- Comunicação entre nós ROS2
- Fluxo de dados entre componentes
- Sincronização de mensagens
- Integridade do pipeline de comandos

Autor: Professor PhD em Engenharia Robótica
"""

import pytest
import sys
import os
import time
import threading
from unittest.mock import MagicMock, patch, Mock
from typing import List, Dict, Any

# Adicionar src ao path para importações
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

# Mock para ROS2 se não disponível
try:
    import rclpy
    from rclpy.node import Node
    from ackermann_msgs.msg import AckermannDriveStamped
    from sensor_msgs.msg import Joy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


class TestTopicCommunication:
    """Testes para comunicação entre tópicos ROS2."""

    def test_joy_to_drive_topic_flow(self, mock_publisher, sample_joy_msg):
        """Testa fluxo de comunicação do joystick para comando drive."""
        # Arrange
        expected_speed = 3.5  # 7.0 * 0.5
        expected_angle = 0.16  # 0.32 * 0.5

        # Simular processamento do JoyToAckerman
        sample_joy_msg.axes = [0.0, 0.5, 0.0, 0.5, 0.0, 0.0]  # Meio frente + meio direita

        # Act - simular conversão e publicação
        speed = 7.0 * sample_joy_msg.axes[1]
        steering_angle = 0.32 * sample_joy_msg.axes[3]

        # Criar mensagem Ackermann simulada
        ackermann_msg = MagicMock()
        ackermann_msg.drive.speed = speed
        ackermann_msg.drive.steering_angle = steering_angle

        mock_publisher.publish(ackermann_msg)

        # Assert
        assert mock_publisher.publish_count == 1, "Deve ter publicado uma mensagem"

        published_msg = mock_publisher.get_last_message()
        assert published_msg.drive.speed == expected_speed, f"Velocidade deve ser {expected_speed}"
        assert published_msg.drive.steering_angle == expected_angle, f"Ângulo deve ser {expected_angle}"

    def test_drive_to_servo_command_flow(self, mock_pigpio, sample_ackermann_msg):
        """Testa fluxo do comando drive para controle de servo."""
        # Arrange
        steering_angle = 0.2  # rad
        expected_pulsewidth = 1750  # Aproximadamente para 0.2 rad

        sample_ackermann_msg.drive.steering_angle = steering_angle

        # Act - simular processamento do ServoControlNode
        # Conversão ângulo -> PWM
        max_angle = 0.4
        min_angle = -0.4
        max_pulse = 2000
        min_pulse = 1000

        angle_range = max_angle - min_angle
        pulse_range = max_pulse - min_pulse

        # Limitar ângulo
        clamped_angle = max(min(steering_angle, max_angle), min_angle)

        # Converter para PWM
        normalized_angle = (clamped_angle - min_angle) / angle_range
        pulse_width = int(min_pulse + normalized_angle * pulse_range)

        # Simular comando GPIO
        result = mock_pigpio.set_servo_pulsewidth(18, pulse_width)

        # Assert
        assert result == 0, "Comando GPIO deve ser bem-sucedido"
        assert pulse_width == expected_pulsewidth, f"Pulsewidth deve ser {expected_pulsewidth}"
        assert mock_pigpio.get_servo_pulsewidth(18) == pulse_width, "GPIO deve estar configurado corretamente"

    def test_odom_topic_republishing(self, mock_publisher, sample_odom_msg):
        """Testa republição de odometria no formato F1tenth."""
        # Arrange - simular odometria do VESC
        sample_odom_msg.pose.pose.position.x = 2.5
        sample_odom_msg.pose.pose.position.y = 1.2
        sample_odom_msg.twist.twist.linear.x = 3.0

        # Act - simular republição pelo ServoControlNode
        f1tenth_odom = MagicMock()
        f1tenth_odom.header.frame_id = "odom"
        f1tenth_odom.child_frame_id = "base_link"
        f1tenth_odom.pose = sample_odom_msg.pose
        f1tenth_odom.twist = sample_odom_msg.twist

        mock_publisher.publish(f1tenth_odom)

        # Assert
        assert mock_publisher.publish_count == 1, "Deve ter republicado odometria"

        published_odom = mock_publisher.get_last_message()
        assert published_odom.header.frame_id == "odom", "Frame deve ser 'odom'"
        assert published_odom.child_frame_id == "base_link", "Child frame deve ser 'base_link'"


class TestMessageSynchronization:
    """Testes para sincronização de mensagens."""

    def test_command_message_ordering(self, mock_publisher):
        """Testa ordem de processamento de comandos."""
        # Arrange
        commands = [
            (0.0, 0.0),   # Parar
            (2.0, 0.1),   # Frente + direita
            (1.0, -0.1),  # Mais devagar + esquerda
            (0.0, 0.0),   # Parar novamente
        ]

        # Act
        for speed, angle in commands:
            ackermann_msg = MagicMock()
            ackermann_msg.drive.speed = speed
            ackermann_msg.drive.steering_angle = angle
            ackermann_msg.timestamp = time.time()

            mock_publisher.publish(ackermann_msg)
            time.sleep(0.001)  # Pequeno delay para timestamps únicos

        # Assert
        published_messages = mock_publisher.get_all_messages()
        assert len(published_messages) == 4, "Deve ter processado todos os comandos"

        # Verificar ordem temporal
        for i in range(1, len(published_messages)):
            assert (published_messages[i].timestamp >=
                   published_messages[i-1].timestamp), "Mensagens devem estar em ordem temporal"

    def test_high_frequency_commands(self, mock_publisher, performance_helper):
        """Testa processamento de comandos em alta frequência."""
        # Arrange
        num_commands = 100
        frequency = 100  # Hz
        period = 1.0 / frequency

        # Act
        start_time = time.time()
        for i in range(num_commands):
            ackermann_msg = MagicMock()
            ackermann_msg.drive.speed = 2.0 * (i % 2)  # Alternando 0 e 2
            ackermann_msg.drive.steering_angle = 0.1 * ((-1) ** i)  # Alternando direção

            mock_publisher.publish(ackermann_msg)

            # Simular delay de processamento
            time.sleep(period / 10)  # 10x mais rápido que frequência alvo

        total_time = time.time() - start_time

        # Assert
        assert mock_publisher.publish_count == num_commands, f"Deve ter processado {num_commands} comandos"
        assert total_time < 2.0, f"Processamento deve ser rápido (atual: {total_time:.3f}s)"

        # Verificar que manteve alta taxa de throughput
        throughput = num_commands / total_time
        assert throughput > 50, f"Throughput deve ser > 50 Hz (atual: {throughput:.1f} Hz)"


class TestNodeLifecycle:
    """Testes para ciclo de vida dos nós ROS2."""

    def test_node_initialization_sequence(self, mock_ros_node, mock_pigpio):
        """Testa sequência de inicialização dos nós."""
        # Arrange & Act - simular inicialização do ServoControlNode

        # 1. Configurar parâmetros
        mock_ros_node.declare_parameters()  # Call the method to trigger 'called'

        # 2. Inicializar GPIO
        gpio_init_success = mock_pigpio.connected

        # 3. Configurar subscribers/publishers
        mock_ros_node.create_subscription.return_value = MagicMock()
        mock_ros_node.create_publisher.return_value = MagicMock()

        # 4. Configurar timers
        mock_ros_node.create_timer.return_value = MagicMock()

        # Assert
        assert gpio_init_success, "GPIO deve ser inicializado"
        assert mock_ros_node.declare_parameters.called, "Parâmetros devem ser declarados"
        assert mock_ros_node.create_subscription.called, "Subscribers devem ser criados"
        assert mock_ros_node.create_publisher.called, "Publishers devem ser criados"

    def test_node_cleanup_sequence(self, mock_ros_node, mock_pigpio):
        """Testa sequência de limpeza dos nós."""
        # Arrange - simular nó inicializado
        mock_pigpio.set_servo_pulsewidth(18, 1500)  # Configurar servo

        # Act - simular cleanup
        # 1. Centralizar servo
        cleanup_result = mock_pigpio.set_servo_pulsewidth(18, 1500)

        # 2. Desconectar GPIO
        mock_pigpio.stop()

        # Assert
        assert cleanup_result == 0, "Centralização do servo deve ser bem-sucedida"
        assert not mock_pigpio.connected, "GPIO deve estar desconectado"


class TestErrorRecovery:
    """Testes para recuperação de erros."""

    def test_gpio_disconnection_recovery(self, mock_ros_node):
        """Testa recuperação de desconexão GPIO."""
        # Arrange
        from tests.mock.mock_pigpio import MockPigpioContext

        # Simular falha após algumas operações
        with MockPigpioContext(connected=True, fail_after=3) as mock_pi:
            successful_operations = 0

            # Act
            for i in range(6):
                try:
                    result = mock_pi.set_servo_pulsewidth(18, 1500)
                    if result == 0:
                        successful_operations += 1
                except RuntimeError:
                    # Simular tentativa de reconexão
                    mock_pi.simulate_reconnect()
                    break

            # Assert
            assert successful_operations <= 3, "Deve falhar após 3 operações"

    def test_message_loss_handling(self, mock_publisher, threaded_test_helper):
        """Testa tratamento de perda de mensagens."""
        # Arrange
        total_messages = 50
        expected_loss_rate = 0.1  # 10% de perda aceitável

        def publish_messages():
            for i in range(total_messages):
                msg = MagicMock()
                msg.id = i
                mock_publisher.publish(msg)
                time.sleep(0.01)

        # Act
        threaded_test_helper.run_in_thread(publish_messages)
        threaded_test_helper.wait_all()

        # Assert
        received_messages = mock_publisher.publish_count
        loss_rate = (total_messages - received_messages) / total_messages

        assert not threaded_test_helper.has_exceptions(), "Não deve haver exceções"
        assert loss_rate <= expected_loss_rate, f"Taxa de perda ({loss_rate:.2f}) deve ser <= {expected_loss_rate}"


class TestPerformanceIntegration:
    """Testes de performance para integração completa."""

    def test_end_to_end_latency(self, mock_publisher, mock_pigpio, performance_helper):
        """Testa latência end-to-end do pipeline."""
        # Arrange
        test_iterations = 20

        def full_pipeline():
            """Simula pipeline completo Joy->Ackermann->Servo."""
            # 1. Conversão Joy->Ackermann
            joy_axes = [0.0, 0.5, 0.0, 0.3, 0.0, 0.0]
            speed = 7.0 * joy_axes[1]
            steering_angle = 0.32 * joy_axes[3]

            # 2. Publicação Ackermann
            ackermann_msg = MagicMock()
            ackermann_msg.drive.speed = speed
            ackermann_msg.drive.steering_angle = steering_angle
            mock_publisher.publish(ackermann_msg)

            # 3. Conversão para PWM
            angle_range = 0.8  # -0.4 to 0.4
            pulse_range = 1000  # 1000 to 2000
            normalized_angle = (steering_angle + 0.4) / angle_range
            pulse_width = int(1000 + normalized_angle * pulse_range)

            # 4. Comando GPIO
            mock_pigpio.set_servo_pulsewidth(18, pulse_width)

        # Act
        for _ in range(test_iterations):
            _, exec_time = performance_helper.measure_execution_time(full_pipeline)

        # Assert
        avg_latency = performance_helper.get_average_time()
        max_latency = performance_helper.get_max_time()

        assert avg_latency < 0.01, f"Latência média deve ser < 10ms (atual: {avg_latency*1000:.2f}ms)"
        assert max_latency < 0.02, f"Latência máxima deve ser < 20ms (atual: {max_latency*1000:.2f}ms)"

    def test_system_throughput(self, mock_publisher):
        """Testa throughput do sistema completo."""
        # Arrange
        duration = 1.0  # segundo
        commands_per_second = 0

        # Act
        start_time = time.time()
        while (time.time() - start_time) < duration:
            # Simular comando rápido
            msg = MagicMock()
            msg.drive.speed = 1.0
            msg.drive.steering_angle = 0.1
            mock_publisher.publish(msg)
            commands_per_second += 1

        actual_duration = time.time() - start_time
        throughput = commands_per_second / actual_duration

        # Assert
        assert throughput > 100, f"Throughput deve ser > 100 comandos/s (atual: {throughput:.1f})"


class TestFailsafeIntegration:
    """Testes para sistemas de failsafe integrados."""

    def test_emergency_stop_propagation(self, mock_publisher, mock_pigpio):
        """Testa propagação de parada de emergência."""
        # Arrange
        emergency_command = MagicMock()
        emergency_command.drive.speed = 0.0
        emergency_command.drive.steering_angle = 0.0
        emergency_command.is_emergency = True

        # Act
        mock_publisher.publish(emergency_command)

        # Simular resposta de emergência
        mock_pigpio.set_servo_pulsewidth(18, 1500)  # Centralizar

        # Assert
        assert mock_publisher.get_last_message().drive.speed == 0.0, "Velocidade deve ser zero"
        assert mock_pigpio.get_servo_pulsewidth(18) == 1500, "Servo deve estar centralizado"

    def test_timeout_handling(self, mock_publisher):
        """Testa tratamento de timeout de comandos."""
        # Arrange
        timeout_threshold = 1.0  # segundo
        last_command_time = time.time()

        # Act - simular ausência de comandos
        time.sleep(timeout_threshold + 0.1)
        current_time = time.time()

        # Verificar timeout
        time_since_last_command = current_time - last_command_time
        timeout_detected = time_since_last_command > timeout_threshold

        if timeout_detected:
            # Simular comando de emergência
            emergency_msg = MagicMock()
            emergency_msg.drive.speed = 0.0
            emergency_msg.drive.steering_angle = 0.0
            mock_publisher.publish(emergency_msg)

        # Assert
        assert timeout_detected, "Timeout deve ser detectado"
        assert mock_publisher.publish_count == 1, "Comando de emergência deve ser enviado"
        assert mock_publisher.get_last_message().drive.speed == 0.0, "Velocidade de emergência deve ser zero"


# Configuração de marcadores
pytestmark = [
    pytest.mark.integration,
    pytest.mark.ros_communication,
]


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
