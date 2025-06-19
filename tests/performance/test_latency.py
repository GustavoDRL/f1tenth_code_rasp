#!/usr/bin/env python3
"""
Testes de performance de latência para o sistema F1tenth.

Este módulo valida:
- Latência de comandos de controle
- Tempo de resposta do servo
- Performance em tempo real
- Jitter de temporização

Autor: Professor PhD em Engenharia Robótica
"""

import pytest
import sys
import os
import time
import statistics
import threading
from unittest.mock import MagicMock
from typing import List, Dict, Tuple

# Adicionar src ao path para importações
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))


class TestCommandLatency:
    """Testes de latência para comandos de controle."""

    def test_joy_to_ackermann_latency(self, performance_helper, sample_joy_msg):
        """Testa latência da conversão Joy para Ackermann."""
        # Arrange
        max_latency = 0.001  # 1ms máximo
        num_iterations = 1000

        # Configurar dados de entrada
        sample_joy_msg.axes = [0.0, 0.5, 0.0, 0.3, 0.0, 0.0]

        def convert_joy_to_ackermann():
            """Simula conversão completa Joy->Ackermann."""
            # Aplicar deadzone
            controller_error = 0.1
            linear_input = sample_joy_msg.axes[1]
            angular_input = sample_joy_msg.axes[3]

            if abs(linear_input) < controller_error:
                linear_input = 0.0
            if abs(angular_input) < controller_error:
                angular_input = 0.0

            # Conversão
            speed = 7.0 * linear_input
            steering_angle = 0.32 * angular_input

            return speed, steering_angle

        # Act
        latencies = []
        for _ in range(num_iterations):
            _, exec_time = performance_helper.measure_execution_time(convert_joy_to_ackermann)
            latencies.append(exec_time)

        # Assert
        avg_latency = statistics.mean(latencies)
        max_measured_latency = max(latencies)
        p95_latency = statistics.quantiles(latencies, n=20)[18]  # 95th percentile

        assert avg_latency < max_latency, f"Latência média ({avg_latency*1000:.3f}ms) deve ser < {max_latency*1000}ms"
        assert max_measured_latency < max_latency * 5, f"Latência máxima ({max_measured_latency*1000:.3f}ms) muito alta"
        assert p95_latency < max_latency * 2, f"P95 latência ({p95_latency*1000:.3f}ms) deve ser < {max_latency*2*1000}ms"

    def test_servo_command_latency(self, performance_helper, mock_pigpio):
        """Testa latência dos comandos de servo."""
        # Arrange
        max_latency = 0.002  # 2ms máximo para GPIO
        num_iterations = 500
        gpio_pin = 18

        def send_servo_command():
            """Simula comando completo de servo."""
            # Conversão ângulo -> PWM
            angle = 0.2
            max_angle = 0.4
            min_angle = -0.4
            max_pulse = 2000
            min_pulse = 1000

            # Limitar ângulo
            clamped_angle = max(min(angle, max_angle), min_angle)

            # Converter para PWM
            angle_range = max_angle - min_angle
            pulse_range = max_pulse - min_pulse
            normalized_angle = (clamped_angle - min_angle) / angle_range
            pulse_width = int(min_pulse + normalized_angle * pulse_range)

            # Comando GPIO
            return mock_pigpio.set_servo_pulsewidth(gpio_pin, pulse_width)

        # Act
        latencies = []
        for _ in range(num_iterations):
            _, exec_time = performance_helper.measure_execution_time(send_servo_command)
            latencies.append(exec_time)

        # Assert
        avg_latency = statistics.mean(latencies)
        max_measured_latency = max(latencies)

        assert avg_latency < max_latency, f"Latência servo média ({avg_latency*1000:.3f}ms) deve ser < {max_latency*1000}ms"
        assert max_measured_latency < max_latency * 3, f"Latência servo máxima ({max_measured_latency*1000:.3f}ms) muito alta"

    def test_end_to_end_pipeline_latency(self, performance_helper, mock_pigpio, mock_publisher):
        """Testa latência end-to-end do pipeline completo."""
        # Arrange
        max_latency = 0.005  # 5ms máximo para pipeline completo
        num_iterations = 200

        def full_control_pipeline():
            """Simula pipeline completo de controle."""
            # 1. Entrada do joystick
            joy_axes = [0.0, 0.6, 0.0, 0.4, 0.0, 0.0]

            # 2. Conversão Joy->Ackermann
            speed = 7.0 * joy_axes[1]
            steering_angle = 0.32 * joy_axes[3]

            # 3. Criação da mensagem
            ackermann_msg = MagicMock()
            ackermann_msg.drive.speed = speed
            ackermann_msg.drive.steering_angle = steering_angle

            # 4. Publicação (simulada)
            mock_publisher.publish(ackermann_msg)

            # 5. Conversão para PWM
            max_angle = 0.4
            min_angle = -0.4
            max_pulse = 2000
            min_pulse = 1000

            clamped_angle = max(min(steering_angle, max_angle), min_angle)
            angle_range = max_angle - min_angle
            pulse_range = max_pulse - min_pulse
            normalized_angle = (clamped_angle - min_angle) / angle_range
            pulse_width = int(min_pulse + normalized_angle * pulse_range)

            # 6. Comando GPIO
            mock_pigpio.set_servo_pulsewidth(18, pulse_width)

            return pulse_width

        # Act
        latencies = []
        for _ in range(num_iterations):
            _, exec_time = performance_helper.measure_execution_time(full_control_pipeline)
            latencies.append(exec_time)

        # Assert
        avg_latency = statistics.mean(latencies)
        max_measured_latency = max(latencies)
        p99_latency = statistics.quantiles(latencies, n=100)[98]  # 99th percentile

        assert avg_latency < max_latency, f"Latência pipeline média ({avg_latency*1000:.3f}ms) deve ser < {max_latency*1000}ms"
        assert p99_latency < max_latency * 2, f"P99 latência pipeline ({p99_latency*1000:.3f}ms) muito alta"


class TestTimingJitter:
    """Testes para jitter de temporização."""

    def test_servo_update_jitter(self, performance_helper, mock_pigpio):
        """Testa jitter nas atualizações do servo."""
        # Arrange
        target_frequency = 100  # Hz
        target_period = 1.0 / target_frequency
        max_jitter = target_period * 0.1  # 10% de jitter máximo
        num_updates = 50

        # Act
        timestamps = []
        for i in range(num_updates):
            start_time = time.perf_counter()

            # Simular comando de servo
            pulse_width = 1500 + int(100 * (i % 11 - 5))  # Varia de 1000 a 2000
            mock_pigpio.set_servo_pulsewidth(18, pulse_width)

            timestamps.append(start_time)

            # Aguardar próximo período
            time.sleep(target_period)

        # Calcular jitter
        intervals = [timestamps[i+1] - timestamps[i] for i in range(len(timestamps)-1)]
        actual_period = statistics.mean(intervals)
        jitter = statistics.stdev(intervals)

        # Assert
        assert abs(actual_period - target_period) < target_period * 0.05, \
            f"Período médio ({actual_period*1000:.3f}ms) deve estar próximo do alvo ({target_period*1000:.3f}ms)"
        assert jitter < max_jitter, \
            f"Jitter ({jitter*1000:.3f}ms) deve ser < {max_jitter*1000:.3f}ms"

    def test_command_processing_consistency(self, performance_helper):
        """Testa consistência no processamento de comandos."""
        # Arrange
        num_commands = 100
        max_variance = 0.0005  # 0.5ms de variância máxima

        def process_command():
            """Simula processamento de comando padrão."""
            # Conversão típica
            angle = 0.25
            angle_range = 0.8
            pulse_range = 1000
            normalized_angle = (angle + 0.4) / angle_range
            pulse_width = int(1000 + normalized_angle * pulse_range)

            # Simular validação
            if pulse_width < 1000:
                pulse_width = 1000
            elif pulse_width > 2000:
                pulse_width = 2000

            return pulse_width

        # Act
        processing_times = []
        for _ in range(num_commands):
            _, exec_time = performance_helper.measure_execution_time(process_command)
            processing_times.append(exec_time)

        # Assert
        variance = statistics.variance(processing_times)
        cv = statistics.stdev(processing_times) / statistics.mean(processing_times)  # Coeficiente de variação

        assert variance < max_variance, f"Variância ({variance*1000:.6f}ms²) deve ser < {max_variance*1000:.6f}ms²"
        assert cv < 0.2, f"Coeficiente de variação ({cv:.3f}) deve ser < 0.2"


class TestRealtimeConstraints:
    """Testes para restrições de tempo real."""

    def test_control_loop_frequency(self, mock_pigpio):
        """Testa se o loop de controle mantém frequência adequada."""
        # Arrange
        target_frequency = 100  # Hz
        test_duration = 1.0  # segundo
        tolerance = 0.05  # 5% de tolerância

        # Simular loop de controle
        loop_count = 0
        start_time = time.perf_counter()

        # Act
        while (time.perf_counter() - start_time) < test_duration:
            # Simular processamento do loop
            pulse_width = 1500 + int(50 * (loop_count % 21 - 10))
            mock_pigpio.set_servo_pulsewidth(18, pulse_width)

            loop_count += 1

            # Aguardar próxima iteração
            time.sleep(1.0 / target_frequency)

        actual_duration = time.perf_counter() - start_time
        actual_frequency = loop_count / actual_duration

        # Assert
        frequency_error = abs(actual_frequency - target_frequency) / target_frequency
        assert frequency_error < tolerance, \
            f"Frequência ({actual_frequency:.1f}Hz) deve estar próxima de {target_frequency}Hz (erro: {frequency_error*100:.1f}%)"

    def test_deadline_compliance(self, performance_helper, mock_pigpio):
        """Testa cumprimento de deadlines de tempo real."""
        # Arrange
        deadline = 0.01  # 10ms deadline para comando
        num_tests = 100
        missed_deadlines = 0

        def time_critical_command():
            """Simula comando crítico de tempo."""
            # Processamento complexo simulado
            for _ in range(10):  # Algumas operações
                angle = 0.1 * (_ % 5 - 2)
                pulse_width = 1500 + int(angle * 1250)

            # Comando final
            mock_pigpio.set_servo_pulsewidth(18, pulse_width)
            return pulse_width

        # Act
        for _ in range(num_tests):
            _, exec_time = performance_helper.measure_execution_time(time_critical_command)

            if exec_time > deadline:
                missed_deadlines += 1

        # Assert
        deadline_miss_rate = missed_deadlines / num_tests
        max_miss_rate = 0.01  # 1% máximo de deadlines perdidos

        assert deadline_miss_rate <= max_miss_rate, \
            f"Taxa de deadlines perdidos ({deadline_miss_rate*100:.1f}%) deve ser <= {max_miss_rate*100:.1f}%"

    def test_worst_case_execution_time(self, performance_helper, mock_pigpio):
        """Testa tempo de execução no pior caso."""
        # Arrange
        wcet_limit = 0.005  # 5ms WCET limite
        num_stress_tests = 200

        def stress_test_command():
            """Simula comando sob stress."""
            # Múltiplas operações para simular pior caso
            angles = [0.4, -0.4, 0.2, -0.2, 0.0, 0.35, -0.35]

            for angle in angles:
                # Conversão completa
                clamped_angle = max(min(angle, 0.4), -0.4)
                normalized_angle = (clamped_angle + 0.4) / 0.8
                pulse_width = int(1000 + normalized_angle * 1000)

                # Validação adicional
                if pulse_width < 1000 or pulse_width > 2000:
                    pulse_width = 1500

            # Comando final
            mock_pigpio.set_servo_pulsewidth(18, pulse_width)
            return pulse_width

        # Act
        execution_times = []
        for _ in range(num_stress_tests):
            _, exec_time = performance_helper.measure_execution_time(stress_test_command)
            execution_times.append(exec_time)

        # Assert
        wcet = max(execution_times)
        p99_time = statistics.quantiles(execution_times, n=100)[98]

        assert wcet < wcet_limit, f"WCET ({wcet*1000:.3f}ms) deve ser < {wcet_limit*1000}ms"
        assert p99_time < wcet_limit * 0.8, f"P99 tempo ({p99_time*1000:.3f}ms) deve ser razoável"


class TestConcurrentPerformance:
    """Testes de performance com concorrência."""

    def test_thread_safety_performance(self, threaded_test_helper, mock_pigpio):
        """Testa performance com acesso concorrente."""
        # Arrange
        num_threads = 4
        commands_per_thread = 50
        max_total_time = 2.0  # segundos

        def concurrent_servo_commands():
            """Executa comandos de servo em thread."""
            for i in range(commands_per_thread):
                angle = 0.1 * (i % 21 - 10)  # -1.0 to 1.0
                clamped_angle = max(min(angle, 0.4), -0.4)
                normalized_angle = (clamped_angle + 0.4) / 0.8
                pulse_width = int(1000 + normalized_angle * 1000)

                mock_pigpio.set_servo_pulsewidth(18, pulse_width)
                time.sleep(0.001)  # Pequeno delay

        # Act
        start_time = time.time()

        for _ in range(num_threads):
            threaded_test_helper.run_in_thread(concurrent_servo_commands)

        threaded_test_helper.wait_all(timeout=max_total_time)

        total_time = time.time() - start_time

        # Assert
        assert not threaded_test_helper.has_exceptions(), "Não deve haver exceções em threads"
        assert total_time < max_total_time, f"Tempo total ({total_time:.2f}s) deve ser < {max_total_time}s"

        # Verificar que todos os comandos foram processados
        expected_operations = num_threads * commands_per_thread
        actual_operations = mock_pigpio.stats['servo_operations']

        # Aceitar alguma perda devido à concorrência
        min_operations = expected_operations * 0.9
        assert actual_operations >= min_operations, \
            f"Operações ({actual_operations}) deve ser >= {min_operations} (90% de {expected_operations})"

    def test_resource_contention_impact(self, performance_helper, mock_pigpio):
        """Testa impacto da contenção de recursos."""
        # Arrange
        baseline_iterations = 100
        contention_iterations = 100
        max_slowdown = 2.0  # Máximo 2x mais lento

        def baseline_command():
            """Comando baseline sem contenção."""
            mock_pigpio.set_servo_pulsewidth(18, 1500)

        def contended_command():
            """Comando com contenção simulada."""
            # Simular múltiplos acessos simultâneos
            for pin in [18, 19, 20]:  # Múltiplos pinos
                mock_pigpio.set_servo_pulsewidth(pin, 1500)

        # Act - Medir baseline
        baseline_times = []
        for _ in range(baseline_iterations):
            _, exec_time = performance_helper.measure_execution_time(baseline_command)
            baseline_times.append(exec_time)

        performance_helper.clear_measurements()

        # Act - Medir com contenção
        contention_times = []
        for _ in range(contention_iterations):
            _, exec_time = performance_helper.measure_execution_time(contended_command)
            contention_times.append(exec_time)

        # Assert
        baseline_avg = statistics.mean(baseline_times)
        contention_avg = statistics.mean(contention_times)
        slowdown = contention_avg / baseline_avg

        assert slowdown < max_slowdown, \
            f"Slowdown ({slowdown:.2f}x) deve ser < {max_slowdown}x devido à contenção"


# Configuração de marcadores
pytestmark = [
    pytest.mark.performance,
    pytest.mark.latency,
    pytest.mark.realtime,
]


if __name__ == '__main__':
    pytest.main([__file__, '-v', '--tb=short'])
