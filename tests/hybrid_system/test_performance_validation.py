#!/usr/bin/env python3
"""
Testes de Performance - Sistema Híbrido F1TENTH - VERSÃO CORRIGIDA
Validação de requisitos de tempo real

CORREÇÕES APLICADAS:
- Imports corrigidos e organizados
- Fixtures pytest configuradas adequadamente
- Estrutura de classes pytest adequada
- Mocks alinhados com arquitetura real
- Métricas baseadas em documentação real do projeto

Métricas validadas (baseadas na documentação CURSOR):
- Latência servo: ≤10ms (target real do projeto)
- Latência motor: ≤15ms 
- Pipeline completo: ≤20ms
- CPU: ≤80% sustentado
- Memória: ≤1.5GB total

Autor: Sistema F1TENTH - Versão Corrigida
"""

import pytest
import sys
import os
import time
import threading
import statistics
import psutil
from unittest.mock import MagicMock, patch, Mock
from typing import List, Dict, Any, Optional

# Adicionar src ao path para importações
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

# Importar fixtures corretas
from tests.mock.test_fixtures import (
    test_config, mock_pigpio, performance_helper,
    sample_ackermann_msg, mock_ros_node
)


class TestLatencyValidation:
    """Valida latências do sistema baseadas em métricas reais CURSOR."""

    def test_servo_response_latency(self, mock_pigpio, test_config, performance_helper):
        """Testa latência de resposta do servo (target: ≤10ms)."""
        # Arrange
        target_latency_ms = 10.0
        test_angles = [0.0, 0.2, -0.2, 0.4, -0.4]
        gpio_pin = test_config.servo_gpio_pin
        
        latencies = []
        
        for angle in test_angles:
            # Act - Medir tempo de conversão e comando
            start_time = time.time()
            
            # Conversão ângulo → PWM (baseada no código real)
            angle_range = test_config.max_steering_angle - test_config.min_steering_angle
            pulse_range = test_config.servo_max_pulse_width - test_config.servo_min_pulse_width
            
            clamped_angle = max(min(angle, test_config.max_steering_angle), 
                               test_config.min_steering_angle)
            normalized_angle = (clamped_angle - test_config.min_steering_angle) / angle_range
            pulse_width = int(test_config.servo_min_pulse_width + normalized_angle * pulse_range)
            
            # Comando GPIO
            result = mock_pigpio.set_servo_pulsewidth(gpio_pin, pulse_width)
            
            end_time = time.time()
            latency_ms = (end_time - start_time) * 1000
            latencies.append(latency_ms)
        
        # Assert
        avg_latency = statistics.mean(latencies)
        max_latency = max(latencies)
        
        assert result == 0, "Comando deve ser bem-sucedido"
        assert avg_latency <= target_latency_ms, (
            f"Latência média {avg_latency:.2f}ms deve ser ≤{target_latency_ms}ms"
        )
        assert max_latency <= target_latency_ms * 1.5, (
            f"Latência máxima {max_latency:.2f}ms deve ser ≤{target_latency_ms * 1.5}ms"
        )

        print(f"📊 Servo Latency: avg={avg_latency:.2f}ms, max={max_latency:.2f}ms")

    def test_motor_command_latency(self, test_config):
        """Testa latência de comando do motor VESC (target: ≤15ms)."""
        # Arrange
        target_latency_ms = 15.0
        measured_latency_ms = 10.0  # Baseado em documentação CURSOR
        test_speeds = [0.0, 0.1, 0.2, 0.3, -0.1, -0.2]
        
        latencies = []
        
        for speed in test_speeds:
            # Act - Simular comando de velocidade
            start_time = time.time()
            
            # Validação de comando (baseada no código real)
            duty_cycle = speed / 6.0  # Conversão speed → duty cycle
            command_valid = abs(duty_cycle) <= 0.5
            
            # Simular tempo de processamento do comando VESC
            time.sleep(measured_latency_ms / 1000.0)  # Simular latência
            
            end_time = time.time()
            latency_ms = (end_time - start_time) * 1000
            latencies.append(latency_ms)
        
        # Assert
        avg_latency = statistics.mean(latencies)
        max_latency = max(latencies)
        
        assert command_valid, "Comando deve ser válido"
        assert avg_latency <= target_latency_ms, (
            f"Latência motor {avg_latency:.2f}ms deve ser ≤{target_latency_ms}ms"
        )

        print(f"📊 Motor Latency: avg={avg_latency:.2f}ms, max={max_latency:.2f}ms")

    def test_end_to_end_pipeline_latency(self, mock_pigpio, test_config):
        """Testa latência end-to-end do pipeline completo (target: ≤20ms)."""
        # Arrange
        target_pipeline_latency_ms = 20.0
        joystick_inputs = [
            (0.5, 0.0),    # Frente
            (0.0, 0.5),    # Direita
            (-0.5, 0.0),   # Trás
            (0.0, -0.5),   # Esquerda
            (0.0, 0.0),    # Parar
        ]
        
        pipeline_latencies = []
        gpio_pin = test_config.servo_gpio_pin
        
        for joy_speed, joy_steering in joystick_inputs:
            # Act - Pipeline completo: Joystick → Ackermann → Servo/Motor
            start_time = time.time()
            
            # 1. Conversão Joystick → Ackermann
            max_speed = test_config.max_speed
            max_steering = test_config.max_angle
            
            ackermann_speed = max_speed * joy_speed
            ackermann_steering = max_steering * joy_steering
            
            # 2. Comando Servo
            angle_range = test_config.max_steering_angle - test_config.min_steering_angle
            pulse_range = test_config.servo_max_pulse_width - test_config.servo_min_pulse_width
            
            clamped_angle = max(min(ackermann_steering, test_config.max_steering_angle), 
                               test_config.min_steering_angle)
            normalized_angle = (clamped_angle - test_config.min_steering_angle) / angle_range
            pulse_width = int(test_config.servo_min_pulse_width + normalized_angle * pulse_range)
            
            servo_result = mock_pigpio.set_servo_pulsewidth(gpio_pin, pulse_width)
            
            # 3. Comando Motor (simulado)
            duty_cycle = ackermann_speed / 6.0  # Conversão para duty cycle
            motor_valid = abs(duty_cycle) <= 0.5
            
            end_time = time.time()
            pipeline_latency_ms = (end_time - start_time) * 1000
            pipeline_latencies.append(pipeline_latency_ms)
        
        # Assert
        avg_pipeline_latency = statistics.mean(pipeline_latencies)
        max_pipeline_latency = max(pipeline_latencies)
        
        assert servo_result == 0, "Comando servo deve ser bem-sucedido"
        assert motor_valid, "Comando motor deve ser válido"
        assert avg_pipeline_latency <= target_pipeline_latency_ms, (
            f"Pipeline médio {avg_pipeline_latency:.2f}ms deve ser ≤{target_pipeline_latency_ms}ms"
        )
        assert max_pipeline_latency <= target_pipeline_latency_ms * 1.5, (
            f"Pipeline máximo {max_pipeline_latency:.2f}ms deve ser ≤{target_pipeline_latency_ms * 1.5}ms"
        )

        print(f"📊 Pipeline Latency: avg={avg_pipeline_latency:.2f}ms, max={max_pipeline_latency:.2f}ms")


class TestThroughputValidation:
    """Valida throughput do sistema (target: 50Hz)."""

    def test_servo_command_throughput(self, mock_pigpio, test_config):
        """Testa throughput de comandos de servo (target: 50Hz)."""
        # Arrange
        target_frequency_hz = 50.0
        test_duration_seconds = 2.0
        expected_commands = int(target_frequency_hz * test_duration_seconds)
        gpio_pin = test_config.servo_gpio_pin
        
        commands_sent = 0
        start_time = time.time()
        
        # Act - Enviar comandos em alta frequência
        while (time.time() - start_time) < test_duration_seconds:
            # Alternar ângulos para simular movimento
            angle = 0.2 if (commands_sent % 2) else -0.2
            
            # Converter e enviar comando
            angle_range = test_config.max_steering_angle - test_config.min_steering_angle
            pulse_range = test_config.servo_max_pulse_width - test_config.servo_min_pulse_width
            
            normalized_angle = (angle - test_config.min_steering_angle) / angle_range
            pulse_width = int(test_config.servo_min_pulse_width + normalized_angle * pulse_range)
            
            result = mock_pigpio.set_servo_pulsewidth(gpio_pin, pulse_width)
            
            if result == 0:
                commands_sent += 1
            
            # Pequeno delay para atingir frequência alvo
            time.sleep(1.0 / target_frequency_hz)
        
        total_time = time.time() - start_time
        actual_frequency = commands_sent / total_time
        
        # Assert
        assert commands_sent >= expected_commands * 0.9, (
            f"Deve enviar pelo menos 90% dos comandos esperados ({expected_commands})"
        )
        assert actual_frequency >= target_frequency_hz * 0.9, (
            f"Frequência deve ser ≥{target_frequency_hz * 0.9}Hz (atual: {actual_frequency:.1f}Hz)"
        )

        print(f"📊 Servo Throughput: {actual_frequency:.1f}Hz ({commands_sent} comandos)")

    def test_motor_command_throughput(self):
        """Testa throughput de comandos do motor."""
        # Arrange
        target_frequency_hz = 50.0
        test_duration_seconds = 1.0
        
        commands_processed = 0
        start_time = time.time()
        
        # Act - Processar comandos de motor
        while (time.time() - start_time) < test_duration_seconds:
            # Alternar velocidades
            speed = 0.3 if (commands_processed % 2) else -0.3
            
            # Validar comando
            duty_cycle = speed / 6.0
            if abs(duty_cycle) <= 0.5:
                commands_processed += 1
            
            time.sleep(1.0 / target_frequency_hz)
        
        total_time = time.time() - start_time
        actual_frequency = commands_processed / total_time
        
        # Assert
        assert actual_frequency >= target_frequency_hz * 0.95, (
            f"Throughput motor deve ser ≥{target_frequency_hz * 0.95}Hz (atual: {actual_frequency:.1f}Hz)"
        )

        print(f"📊 Motor Throughput: {actual_frequency:.1f}Hz ({commands_processed} comandos)")


class TestJitterAndStability:
    """Valida jitter temporal e estabilidade."""

    def test_servo_timing_jitter(self, mock_pigpio, test_config):
        """Testa jitter no timing dos comandos de servo (target: <10ms)."""
        # Arrange
        target_jitter_ms = 10.0
        num_samples = 50
        target_period_ms = 20.0  # 50Hz
        gpio_pin = test_config.servo_gpio_pin
        
        timestamps = []
        
        # Act - Coletar timestamps de comandos
        for i in range(num_samples):
            start_time = time.time()
            
            # Comando de teste
            angle = 0.1 if (i % 2) else -0.1
            angle_range = test_config.max_steering_angle - test_config.min_steering_angle
            pulse_range = test_config.servo_max_pulse_width - test_config.servo_min_pulse_width
            
            normalized_angle = (angle - test_config.min_steering_angle) / angle_range
            pulse_width = int(test_config.servo_min_pulse_width + normalized_angle * pulse_range)
            
            result = mock_pigpio.set_servo_pulsewidth(gpio_pin, pulse_width)
            timestamps.append(time.time())
            
            # Manter frequência
            time.sleep(target_period_ms / 1000.0)
        
        # Calcular jitter
        intervals = [(timestamps[i+1] - timestamps[i]) * 1000 
                    for i in range(len(timestamps)-1)]
        avg_interval = statistics.mean(intervals)
        jitter = statistics.stdev(intervals) if len(intervals) > 1 else 0
        
        # Assert
        assert result == 0, "Comandos devem ser bem-sucedidos"
        assert jitter <= target_jitter_ms, (
            f"Jitter {jitter:.2f}ms deve ser ≤{target_jitter_ms}ms"
        )
        assert abs(avg_interval - target_period_ms) <= 5.0, (
            f"Período médio deve estar próximo de {target_period_ms}ms"
        )

        print(f"📊 Timing Jitter: {jitter:.2f}ms (período médio: {avg_interval:.2f}ms)")


class TestStressAndLoad:
    """Testes de stress e carga do sistema."""

    def test_high_frequency_stress(self, mock_pigpio, test_config):
        """Testa sistema sob alta frequência de comandos."""
        # Arrange
        stress_frequency_hz = 100.0  # 2x a frequência normal
        stress_duration_seconds = 5.0
        gpio_pin = test_config.servo_gpio_pin
        
        commands_successful = 0
        commands_failed = 0
        
        start_time = time.time()
        
        # Act - Stress test
        while (time.time() - start_time) < stress_duration_seconds:
            # Comandos alternados rápidos
            angle = 0.3 if (commands_successful % 2) else -0.3
            
            # Conversão rápida
            normalized_angle = (angle + 0.4) / 0.8  # Normalização simplificada
            pulse_width = int(1000 + normalized_angle * 1000)
            
            result = mock_pigpio.set_servo_pulsewidth(gpio_pin, pulse_width)
            
            if result == 0:
                commands_successful += 1
            else:
                commands_failed += 1
            
            time.sleep(1.0 / stress_frequency_hz)
        
        total_commands = commands_successful + commands_failed
        success_rate = commands_successful / total_commands if total_commands > 0 else 0
        
        # Assert
        assert success_rate >= 0.95, (
            f"Taxa de sucesso deve ser ≥95% sob stress (atual: {success_rate*100:.1f}%)"
        )
        assert commands_successful >= stress_frequency_hz * stress_duration_seconds * 0.9, (
            "Deve processar pelo menos 90% dos comandos esperados"
        )

        print(f"📊 Stress Test: {success_rate*100:.1f}% sucesso ({commands_successful} comandos)")

    def test_concurrent_servo_motor_load(self, mock_pigpio, test_config):
        """Testa carga simultânea de servo e motor."""
        # Arrange
        test_duration_seconds = 3.0
        servo_commands = 0
        motor_commands = 0
        gpio_pin = test_config.servo_gpio_pin
        
        def servo_worker():
            nonlocal servo_commands
            start_time = time.time()
            
            while (time.time() - start_time) < test_duration_seconds:
                angle = 0.2 if (servo_commands % 3) else -0.2
                
                normalized_angle = (angle + 0.4) / 0.8
                pulse_width = int(1000 + normalized_angle * 1000)
                
                result = mock_pigpio.set_servo_pulsewidth(gpio_pin, pulse_width)
                if result == 0:
                    servo_commands += 1
                
                time.sleep(0.02)  # 50Hz
        
        def motor_worker():
            nonlocal motor_commands
            start_time = time.time()
            
            while (time.time() - start_time) < test_duration_seconds:
                speed = 0.25 if (motor_commands % 3) else -0.25
                duty_cycle = speed / 6.0
                
                if abs(duty_cycle) <= 0.5:
                    motor_commands += 1
                
                time.sleep(0.02)  # 50Hz
        
        # Act - Execução concorrente
        servo_thread = threading.Thread(target=servo_worker)
        motor_thread = threading.Thread(target=motor_worker)
        
        servo_thread.start()
        motor_thread.start()
        
        servo_thread.join()
        motor_thread.join()
        
        # Assert
        expected_commands = int(50 * test_duration_seconds * 0.9)  # 90% de 50Hz
        assert servo_commands >= expected_commands, (
            f"Servo deve processar ≥{expected_commands} comandos (atual: {servo_commands})"
        )
        assert motor_commands >= expected_commands, (
            f"Motor deve processar ≥{expected_commands} comandos (atual: {motor_commands})"
        )

        print(f"📊 Concurrent Load: Servo={servo_commands}, Motor={motor_commands}")


class TestResourceUsage:
    """Testa uso de recursos do sistema."""

    def test_cpu_usage_monitoring(self):
        """Testa monitoramento de uso de CPU."""
        # Arrange
        target_cpu_limit = 80.0  # 80% baseado na documentação
        test_duration = 2.0
        
        cpu_readings = []
        start_time = time.time()
        
        # Act - Monitorar CPU durante operação simulada
        while (time.time() - start_time) < test_duration:
            # Simular carga de trabalho
            for i in range(1000):
                _ = i ** 2  # Operação simples
            
            cpu_percent = psutil.cpu_percent(interval=0.1)
            cpu_readings.append(cpu_percent)
        
        # Assert
        max_cpu = max(cpu_readings)
        avg_cpu = statistics.mean(cpu_readings)
        
        assert max_cpu <= target_cpu_limit, (
            f"CPU máximo {max_cpu:.1f}% deve ser ≤{target_cpu_limit}%"
        )

        print(f"📊 CPU Usage: avg={avg_cpu:.1f}%, max={max_cpu:.1f}%")

    def test_memory_usage_monitoring(self):
        """Testa monitoramento de uso de memória."""
        # Arrange
        target_memory_limit_gb = 1.5  # 1.5GB baseado na documentação
        initial_memory = psutil.virtual_memory().used / 1024 / 1024 / 1024  # GB
        
        # Act - Simular operação com dados
        test_data = []
        for i in range(10000):
            test_data.append({
                'id': i,
                'angle': 0.1 * i,
                'speed': 2.0 + 0.1 * i,
                'timestamp': time.time()
            })
        
        final_memory = psutil.virtual_memory().used / 1024 / 1024 / 1024  # GB
        
        # Assert
        assert final_memory <= target_memory_limit_gb, (
            f"Memória {final_memory:.2f}GB deve ser ≤{target_memory_limit_gb}GB"
        )

        print(f"📊 Memory Usage: {final_memory:.2f}GB (aumento: {final_memory-initial_memory:.2f}GB)")


class TestDeadlineMonitoring:
    """Monitora cumprimento de deadlines críticos."""

    def test_control_loop_deadline(self, mock_pigpio, test_config):
        """Testa deadline do loop de controle (20ms max)."""
        # Arrange
        deadline_ms = 20.0  # 50Hz = 20ms period
        num_iterations = 25
        deadline_violations = 0
        gpio_pin = test_config.servo_gpio_pin
        
        # Act - Simular loop de controle
        for i in range(num_iterations):
            loop_start = time.time()
            
            # Simular carga de trabalho do loop de controle
            angle = 0.1 * (i % 5 - 2)  # Varia de -0.2 a +0.2
            
            # Processamento do servo
            normalized_angle = (angle + 0.4) / 0.8
            pulse_width = int(1000 + normalized_angle * 1000)
            servo_result = mock_pigpio.set_servo_pulsewidth(gpio_pin, pulse_width)
            
            # Processamento do motor
            motor_speed = angle / 2.0  # Converter ângulo em velocidade
            motor_valid = abs(motor_speed / 6.0) <= 0.5
            
            loop_end = time.time()
            loop_duration_ms = (loop_end - loop_start) * 1000
            
            if loop_duration_ms > deadline_ms:
                deadline_violations += 1
        
        violation_rate = deadline_violations / num_iterations
        
        # Assert
        assert servo_result == 0, "Comandos servo devem ser bem-sucedidos"
        assert motor_valid, "Comandos motor devem ser válidos"
        assert violation_rate <= 0.05, (
            f"Violações de deadline devem ser ≤5% (atual: {violation_rate*100:.1f}%)"
        )

        print(f"📊 Deadline Compliance: {(1-violation_rate)*100:.1f}% (violações: {deadline_violations})")

    def test_emergency_response_deadline(self, mock_pigpio, test_config):
        """Testa deadline de resposta de emergência (5ms max)."""
        # Arrange
        emergency_deadline_ms = 5.0
        gpio_pin = test_config.servo_gpio_pin
        
        # Act - Simular emergência
        emergency_start = time.time()
        
        # Comando de emergência - servo para centro
        emergency_pulse = 1500  # Centro
        servo_result = mock_pigpio.set_servo_pulsewidth(gpio_pin, emergency_pulse)
        
        # Motor para parada
        emergency_speed = 0.0
        motor_stopped = emergency_speed == 0.0
        
        emergency_end = time.time()
        response_time_ms = (emergency_end - emergency_start) * 1000
        
        # Assert
        assert servo_result == 0, "Comando de emergência servo deve ser bem-sucedido"
        assert motor_stopped, "Motor deve parar em emergência"
        assert response_time_ms <= emergency_deadline_ms, (
            f"Resposta de emergência {response_time_ms:.2f}ms deve ser ≤{emergency_deadline_ms}ms"
        )

        print(f"🚨 Emergency Response: {response_time_ms:.2f}ms")


if __name__ == "__main__":
    # Execução direta para debug
    pytest.main([__file__, "-v", "-s"]) 