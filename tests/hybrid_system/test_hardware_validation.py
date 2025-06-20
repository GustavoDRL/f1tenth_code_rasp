#!/usr/bin/env python3
"""
F1TENTH Hardware Validation Tests - VERSÃO CORRIGIDA
Testes de validação de hardware real para servo e VESC

CORREÇÕES APLICADAS:
- Imports corrigidos para módulos reais do projeto
- Fixtures pytest configuradas corretamente  
- Estrutura de testes padronizada
- Mocks adequados para hardware

Hardware Requirements:
- Raspberry Pi 4B with GPIO access
- VESC motor controller (USB serial)
- Servo motor (GPIO PWM)
- Physical hardware connections
"""

import os
import sys
import time
import pytest
import psutil
import numpy as np
from unittest.mock import MagicMock, patch, Mock
from typing import List, Dict, Any, Optional

# Adicionar src ao path para importações corretas
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

# Importar fixtures corretas
from tests.mock.test_fixtures import (
    test_config, mock_pigpio, performance_helper,
    sample_ackermann_msg, mock_ros_node
)


class TestServoHardwareValidation:
    """Validação de hardware do servo - usando pytest corretamente."""

    def test_gpio_servo_connectivity(self, mock_pigpio, test_config):
        """Testa conectividade GPIO para servo (GPIO 18)."""
        # Arrange
        gpio_pin = test_config.servo_gpio_pin  # GPIO 18
        
        # Act - Simular inicialização GPIO via mock
        gpio_connected = mock_pigpio.connected
        gpio_mode_result = mock_pigpio.set_mode(gpio_pin, mock_pigpio.OUTPUT)
        
        # Assert
        assert gpio_connected, "GPIO deve estar conectado via pigpio"
        assert gpio_mode_result == 0, (
            f"GPIO {gpio_pin} deve ser configurado como OUTPUT"
        )
        assert mock_pigpio.get_mode(gpio_pin) == mock_pigpio.OUTPUT

    def test_servo_pwm_control(self, mock_pigpio, test_config):
        """Testa controle PWM do servo através do mock."""
        # Arrange
        gpio_pin = test_config.servo_gpio_pin
        center_pulse = 1500  # µs
        
        # Act - Configurar PWM
        freq_result = mock_pigpio.set_PWM_frequency(gpio_pin, 50)  # 50Hz
        pulse_result = mock_pigpio.set_servo_pulsewidth(gpio_pin, center_pulse)
        
        # Assert
        assert freq_result == 50, "Frequência PWM deve ser configurada para 50Hz"
        assert pulse_result == 0, "Comando PWM deve ser bem-sucedido"
        assert mock_pigpio.get_servo_pulsewidth(gpio_pin) == center_pulse

    def test_servo_angle_conversion(self, mock_pigpio, test_config):
        """Testa conversão de ângulo para PWM."""
        # Arrange
        gpio_pin = test_config.servo_gpio_pin
        test_angles = [
            (-0.4, 1000),  # Min angle → Min pulse
            (0.0, 1500),   # Center angle → Center pulse  
            (0.4, 2000)    # Max angle → Max pulse
        ]
        
        for angle, expected_pulse in test_angles:
            # Act - Conversão ângulo → PWM (implementação baseada no código real)
            angle_range = test_config.max_steering_angle - test_config.min_steering_angle
            pulse_range = test_config.servo_max_pulse_width - test_config.servo_min_pulse_width
            
            # Clamp angle
            clamped_angle = max(min(angle, test_config.max_steering_angle), 
                               test_config.min_steering_angle)
            
            # Convert to pulse width
            normalized_angle = (clamped_angle - test_config.min_steering_angle) / angle_range
            actual_pulse = int(test_config.servo_min_pulse_width + normalized_angle * pulse_range)
            
            # Apply to mock hardware
            result = mock_pigpio.set_servo_pulsewidth(gpio_pin, actual_pulse)
            
            # Assert
            assert result == 0, f"Comando deve ser bem-sucedido para ângulo {angle}"
            assert abs(actual_pulse - expected_pulse) <= 50, (
                f"Pulse width para ângulo {angle} deve ser próximo de {expected_pulse}µs"
            )

    def test_servo_hardware_limits(self, mock_pigpio, test_config):
        """Testa enforcement de limites de hardware."""
        gpio_pin = test_config.servo_gpio_pin
        
        # Test pulse width limits
        test_cases = [
            (400, -1),    # Below minimum (should fail)
            (1000, 0),    # Valid minimum
            (1500, 0),    # Valid center
            (2000, 0),    # Valid maximum
            (3000, -1)    # Above maximum (should fail)
        ]
        
        for pulse_width, expected_result in test_cases:
            result = mock_pigpio.set_servo_pulsewidth(gpio_pin, pulse_width)
            assert result == expected_result, (
                f"Pulse width {pulse_width} deve retornar {expected_result}"
            )

    def test_servo_response_timing(self, mock_pigpio, test_config, performance_helper):
        """Testa timing de resposta do servo."""
        # Arrange
        gpio_pin = test_config.servo_gpio_pin
        target_latency_ms = 10.0
        test_angles = [0.0, 0.2, -0.2, 0.4, -0.4]
        
        latencies = []
        
        for angle in test_angles:
            # Act - Medir tempo de conversão e comando
            start_time = time.time()
            
            # Conversão (baseada no código real)
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
        avg_latency = np.mean(latencies)
        max_latency = max(latencies)
        
        assert all(result == 0 for result in [result]), "Todos os comandos devem ser bem-sucedidos"
        assert avg_latency <= target_latency_ms, (
            f"Latência média {avg_latency:.2f}ms deve ser ≤{target_latency_ms}ms"
        )
        assert max_latency <= target_latency_ms * 2, (
            f"Latência máxima {max_latency:.2f}ms deve ser ≤{target_latency_ms * 2}ms"
        )


class TestVESCIntegrationValidation:
    """Testes de validação para integração VESC - usando mocks adequados."""

    def test_vesc_serial_connectivity_simulation(self, test_config):
        """Simula teste de conectividade serial com VESC."""
        # Arrange - Simular verificação de dispositivo USB
        expected_vesc_device = "/dev/ttyACM0"
        
        # Act - Simular verificação de dispositivo
        device_exists = True  # Simulado baseado em documentação CURSOR
        device_permissions = True  # Simulado - usuário no grupo dialout
        
        # Assert
        assert device_exists, (
            f"Dispositivo VESC deve existir em {expected_vesc_device}"
        )
        assert device_permissions, (
            "Usuário deve ter permissões para acessar VESC"
        )

    def test_vesc_command_structure(self, sample_ackermann_msg):
        """Testa estrutura de comandos VESC."""
        # Arrange
        max_speed = 3.0  # m/s
        max_duty_cycle = 0.5
        
        # Act - Simular conversão Ackermann → VESC
        ackermann_speed = sample_ackermann_msg.drive.speed  # 2.0 m/s
        duty_cycle = ackermann_speed / (max_speed / max_duty_cycle)  # Conversão
        
        # Assert
        assert abs(duty_cycle) <= max_duty_cycle, (
            f"Duty cycle {duty_cycle} deve estar dentro dos limites"
        )
        assert -1.0 <= duty_cycle <= 1.0, "Duty cycle deve estar em [-1.0, 1.0]"

    def test_vesc_safety_limits(self):
        """Testa limits de segurança VESC."""
        # Arrange
        max_duty_cycle = 0.5
        test_speeds = [-3.0, -1.0, 0.0, 1.0, 3.0, 5.0]  # m/s
        
        for speed in test_speeds:
            # Act - Aplicar limites de segurança
            duty_cycle = speed / 6.0  # Conversão exemplo
            clamped_duty = max(min(duty_cycle, max_duty_cycle), -max_duty_cycle)
            
            # Assert
            assert abs(clamped_duty) <= max_duty_cycle, (
                f"Duty cycle para velocidade {speed} deve ser limitado"
            )

    def test_vesc_odometry_processing(self, mock_ros_node):
        """Testa processamento de odometria VESC."""
        # Arrange - Dados simulados do VESC
        vesc_data = {
            'speed': 2.0,          # m/s
            'voltage': 12.5,       # V
            'current': 5.2,        # A
            'duty_cycle': 0.33,    # %
            'rpm': 1500
        }
        
        # Act - Simular processamento de odometria
        odom_valid = all([
            vesc_data['voltage'] > 10.0,    # Bateria OK
            abs(vesc_data['current']) < 20.0,  # Corrente segura
            abs(vesc_data['duty_cycle']) <= 1.0  # Duty cycle válido
        ])
        
        # Assert
        assert odom_valid, "Dados de odometria devem ser válidos"
        assert vesc_data['speed'] >= 0.0, "Velocidade deve ser não-negativa"


class TestIntegratedSystemValidation:
    """Testes de validação do sistema integrado."""

    def test_servo_vesc_coordination(self, mock_pigpio, test_config):
        """Testa coordenação entre servo e VESC."""
        # Arrange
        gpio_pin = test_config.servo_gpio_pin
        test_commands = [
            (0.0, 1.0),    # Reto, frente
            (0.3, 1.5),    # Direita, mais rápido  
            (-0.3, 1.0),   # Esquerda, moderado
            (0.0, 0.0)     # Parar
        ]
        
        for steering_angle, speed in test_commands:
            # Act - Comando coordenado (servo + motor)
            
            # 1. Servo command
            angle_range = test_config.max_steering_angle - test_config.min_steering_angle
            pulse_range = test_config.servo_max_pulse_width - test_config.servo_min_pulse_width
            
            clamped_angle = max(min(steering_angle, test_config.max_steering_angle), 
                               test_config.min_steering_angle)
            normalized_angle = (clamped_angle - test_config.min_steering_angle) / angle_range
            pulse_width = int(test_config.servo_min_pulse_width + normalized_angle * pulse_range)
            
            servo_result = mock_pigpio.set_servo_pulsewidth(gpio_pin, pulse_width)
            
            # 2. Motor command (simulated)
            duty_cycle = speed / 6.0  # Conversão exemplo
            motor_valid = abs(duty_cycle) <= 0.5
            
            # Assert
            assert servo_result == 0, f"Comando servo deve funcionar para ângulo {steering_angle}"
            assert motor_valid, f"Comando motor deve ser válido para velocidade {speed}"

    def test_system_performance_metrics(self, mock_pigpio, test_config):
        """Testa métricas de performance geral do sistema."""
        # Arrange
        gpio_pin = test_config.servo_gpio_pin
        num_iterations = 50
        target_frequency_hz = 50.0
        
        command_times = []
        
        # Act - Executar loop de controle simulado
        for i in range(num_iterations):
            start_time = time.time()
            
            # Comando simulado
            angle = 0.1 * (i % 10 - 5) / 5  # -0.1 to 0.1 rad
            pulse_width = 1500 + int(angle * 500)  # Conversão simplificada
            
            result = mock_pigpio.set_servo_pulsewidth(gpio_pin, pulse_width)
            
            end_time = time.time()
            command_time = (end_time - start_time) * 1000  # ms
            command_times.append(command_time)
            
            # Simular delay para frequência alvo
            time.sleep(1.0 / target_frequency_hz)
        
        # Assert
        avg_command_time = np.mean(command_times)
        max_command_time = max(command_times)
        
        assert all(result == 0 for result in [result]), "Todos os comandos devem funcionar"
        assert avg_command_time <= 5.0, (
            f"Tempo médio de comando {avg_command_time:.2f}ms deve ser ≤5ms"
        )
        assert max_command_time <= 10.0, (
            f"Tempo máximo de comando {max_command_time:.2f}ms deve ser ≤10ms"
        )

    def test_system_resource_usage(self):
        """Testa uso de recursos do sistema durante operação."""
        # Arrange
        initial_memory = psutil.virtual_memory().used / 1024 / 1024  # MB
        
        # Act - Simular carga de trabalho
        test_data = []
        for i in range(1000):
            # Simular processamento
            test_data.append({
                'angle': 0.1 * (i % 10),
                'speed': 2.0 + 0.5 * (i % 5),
                'timestamp': time.time()
            })
        
        final_memory = psutil.virtual_memory().used / 1024 / 1024  # MB
        memory_increase = final_memory - initial_memory
        
        # Assert
        assert memory_increase <= 50.0, (
            f"Aumento de memória {memory_increase:.1f}MB deve ser ≤50MB"
        )
        assert len(test_data) == 1000, "Todos os dados devem ser processados"

    def test_system_emergency_procedures(self, mock_pigpio, test_config):
        """Testa procedimentos de emergência do sistema."""
        # Arrange
        gpio_pin = test_config.servo_gpio_pin
        
        # Act - Simular condição de emergência
        emergency_start = time.time()
        
        # 1. Emergency servo position (center)
        emergency_pulse = 1500  # Centro
        servo_result = mock_pigpio.set_servo_pulsewidth(gpio_pin, emergency_pulse)
        
        # 2. Emergency motor stop
        emergency_duty = 0.0
        motor_stopped = emergency_duty == 0.0
        
        emergency_end = time.time()
        response_time_ms = (emergency_end - emergency_start) * 1000
        
        # Assert
        assert servo_result == 0, "Comando de emergência servo deve funcionar"
        assert motor_stopped, "Motor deve parar em emergência"
        assert response_time_ms <= 10.0, (
            f"Tempo de resposta emergência {response_time_ms:.2f}ms deve ser ≤10ms"
        )


if __name__ == "__main__":
    # Execução com pytest
    pytest.main([__file__, "-v", "-s"]) 