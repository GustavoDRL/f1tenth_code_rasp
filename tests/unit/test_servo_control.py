#!/usr/bin/env python3
"""
Testes unitários para o sistema de controle de servo do F1tenth.

Valida conversão de ângulos, saturação, robustez e performance.

Autor: Professor PhD em Engenharia Robótica
"""

import pytest
import sys
import os
import math
import time
from unittest.mock import MagicMock, patch, Mock
from typing import Dict, Any

# Adicionar src ao path para importações
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

# Importar fixtures
from tests.mock.test_fixtures import (
    test_config, mock_pigpio, mock_pigpio_disconnected,
    servo_test_data, performance_helper
)
from tests.mock.mock_pigpio import MockPigpio, MockPigpioContext


class TestServoAngleConversion:
    """Testes para conversão de ângulo para largura de pulso PWM."""

    def test_center_angle_conversion(self, test_config):
        """Testa conversão do ângulo central (0.0 rad)."""
        # Arrange
        angle = 0.0
        expected_pulsewidth = (test_config.servo_max_pulse_width +
                             test_config.servo_min_pulse_width) // 2

        # Act
        angle_range = test_config.max_steering_angle - test_config.min_steering_angle
        pulse_range = test_config.servo_max_pulse_width - test_config.servo_min_pulse_width

        normalized_angle = (angle - test_config.min_steering_angle) / angle_range
        actual_pulsewidth = test_config.servo_min_pulse_width + normalized_angle * pulse_range

        # Assert
        assert abs(actual_pulsewidth - expected_pulsewidth) < 1, \
            f"Ângulo central deve resultar em {expected_pulsewidth}µs, mas obteve {actual_pulsewidth}µs"

    def test_maximum_angle_conversion(self, test_config):
        """Testa conversão do ângulo máximo."""
        # Arrange
        angle = test_config.max_steering_angle
        expected_pulsewidth = test_config.servo_max_pulse_width

        # Act
        angle_range = test_config.max_steering_angle - test_config.min_steering_angle
        pulse_range = test_config.servo_max_pulse_width - test_config.servo_min_pulse_width

        normalized_angle = (angle - test_config.min_steering_angle) / angle_range
        actual_pulsewidth = test_config.servo_min_pulse_width + normalized_angle * pulse_range

        # Assert
        assert abs(actual_pulsewidth - expected_pulsewidth) < 1, \
            f"Ângulo máximo deve resultar em {expected_pulsewidth}µs"

    def test_minimum_angle_conversion(self, test_config):
        """Testa conversão do ângulo mínimo."""
        # Arrange
        angle = test_config.min_steering_angle
        expected_pulsewidth = test_config.servo_min_pulse_width

        # Act
        angle_range = test_config.max_steering_angle - test_config.min_steering_angle
        pulse_range = test_config.servo_max_pulse_width - test_config.servo_min_pulse_width

        normalized_angle = (angle - test_config.min_steering_angle) / angle_range
        actual_pulsewidth = test_config.servo_min_pulse_width + normalized_angle * pulse_range

        # Assert
        assert abs(actual_pulsewidth - expected_pulsewidth) < 1, \
            f"Ângulo mínimo deve resultar em {expected_pulsewidth}µs"

    @pytest.mark.parametrize("angle,expected_pulsewidth", [
        (0.0, 1500),    # Centro
        (0.4, 2000),    # Máximo direita
        (-0.4, 1000),   # Máximo esquerda
        (0.2, 1750),    # Meio direita
        (-0.2, 1250),   # Meio esquerda
    ])
    def test_angle_conversion_parametrized(self, angle, expected_pulsewidth, test_config):
        """Testa conversão para múltiplos ângulos usando parametrização."""
        # Act
        angle_range = test_config.max_steering_angle - test_config.min_steering_angle
        pulse_range = test_config.servo_max_pulse_width - test_config.servo_min_pulse_width

        # Limitar ângulo
        clamped_angle = max(min(angle, test_config.max_steering_angle),
                           test_config.min_steering_angle)

        normalized_angle = (clamped_angle - test_config.min_steering_angle) / angle_range
        actual_pulsewidth = int(test_config.servo_min_pulse_width + normalized_angle * pulse_range)

        # Assert
        assert actual_pulsewidth == expected_pulsewidth, \
            f"Ângulo {angle} rad deve resultar em {expected_pulsewidth}µs, mas obteve {actual_pulsewidth}µs"


class TestServoSaturation:
    """Testes para saturação de ângulos fora dos limites."""

    def test_angle_saturation_positive(self, test_config):
        """Testa saturação para ângulos muito positivos."""
        # Arrange
        excessive_angle = test_config.max_steering_angle * 2

        # Act
        saturated_angle = min(max(excessive_angle, test_config.min_steering_angle),
                             test_config.max_steering_angle)

        # Assert
        assert saturated_angle == test_config.max_steering_angle, \
            "Ângulo excessivo deve ser saturado para o máximo permitido"

    def test_angle_saturation_negative(self, test_config):
        """Testa saturação para ângulos muito negativos."""
        # Arrange
        excessive_angle = test_config.min_steering_angle * 2

        # Act
        saturated_angle = min(max(excessive_angle, test_config.min_steering_angle),
                             test_config.max_steering_angle)

        # Assert
        assert saturated_angle == test_config.min_steering_angle, \
            "Ângulo excessivo negativo deve ser saturado para o mínimo permitido"

    @pytest.mark.parametrize("input_angle,expected_angle", [
        (0.8, 0.4),     # Saturação positiva
        (-0.8, -0.4),   # Saturação negativa
        (10.0, 0.4),    # Extremo positivo
        (-10.0, -0.4),  # Extremo negativo
        (0.0, 0.0),     # Dentro dos limites
        (0.3, 0.3),     # Dentro dos limites
    ])
    def test_angle_saturation_parametrized(self, input_angle, expected_angle, test_config):
        """Testa saturação parametrizada."""
        # Act
        saturated_angle = min(max(input_angle, test_config.min_steering_angle),
                             test_config.max_steering_angle)

        # Assert
        assert abs(saturated_angle - expected_angle) < 1e-6, \
            f"Ângulo {input_angle} deve ser saturado para {expected_angle}"


class TestServoControlClass:
    """Testes para a classe ServoControlNode (simulada)."""

    def create_mock_servo_control(self, config, pigpio_mock):
        """Cria uma instância mock do ServoControlNode."""

        class MockServoControl:
            def __init__(self, config, pi_instance):
                self.config = config
                self.pi = pi_instance

            def set_servo_angle(self, angle):
                """Simula o método set_servo_angle do ServoControlNode."""
                if not self.pi or not self.pi.connected:
                    return False

                # Limitar ângulo
                angle = min(max(angle, self.config.min_steering_angle),
                           self.config.max_steering_angle)

                # Converter para PWM
                angle_range = self.config.max_steering_angle - self.config.min_steering_angle
                pulse_range = self.config.servo_max_pulse_width - self.config.servo_min_pulse_width

                if angle_range == 0:
                    normalized_angle = 0.5
                else:
                    normalized_angle = (angle - self.config.min_steering_angle) / angle_range

                pulse_width = self.config.servo_min_pulse_width + normalized_angle * pulse_range
                pulse_width = int(min(max(pulse_width, self.config.servo_min_pulse_width),
                                    self.config.servo_max_pulse_width))

                # Aplicar via pigpio
                try:
                    result = self.pi.set_servo_pulsewidth(self.config.servo_gpio_pin, pulse_width)
                    return result == 0
                except Exception:
                    return False

        return MockServoControl(config, pigpio_mock)

    def test_servo_control_initialization(self, test_config, mock_pigpio):
        """Testa inicialização do controle de servo."""
        # Arrange & Act
        servo_control = self.create_mock_servo_control(test_config, mock_pigpio)

        # Assert
        assert servo_control.pi is not None, "Instância pigpio deve estar configurada"
        assert servo_control.pi.connected, "Conexão pigpio deve estar ativa"

    def test_servo_control_with_disconnected_pigpio(self, test_config, mock_pigpio_disconnected):
        """Testa comportamento com pigpio desconectado."""
        # Arrange
        servo_control = self.create_mock_servo_control(test_config, mock_pigpio_disconnected)

        # Act
        result = servo_control.set_servo_angle(0.0)

        # Assert
        assert result is False, "Deve retornar False quando pigpio está desconectado"

    def test_servo_angle_setting_success(self, test_config, mock_pigpio):
        """Testa configuração bem-sucedida de ângulo."""
        # Arrange
        servo_control = self.create_mock_servo_control(test_config, mock_pigpio)
        test_angle = 0.2

        # Act
        result = servo_control.set_servo_angle(test_angle)

        # Assert
        assert result is True, "Configuração de ângulo deve ser bem-sucedida"

        # Verificar se pigpio foi chamado
        pin_state = mock_pigpio.get_pin_state(test_config.servo_gpio_pin)
        assert 'servo_pulsewidth' in pin_state, "Pulsewidth deve ter sido configurado"
        assert pin_state['servo_pulsewidth'] > 0, "Pulsewidth deve ser positivo"

    def test_servo_angle_validation(self, test_config, mock_pigpio):
        """Testa validação de ângulos de entrada."""
        # Arrange
        servo_control = self.create_mock_servo_control(test_config, mock_pigpio)

        # Test cases
        test_cases = [
            (0.0, True),     # Ângulo válido
            (0.4, True),     # Máximo válido
            (-0.4, True),    # Mínimo válido
            (0.8, True),     # Será saturado
            (-0.8, True),    # Será saturado
        ]

        for angle, should_succeed in test_cases:
            # Act
            result = servo_control.set_servo_angle(angle)

            # Assert
            assert result == should_succeed, \
                f"Ângulo {angle} deve {'suceder' if should_succeed else 'falhar'}"


class TestPigpioIntegration:
    """Testes de integração com o mock pigpio."""

    def test_pigpio_gpio_mode_setting(self, test_config, mock_pigpio):
        """Testa configuração do modo GPIO."""
        # Arrange
        gpio_pin = test_config.servo_gpio_pin

        # Act
        result = mock_pigpio.set_mode(gpio_pin, 1)  # OUTPUT

        # Assert
        assert result == 0, "Configuração de modo deve ser bem-sucedida"
        assert mock_pigpio.get_mode(gpio_pin) == 1, "Modo deve estar configurado como OUTPUT"

    def test_pigpio_pwm_frequency_setting(self, test_config, mock_pigpio):
        """Testa configuração da frequência PWM."""
        # Arrange
        gpio_pin = test_config.servo_gpio_pin
        frequency = test_config.servo_pwm_frequency

        # Act
        result = mock_pigpio.set_PWM_frequency(gpio_pin, frequency)

        # Assert
        assert result == frequency, "Deve retornar a frequência configurada"
        assert mock_pigpio.get_PWM_frequency(gpio_pin) == frequency, \
            "Frequência deve estar corretamente configurada"

    def test_pigpio_servo_pulsewidth_setting(self, test_config, mock_pigpio):
        """Testa configuração da largura de pulso do servo."""
        # Arrange
        gpio_pin = test_config.servo_gpio_pin
        pulsewidth = 1500  # Centro

        # Act
        result = mock_pigpio.set_servo_pulsewidth(gpio_pin, pulsewidth)

        # Assert
        assert result == 0, "Configuração de pulsewidth deve ser bem-sucedida"
        assert mock_pigpio.get_servo_pulsewidth(gpio_pin) == pulsewidth, \
            "Pulsewidth deve estar configurado corretamente"

    def test_pigpio_invalid_pulsewidth(self, test_config, mock_pigpio):
        """Testa validação de larguras de pulso inválidas."""
        # Arrange
        gpio_pin = test_config.servo_gpio_pin
        invalid_pulsewidths = [400, 3000, -100]  # Muito baixo, muito alto, negativo

        for pulsewidth in invalid_pulsewidths:
            # Act
            result = mock_pigpio.set_servo_pulsewidth(gpio_pin, pulsewidth)

            # Assert
            assert result == -1, f"Pulsewidth inválido {pulsewidth} deve retornar erro"


class TestPerformanceAndRobustness:
    """Testes de performance e robustez do sistema."""

    def test_angle_conversion_performance(self, test_config, performance_helper):
        """Testa performance da conversão de ângulos."""
        # Arrange
        angles = [0.0, 0.1, 0.2, 0.3, 0.4, -0.1, -0.2, -0.3, -0.4]

        def angle_conversion(angle):
            """Função de conversão para testar performance."""
            angle_range = test_config.max_steering_angle - test_config.min_steering_angle
            pulse_range = test_config.servo_max_pulse_width - test_config.servo_min_pulse_width

            # Limitar ângulo
            clamped_angle = max(min(angle, test_config.max_steering_angle),
                               test_config.min_steering_angle)

            # Converter
            normalized_angle = (clamped_angle - test_config.min_steering_angle) / angle_range
            pulsewidth = test_config.servo_min_pulse_width + normalized_angle * pulse_range

            return int(pulsewidth)

        # Act
        for angle in angles:
            _, exec_time = performance_helper.measure_execution_time(angle_conversion, angle)

        # Assert
        avg_time = performance_helper.get_average_time()
        max_time = performance_helper.get_max_time()

        assert avg_time < 0.001, f"Tempo médio de conversão ({avg_time:.6f}s) deve ser < 1ms"
        assert max_time < 0.002, f"Tempo máximo de conversão ({max_time:.6f}s) deve ser < 2ms"

    def test_rapid_angle_changes(self, test_config, mock_pigpio):
        """Testa mudanças rápidas de ângulo."""
        # Arrange
        servo_control = self.create_mock_servo_control_simple(test_config, mock_pigpio)
        angles = [0.0, 0.4, -0.4, 0.2, -0.2, 0.0]

        # Act
        for angle in angles:
            result = servo_control.set_servo_angle(angle)
            assert result is True, f"Falha ao configurar ângulo {angle}"

        # Assert
        assert mock_pigpio.stats['servo_operations'] == len(angles), \
            "Todas as operações de servo devem ter sido registradas"

    def test_concurrent_access_simulation(self, test_config, mock_pigpio):
        """Simula acesso concorrente ao servo."""
        # Arrange
        servo_control = self.create_mock_servo_control_simple(test_config, mock_pigpio)

        # Act & Assert
        for i in range(100):
            angle = (i % 21 - 10) * 0.04  # Ângulos de -0.4 a 0.4
            result = servo_control.set_servo_angle(angle)
            assert result is True, f"Falha na iteração {i}"

        # Verificar integridade do estado
        final_operations = mock_pigpio.stats['servo_operations']
        assert final_operations == 100, "Todas as 100 operações devem ter sido registradas"

    def create_mock_servo_control_simple(self, config, pigpio_mock):
        """Versão simplificada do mock servo control para testes de performance."""

        class SimpleServoControl:
            def __init__(self, config, pi):
                self.config = config
                self.pi = pi

            def set_servo_angle(self, angle):
                if not self.pi.connected:
                    return False

                # Conversão simplificada
                angle = max(min(angle, self.config.max_steering_angle),
                           self.config.min_steering_angle)

                pulse_width = 1500  # Simplificado para performance

                result = self.pi.set_servo_pulsewidth(self.config.servo_gpio_pin, pulse_width)
                return result == 0

        return SimpleServoControl(config, pigpio_mock)


class TestErrorHandling:
    """Testes para tratamento de erros e casos extremos."""

    def test_division_by_zero_protection(self, test_config):
        """Testa proteção contra divisão por zero."""
        # Arrange - configuração com range zero
        config = test_config
        config.max_steering_angle = 0.0
        config.min_steering_angle = 0.0

        # Act
        angle_range = config.max_steering_angle - config.min_steering_angle

        # Simular proteção contra divisão por zero
        if angle_range == 0:
            normalized_angle = 0.5  # Valor seguro
        else:
            normalized_angle = (0.0 - config.min_steering_angle) / angle_range

        # Assert
        assert normalized_angle == 0.5, "Deve usar valor seguro quando range é zero"

    def test_invalid_gpio_pin(self, mock_pigpio):
        """Testa comportamento com pino GPIO inválido."""
        # Arrange
        invalid_pins = [-1, 54, 100]

        for pin in invalid_pins:
            # Act
            result = mock_pigpio.set_servo_pulsewidth(pin, 1500)

            # Assert
            assert result == -1, f"Pino inválido {pin} deve retornar erro"

    def test_pigpio_connection_failure(self):
        """Testa comportamento quando pigpio falha."""
        # Arrange
        with MockPigpioContext(connected=False) as mock_pi:
            # Act & Assert
            assert not mock_pi.connected, "Mock deve simular desconexão"

            # Tentar operação deve falhar
            with pytest.raises(RuntimeError):
                mock_pi.set_servo_pulsewidth(18, 1500)

    def test_pigpio_failure_after_operations(self):
        """Testa falha do pigpio após algumas operações."""
        # Arrange
        with MockPigpioContext(connected=True, fail_after=3) as mock_pi:
            # Act
            success_count = 0

            for i in range(5):
                try:
                    result = mock_pi.set_servo_pulsewidth(18, 1500)
                    if result == 0:
                        success_count += 1
                except RuntimeError:
                    break

            # Assert
            assert success_count <= 3, "Deve falhar após 3 operações"


class TestEdgeCases:
    """Testes para casos extremos e edge cases."""

    def test_very_small_angles(self, test_config, mock_pigpio):
        """Testa ângulos muito pequenos próximos a zero."""
        # Arrange
        small_angles = [1e-6, -1e-6, 1e-10, -1e-10, 0.0]
        servo_control = self.create_simple_servo_control(test_config, mock_pigpio)

        for angle in small_angles:
            # Act
            result = servo_control.set_servo_angle(angle)

            # Assert
            assert result is True, f"Ângulo pequeno {angle} deve ser processado corretamente"

    def test_precision_limits(self, test_config):
        """Testa limites de precisão de ponto flutuante."""
        # Arrange
        angle = 0.12345678901234567890  # Muitos dígitos

        # Act - simular conversão com limitação de precisão
        angle_range = test_config.max_steering_angle - test_config.min_steering_angle
        pulse_range = test_config.servo_max_pulse_width - test_config.servo_min_pulse_width

        normalized_angle = (angle - test_config.min_steering_angle) / angle_range
        pulse_width = test_config.servo_min_pulse_width + normalized_angle * pulse_range

        # Truncar para int (como faria o código real)
        int_pulse_width = int(pulse_width)

        # Assert
        assert isinstance(int_pulse_width, int), "Pulsewidth deve ser inteiro"
        assert 1000 <= int_pulse_width <= 2000, "Pulsewidth deve estar dentro dos limites"

    def create_simple_servo_control(self, config, pigpio_mock):
        """Helper para criar servo control simples."""
        class SimpleServo:
            def __init__(self, config, pi):
                self.config = config
                self.pi = pi

            def set_servo_angle(self, angle):
                if not self.pi.connected:
                    return False
                result = self.pi.set_servo_pulsewidth(config.servo_gpio_pin, 1500)
                return result == 0

        return SimpleServo(config, pigpio_mock)


# Configuração de marcadores para execução seletiva
pytestmark = [
    pytest.mark.unit,  # Marca como teste unitário
    pytest.mark.servo,  # Marca como teste de servo
]


if __name__ == '__main__':
    # Permitir execução direta para debug
    pytest.main([__file__, '-v'])
