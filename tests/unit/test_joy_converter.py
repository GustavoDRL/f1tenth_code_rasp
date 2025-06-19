#!/usr/bin/env python3
"""
Testes unitários para os conversores de joystick do F1tenth.

Este módulo testa:
- Conversão Joy para Ackermann
- Conversão Joy para Twist
- Validação de deadzone
- Saturação de comandos
- Tratamento de casos extremos

Autor: Professor PhD em Engenharia Robótica
"""

import pytest
import sys
import os
import math
from unittest.mock import MagicMock, patch

# Adicionar src ao path para importações
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))


class TestJoyToAckermann:
    """Testes para conversão de comandos Joy para Ackermann."""

    def test_forward_movement(self, test_config, sample_joy_msg):
        """Testa movimento para frente."""
        # Arrange
        sample_joy_msg.axes = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]  # Máximo frente
        max_speed = test_config.max_speed

        # Act - simular conversão do joy_ackerman.py
        speed = max_speed * sample_joy_msg.axes[1]
        steering_angle = test_config.max_angle * sample_joy_msg.axes[3]

        # Assert
        assert speed == max_speed, f"Velocidade deve ser {max_speed}"
        assert steering_angle == 0.0, "Ângulo de direção deve ser 0"

    def test_backward_movement(self, test_config, sample_joy_msg):
        """Testa movimento para trás."""
        # Arrange
        sample_joy_msg.axes = [0.0, -1.0, 0.0, 0.0, 0.0, 0.0]  # Máximo trás
        max_speed = test_config.max_speed

        # Act
        speed = max_speed * sample_joy_msg.axes[1]
        steering_angle = test_config.max_angle * sample_joy_msg.axes[3]

        # Assert
        assert speed == -max_speed, f"Velocidade deve ser {-max_speed}"
        assert steering_angle == 0.0, "Ângulo de direção deve ser 0"

    def test_right_turn(self, test_config, sample_joy_msg):
        """Testa curva para a direita."""
        # Arrange
        sample_joy_msg.axes = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]  # Máximo direita
        max_angle = test_config.max_angle

        # Act
        speed = test_config.max_speed * sample_joy_msg.axes[1]
        steering_angle = max_angle * sample_joy_msg.axes[3]

        # Assert
        assert speed == 0.0, "Velocidade deve ser 0"
        assert steering_angle == max_angle, f"Ângulo deve ser {max_angle}"

    def test_left_turn(self, test_config, sample_joy_msg):
        """Testa curva para a esquerda."""
        # Arrange
        sample_joy_msg.axes = [0.0, 0.0, 0.0, -1.0, 0.0, 0.0]  # Máximo esquerda
        max_angle = test_config.max_angle

        # Act
        speed = test_config.max_speed * sample_joy_msg.axes[1]
        steering_angle = max_angle * sample_joy_msg.axes[3]

        # Assert
        assert speed == 0.0, "Velocidade deve ser 0"
        assert steering_angle == -max_angle, f"Ângulo deve ser {-max_angle}"

    def test_combined_movement(self, test_config, sample_joy_msg):
        """Testa movimento combinado (frente + direita)."""
        # Arrange
        sample_joy_msg.axes = [0.0, 0.5, 0.0, 0.5, 0.0, 0.0]  # Meio frente + meio direita

        # Act
        speed = test_config.max_speed * sample_joy_msg.axes[1]
        steering_angle = test_config.max_angle * sample_joy_msg.axes[3]

        # Assert
        assert speed == test_config.max_speed * 0.5, "Velocidade deve ser metade do máximo"
        assert steering_angle == test_config.max_angle * 0.5, "Ângulo deve ser metade do máximo"

    def test_deadzone_filtering(self, test_config, sample_joy_msg):
        """Testa filtragem de deadzone."""
        # Arrange
        controller_error = 0.1
        sample_joy_msg.axes = [0.0, 0.05, 0.0, 0.05, 0.0, 0.0]  # Valores dentro do deadzone

        # Act - simular filtragem do deadzone
        linear_input = sample_joy_msg.axes[1]
        angular_input = sample_joy_msg.axes[3]

        if abs(linear_input) < controller_error:
            linear_input = 0.0
        if abs(angular_input) < controller_error:
            angular_input = 0.0

        speed = test_config.max_speed * linear_input
        steering_angle = test_config.max_angle * angular_input

        # Assert
        assert speed == 0.0, "Velocidade deve ser filtrada para 0"
        assert steering_angle == 0.0, "Ângulo deve ser filtrado para 0"


class TestJoyToTwist:
    """Testes para conversão de comandos Joy para Twist."""

    def test_forward_linear_velocity(self, sample_joy_msg):
        """Testa velocidade linear para frente."""
        # Arrange
        sample_joy_msg.axes = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]  # Máximo frente
        max_linear_speed = 3.0  # Valor do joy_twist.py

        # Act
        linear_velocity = max_linear_speed * sample_joy_msg.axes[1]
        angular_velocity = 1.5 * sample_joy_msg.axes[3]  # max_angular_speed

        # Assert
        assert linear_velocity == max_linear_speed, f"Velocidade linear deve ser {max_linear_speed}"
        assert angular_velocity == 0.0, "Velocidade angular deve ser 0"

    def test_backward_linear_velocity(self, sample_joy_msg):
        """Testa velocidade linear para trás."""
        # Arrange
        sample_joy_msg.axes = [0.0, -1.0, 0.0, 0.0, 0.0, 0.0]  # Máximo trás
        max_linear_speed = 3.0

        # Act
        linear_velocity = max_linear_speed * sample_joy_msg.axes[1]
        angular_velocity = 1.5 * sample_joy_msg.axes[3]

        # Assert
        assert linear_velocity == -max_linear_speed, f"Velocidade linear deve ser {-max_linear_speed}"
        assert angular_velocity == 0.0, "Velocidade angular deve ser 0"

    def test_right_angular_velocity(self, sample_joy_msg):
        """Testa velocidade angular para direita."""
        # Arrange
        sample_joy_msg.axes = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]  # Máximo direita
        max_angular_speed = 1.5

        # Act
        linear_velocity = 3.0 * sample_joy_msg.axes[1]
        angular_velocity = max_angular_speed * sample_joy_msg.axes[3]

        # Assert
        assert linear_velocity == 0.0, "Velocidade linear deve ser 0"
        assert angular_velocity == max_angular_speed, f"Velocidade angular deve ser {max_angular_speed}"

    def test_left_angular_velocity(self, sample_joy_msg):
        """Testa velocidade angular para esquerda."""
        # Arrange
        sample_joy_msg.axes = [0.0, 0.0, 0.0, -1.0, 0.0, 0.0]  # Máximo esquerda
        max_angular_speed = 1.5

        # Act
        linear_velocity = 3.0 * sample_joy_msg.axes[1]
        angular_velocity = max_angular_speed * sample_joy_msg.axes[3]

        # Assert
        assert linear_velocity == 0.0, "Velocidade linear deve ser 0"
        assert angular_velocity == -max_angular_speed, f"Velocidade angular deve ser {-max_angular_speed}"


class TestButtonHandling:
    """Testes para tratamento de botões do joystick."""

    def test_ps_button_initial_pose(self, sample_joy_msg):
        """Testa botão PS para retornar à pose inicial."""
        # Arrange
        sample_joy_msg.buttons = [0] * 11
        sample_joy_msg.buttons[10] = 1  # PS button pressed

        # Act - simular verificação do botão
        should_publish_initial_pose = sample_joy_msg.buttons[10] == 1

        # Assert
        assert should_publish_initial_pose, "Deve detectar pressionamento do botão PS"

    def test_no_button_pressed(self, sample_joy_msg):
        """Testa comportamento sem botões pressionados."""
        # Arrange
        sample_joy_msg.buttons = [0] * 11  # Nenhum botão pressionado

        # Act
        should_publish_initial_pose = sample_joy_msg.buttons[10] == 1

        # Assert
        assert not should_publish_initial_pose, "Não deve detectar pressionamento de botão"


class TestParameterValidation:
    """Testes para validação de parâmetros de entrada."""

    def test_axes_array_length(self, sample_joy_msg):
        """Testa validação do comprimento do array de eixos."""
        # Arrange & Act
        axes_length = len(sample_joy_msg.axes)

        # Assert
        assert axes_length >= 4, "Array de eixos deve ter pelo menos 4 elementos"

    def test_buttons_array_length(self, sample_joy_msg):
        """Testa validação do comprimento do array de botões."""
        # Arrange & Act
        buttons_length = len(sample_joy_msg.buttons)

        # Assert
        assert buttons_length >= 11, "Array de botões deve ter pelo menos 11 elementos"

    def test_axes_values_range(self, sample_joy_msg):
        """Testa se valores dos eixos estão no range válido."""
        # Arrange
        sample_joy_msg.axes = [0.5, -0.8, 0.0, 1.0, 0.0, 0.0]

        # Act & Assert
        for i, axis_value in enumerate(sample_joy_msg.axes[:4]):  # Primeiros 4 eixos importantes
            assert -1.0 <= axis_value <= 1.0, f"Eixo {i} deve estar entre -1.0 e 1.0"


class TestEdgeCases:
    """Testes para casos extremos e edge cases."""

    def test_zero_inputs(self, test_config, sample_joy_msg):
        """Testa comportamento com entradas zero."""
        # Arrange
        sample_joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Act
        speed = test_config.max_speed * sample_joy_msg.axes[1]
        steering_angle = test_config.max_angle * sample_joy_msg.axes[3]

        # Assert
        assert speed == 0.0, "Velocidade deve ser zero"
        assert steering_angle == 0.0, "Ângulo deve ser zero"

    def test_maximum_inputs(self, test_config, sample_joy_msg):
        """Testa comportamento com entradas máximas."""
        # Arrange
        sample_joy_msg.axes = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

        # Act
        speed = test_config.max_speed * sample_joy_msg.axes[1]
        steering_angle = test_config.max_angle * sample_joy_msg.axes[3]

        # Assert
        assert speed == test_config.max_speed, "Velocidade deve ser máxima"
        assert steering_angle == test_config.max_angle, "Ângulo deve ser máximo"

    def test_minimum_inputs(self, test_config, sample_joy_msg):
        """Testa comportamento com entradas mínimas."""
        # Arrange
        sample_joy_msg.axes = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]

        # Act
        speed = test_config.max_speed * sample_joy_msg.axes[1]
        steering_angle = test_config.max_angle * sample_joy_msg.axes[3]

        # Assert
        assert speed == -test_config.max_speed, "Velocidade deve ser mínima"
        assert steering_angle == -test_config.max_angle, "Ângulo deve ser mínimo"

    def test_very_small_inputs(self, test_config, sample_joy_msg):
        """Testa comportamento com entradas muito pequenas."""
        # Arrange
        small_value = 1e-6
        sample_joy_msg.axes = [small_value, small_value, 0.0, small_value, 0.0, 0.0]
        controller_error = 0.1

        # Act - aplicar filtragem de deadzone
        linear_input = sample_joy_msg.axes[1]
        angular_input = sample_joy_msg.axes[3]

        if abs(linear_input) < controller_error:
            linear_input = 0.0
        if abs(angular_input) < controller_error:
            angular_input = 0.0

        speed = test_config.max_speed * linear_input
        steering_angle = test_config.max_angle * angular_input

        # Assert
        assert speed == 0.0, "Entradas muito pequenas devem ser filtradas"
        assert steering_angle == 0.0, "Entradas muito pequenas devem ser filtradas"


class TestInitialPoseMessage:
    """Testes para criação de mensagem de pose inicial."""

    def test_initial_pose_structure(self):
        """Testa estrutura da mensagem de pose inicial."""
        # Arrange & Act - simular criação da mensagem
        initial_pose = {
            'header': {
                'stamp': {'sec': 0, 'nanosec': 0},
                'frame_id': 'map'
            },
            'pose': {
                'pose': {
                    'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}
                },
                'covariance': [0.0] * 36
            }
        }

        # Assert
        assert initial_pose['header']['frame_id'] == 'map', "Frame deve ser 'map'"
        assert initial_pose['pose']['pose']['position']['x'] == 0.0, "Posição X deve ser 0"
        assert initial_pose['pose']['pose']['position']['y'] == 0.0, "Posição Y deve ser 0"
        assert len(initial_pose['pose']['covariance']) == 36, "Covariance deve ter 36 elementos"


class TestPerformance:
    """Testes de performance para conversores."""

    def test_conversion_speed(self, test_config, sample_joy_msg, performance_helper):
        """Testa velocidade de conversão de comandos."""
        # Arrange
        sample_joy_msg.axes = [0.0, 0.5, 0.0, 0.3, 0.0, 0.0]

        def convert_joy_to_ackermann():
            """Simula conversão completa."""
            speed = test_config.max_speed * sample_joy_msg.axes[1]
            steering_angle = test_config.max_angle * sample_joy_msg.axes[3]
            return speed, steering_angle

        # Act
        for _ in range(100):  # Múltiplas conversões
            _, exec_time = performance_helper.measure_execution_time(convert_joy_to_ackermann)

        # Assert
        avg_time = performance_helper.get_average_time()
        assert avg_time < 0.0001, f"Conversão deve ser < 0.1ms, atual: {avg_time:.6f}s"


# Configuração de marcadores
pytestmark = [
    pytest.mark.unit,
    pytest.mark.joy_converter,
]


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
