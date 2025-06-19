#!/usr/bin/env python3
"""
Mock do módulo pigpio para permitir testes unitários sem hardware físico.

Este mock simula o comportamento do pigpio mantendo estado interno
para validar operações GPIO e PWM.

Autor: Professor PhD em Engenharia Robótica
"""

import time
import threading
from typing import Dict, Any, Optional
from unittest.mock import MagicMock

# Constantes do pigpio
OUTPUT = 1
INPUT = 0

class MockPigpio:
    """Mock da classe principal do pigpio."""

    def __init__(self, host: str = 'localhost', port: int = 8888):
        self.host = host
        self.port = port
        self._connected = True
        self._pins: Dict[int, Dict[str, Any]] = {}
        self._lock = threading.RLock()

        # Estatísticas para validação
        self.stats = {
            'gpio_operations': 0,
            'pwm_operations': 0,
            'servo_operations': 0,
            'mode_changes': 0
        }

    @property
    def connected(self) -> bool:
        """Retorna status da conexão."""
        return self._connected

    def stop(self):
        """Simula desconexão do daemon pigpio."""
        with self._lock:
            self._connected = False

    def set_mode(self, gpio: int, mode: int) -> int:
        """Simula configuração do modo de um pino GPIO."""
        with self._lock:
            if not self._connected:
                raise RuntimeError("Not connected to pigpio daemon")

            if gpio < 0 or gpio > 53:  # GPIOs válidos no Raspberry Pi
                return -1

            if gpio not in self._pins:
                self._pins[gpio] = {}

            self._pins[gpio]['mode'] = mode
            self.stats['mode_changes'] += 1

            return 0  # Sucesso

    def get_mode(self, gpio: int) -> int:
        """Retorna o modo configurado de um pino."""
        with self._lock:
            if gpio in self._pins and 'mode' in self._pins[gpio]:
                return self._pins[gpio]['mode']
            return -1  # Pino não configurado

    def set_PWM_frequency(self, gpio: int, frequency: int) -> int:
        """Simula configuração da frequência PWM."""
        with self._lock:
            if not self._connected:
                raise RuntimeError("Not connected to pigpio daemon")

            if gpio < 0 or gpio > 53:
                return -1

            if frequency < 1 or frequency > 40000:  # Limites do pigpio
                return -1

            if gpio not in self._pins:
                self._pins[gpio] = {}

            self._pins[gpio]['pwm_frequency'] = frequency
            self.stats['pwm_operations'] += 1

            return frequency  # Retorna a frequência configurada

    def get_PWM_frequency(self, gpio: int) -> int:
        """Retorna a frequência PWM configurada."""
        with self._lock:
            if gpio in self._pins and 'pwm_frequency' in self._pins[gpio]:
                return self._pins[gpio]['pwm_frequency']
            return 800  # Frequência padrão

    def set_servo_pulsewidth(self, gpio: int, pulsewidth: int) -> int:
        """Simula controle de servo via largura de pulso."""
        with self._lock:
            if not self._connected:
                raise RuntimeError("Not connected to pigpio daemon")

            if gpio < 0 or gpio > 53:
                return -1

            # Validar largura de pulso (0 = off, 500-2500µs válidos)
            if pulsewidth != 0 and (pulsewidth < 500 or pulsewidth > 2500):
                return -1

            if gpio not in self._pins:
                self._pins[gpio] = {}

            self._pins[gpio]['servo_pulsewidth'] = pulsewidth
            self._pins[gpio]['last_servo_update'] = time.time()
            self.stats['servo_operations'] += 1

            return 0  # Sucesso

    def get_servo_pulsewidth(self, gpio: int) -> int:
        """Retorna a largura de pulso configurada do servo."""
        with self._lock:
            if gpio in self._pins and 'servo_pulsewidth' in self._pins[gpio]:
                return self._pins[gpio]['servo_pulsewidth']
            return 0  # Servo não configurado

    def write(self, gpio: int, level: int) -> int:
        """Simula escrita digital em um pino."""
        with self._lock:
            if not self._connected:
                raise RuntimeError("Not connected to pigpio daemon")

            if gpio < 0 or gpio > 53:
                return -1

            if level not in [0, 1]:
                return -1

            if gpio not in self._pins:
                self._pins[gpio] = {}

            self._pins[gpio]['level'] = level
            self.stats['gpio_operations'] += 1

            return 0

    def read(self, gpio: int) -> int:
        """Simula leitura digital de um pino."""
        with self._lock:
            if gpio in self._pins and 'level' in self._pins[gpio]:
                return self._pins[gpio]['level']
            return 0  # Padrão LOW

    def set_PWM_dutycycle(self, gpio: int, dutycycle: int) -> int:
        """Simula configuração do duty cycle PWM."""
        with self._lock:
            if not self._connected:
                raise RuntimeError("Not connected to pigpio daemon")

            if gpio < 0 or gpio > 53:
                return -1

            if dutycycle < 0 or dutycycle > 255:
                return -1

            if gpio not in self._pins:
                self._pins[gpio] = {}

            self._pins[gpio]['pwm_dutycycle'] = dutycycle
            self.stats['pwm_operations'] += 1

            return 0

    def get_PWM_dutycycle(self, gpio: int) -> int:
        """Retorna o duty cycle PWM configurado."""
        with self._lock:
            if gpio in self._pins and 'pwm_dutycycle' in self._pins[gpio]:
                return self._pins[gpio]['pwm_dutycycle']
            return 0

    def get_pin_state(self, gpio: int) -> Dict[str, Any]:
        """Retorna estado completo de um pino (para testes)."""
        with self._lock:
            return self._pins.get(gpio, {}).copy()

    def get_all_pins_state(self) -> Dict[int, Dict[str, Any]]:
        """Retorna estado de todos os pinos (para testes)."""
        with self._lock:
            return {gpio: state.copy() for gpio, state in self._pins.items()}

    def reset_stats(self):
        """Reseta estatísticas de operações."""
        with self._lock:
            self.stats = {
                'gpio_operations': 0,
                'pwm_operations': 0,
                'servo_operations': 0,
                'mode_changes': 0
            }

    def simulate_disconnect(self):
        """Simula perda de conexão com o daemon."""
        with self._lock:
            self._connected = False

    def simulate_reconnect(self):
        """Simula reconexão com o daemon."""
        with self._lock:
            self._connected = True


class MockPigpioFailure(MockPigpio):
    """Mock que simula falhas de conexão pigpio."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._connected = False

    def stop(self):
        pass  # Não faz nada


# Funções de conveniência para mocking
def create_mock_pi(connected: bool = True) -> MockPigpio:
    """Cria uma instância mock do pigpio."""
    if connected:
        return MockPigpio()
    else:
        return MockPigpioFailure()


def patch_pigpio_module():
    """Retorna um patch para substituir o módulo pigpio inteiro."""
    import sys
    from unittest.mock import MagicMock

    # Criar módulo mock
    mock_module = MagicMock()
    mock_module.OUTPUT = OUTPUT
    mock_module.INPUT = INPUT
    mock_module.pi = MockPigpio

    return mock_module


# Contexto para testes que requerem pigpio
class MockPigpioContext:
    """Context manager para usar mock pigpio em testes."""

    def __init__(self, connected: bool = True, fail_after: Optional[int] = None):
        self.connected = connected
        self.fail_after = fail_after
        self.mock_pi = None
        self.operation_count = 0

    def __enter__(self):
        self.mock_pi = create_mock_pi(self.connected)

        # Interceptar operações para simular falha
        if self.fail_after is not None:
            original_servo = self.mock_pi.set_servo_pulsewidth

            def failing_servo(*args, **kwargs):
                self.operation_count += 1
                if self.operation_count > self.fail_after:
                    self.mock_pi.simulate_disconnect()
                    raise RuntimeError("Simulated pigpio failure")
                return original_servo(*args, **kwargs)

            self.mock_pi.set_servo_pulsewidth = failing_servo

        return self.mock_pi

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.mock_pi:
            self.mock_pi.stop()


# Para compatibilidade com código existente
pi = create_mock_pi
