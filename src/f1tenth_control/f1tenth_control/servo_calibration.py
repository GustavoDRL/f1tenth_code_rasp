#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# Tentativa de importação segura para compatibilidade com Windows
try:
    import pigpio
except ImportError:
    print("\n*** Atenção: Biblioteca pigpio não encontrada. O controle de GPIO não funcionará. ***")
    print("*** Instale em sistemas baseados em Debian/Ubuntu com: sudo apt install python3-pigpio ***")
    print("*** Lembre-se de iniciar o daemon: sudo systemctl start pigpiod ***\n")
    pigpio = None # Define como None se não puder ser importado

import time
import sys

class ServoCalibrationNode(Node):
    def __init__(self):
        super().__init__('servo_calibration_node')
        
        # Parâmetros padrão (podem ser sobrescritos via linha de comando ou launch file)
        self.declare_parameter('gpio_pin', 18)
        self.declare_parameter('frequency', 50)
        
        self.gpio_pin = self.get_parameter('gpio_pin').value
        self.frequency = self.get_parameter('frequency').value
        
        self.pi = None
        if pigpio:
            try:
                self.pi = pigpio.pi()
                if not self.pi.connected:
                    self.get_logger().error('Não foi possível conectar ao daemon pigpio. Saindo.')
                    rclpy.shutdown()
                    sys.exit(1)
                
                # Configurar o pino GPIO
                self.pi.set_mode(self.gpio_pin, pigpio.OUTPUT)
                self.pi.set_PWM_frequency(self.gpio_pin, self.frequency)
                self.get_logger().info(f"Conectado ao pigpio. Controlando GPIO {self.gpio_pin} com freq {self.frequency} Hz.")

            except Exception as e:
                self.get_logger().error(f'Erro ao inicializar pigpio: {e}. Saindo.')
                rclpy.shutdown()
                sys.exit(1)
        else:
            self.get_logger().error("Biblioteca pigpio não encontrada. Este script requer pigpio para funcionar. Saindo.")
            rclpy.shutdown()
            sys.exit(1)
        
        self.get_logger().info("""
        ----------------------------------------
        Iniciando calibração interativa do servo
        ----------------------------------------
        Use +/- para ajustar (passo largo)
        Use a/d para ajustar (passo fino -)
        Use w/s para ajustar (passo fino +)
        Use c para centralizar (1500 µs)
        Use 1 para definir mínimo
        Use 2 para definir centro (opcional)
        Use 3 para definir máximo
        Use q para salvar e sair
        ----------------------------------------
        Anote os valores MIN e MAX para control_params.yaml
        """)
        
        # Posição central inicial
        self.current_pulse_width = 1500  # Valor típico para posição central
        self.min_pulse_width = None
        self.center_pulse_width = None
        self.max_pulse_width = None
        self.set_pulse_width(self.current_pulse_width)
        
    def set_pulse_width(self, pulse_width):
        """Define a largura de pulso em microssegundos"""
        if not self.pi: return # Não faz nada se pigpio não estiver disponível
        
        # Limita a largura de pulso a um range seguro (geralmente 500-2500 para servos)
        pulse_width = int(min(max(pulse_width, 500), 2500))
        self.current_pulse_width = pulse_width
        
        try:
            self.pi.set_servo_pulsewidth(self.gpio_pin, self.current_pulse_width)
            self.get_logger().info(f'Pulse width: {self.current_pulse_width} µs', skip_first=True, throttle_duration_sec=0.1)
        except Exception as e:
            self.get_logger().error(f'Erro ao definir largura de pulso: {e}')
        
    def run_interactive(self):
        """Modo interativo para calibração"""
        large_increment = 10  # Incremento/decremento largo
        small_increment = 1   # Incremento/decremento fino
        
        print(f"\nPosição inicial: {self.current_pulse_width} µs")
        
        # Loop principal de interação
        while rclpy.ok():
            cmd = input("> ").lower()
            
            if cmd == 'q':
                break
            elif cmd == '+':
                self.set_pulse_width(self.current_pulse_width + large_increment)
            elif cmd == '-':
                self.set_pulse_width(self.current_pulse_width - large_increment)
            elif cmd == 'w':
                self.set_pulse_width(self.current_pulse_width + small_increment)
            elif cmd == 's':
                self.set_pulse_width(self.current_pulse_width - small_increment)
            elif cmd == 'd': # Ajuste mais rápido
                self.set_pulse_width(self.current_pulse_width + 5)
            elif cmd == 'a': # Ajuste mais rápido
                self.set_pulse_width(self.current_pulse_width - 5)
            elif cmd == 'c': # Centralizar
                self.set_pulse_width(1500)
            elif cmd == '1':
                self.min_pulse_width = self.current_pulse_width
                self.get_logger().info(f'>>> Posição MÍNIMA definida: {self.min_pulse_width} µs <<<')
            elif cmd == '2':
                self.center_pulse_width = self.current_pulse_width
                self.get_logger().info(f'>>> Posição CENTRAL definida: {self.center_pulse_width} µs <<<')
            elif cmd == '3':
                self.max_pulse_width = self.current_pulse_width
                self.get_logger().info(f'>>> Posição MÁXIMA definida: {self.max_pulse_width} µs <<<')
            else:
                 self.get_logger().warn(f"Comando inválido: '{cmd}'")
                
        # No final, mostrar os valores para configuração
        print("\n----------------------------------------")
        print("  Resultados da Calibração")
        print("----------------------------------------")
        if self.min_pulse_width is not None:
            print(f"servo_min_pulse_width: {self.min_pulse_width}")
        else:
            print("servo_min_pulse_width: [NÃO DEFINIDO]")
            
        # Centro é opcional, mas bom ter como referência
        # if self.center_pulse_width is not None:
        #     print(f"servo_center_pulse_width: {self.center_pulse_width}")
            
        if self.max_pulse_width is not None:
            print(f"servo_max_pulse_width: {self.max_pulse_width}")
        else:
            print("servo_max_pulse_width: [NÃO DEFINIDO]")
        print("----------------------------------------")
        print("Atualize seu arquivo 'control_params.yaml' com os valores MIN e MAX.")
        print("----------------------------------------")
        
        # Centralizar o servo antes de sair e liberar GPIO
        self.destroy()
        
    def destroy(self):
        """Libera recursos GPIO."""
        if self.pi and self.pi.connected:
            self.get_logger().info('Centralizando servo e liberando GPIO...')
            self.pi.set_servo_pulsewidth(self.gpio_pin, 1500)
            time.sleep(0.5)
            self.pi.set_PWM_dutycycle(self.gpio_pin, 0) # Desliga PWM
            self.pi.stop()
            self.get_logger().info('GPIO liberado.')

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ServoCalibrationNode()
        node.run_interactive()
    except KeyboardInterrupt:
        print("\nCalibração interrompida pelo usuário.")
    except Exception as e:
        print(f"\nErro inesperado durante calibração: {e}")
    finally:
        # Garante que o GPIO seja liberado mesmo em caso de erro
        if node and node.pi and node.pi.connected:
            node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 