#!/usr/bin/env python3
"""
Script de teste para validar a calibração do servo atualizada
Testa os valores descobertos empiricamente
"""

try:
    import pigpio  # noqa: F401
except ImportError:
    print("Erro: pigpio não instalado")
    exit(1)

import time

def test_calibrated_values():
    """Testa os valores de calibração descobertos"""
    
    # Valores calibrados descobertos
    CENTER = 1175
    LEFT_MAX = 850  
    RIGHT_MAX = 1500
    GPIO_PIN = 18
    
    print("=== TESTE DE CALIBRAÇÃO SERVO F1TENTH ===")
    print(f"Centro: {CENTER}µs")
    print(f"Esquerda máxima: {LEFT_MAX}µs") 
    print(f"Direita máxima: {RIGHT_MAX}µs")
    print("=========================================")
    
    # Conectar ao pigpio
    pi = pigpio.pi()
    if not pi.connected:
        print("Erro: não foi possível conectar ao pigpiod")
        return False
        
    try:
        print("\n1. Movendo para o CENTRO...")
        pi.set_servo_pulsewidth(GPIO_PIN, CENTER)
        input("   O servo está centralizado? [Enter para continuar]")
        
        print("\n2. Movendo para ESQUERDA MÁXIMA...")
        pi.set_servo_pulsewidth(GPIO_PIN, LEFT_MAX)
        input("   O servo está no extremo esquerdo? [Enter para continuar]")
        
        print("\n3. Movendo para DIREITA MÁXIMA...")
        pi.set_servo_pulsewidth(GPIO_PIN, RIGHT_MAX)
        input("   O servo está no extremo direito? [Enter para continuar]")
        
        print("\n4. Retornando ao CENTRO...")
        pi.set_servo_pulsewidth(GPIO_PIN, CENTER)
        time.sleep(1)
        
        print("\n5. Teste de sequência...")
        for i in range(3):
            print(f"   Ciclo {i+1}/3...")
            pi.set_servo_pulsewidth(GPIO_PIN, LEFT_MAX)
            time.sleep(0.5)
            pi.set_servo_pulsewidth(GPIO_PIN, CENTER)
            time.sleep(0.5)
            pi.set_servo_pulsewidth(GPIO_PIN, RIGHT_MAX)
            time.sleep(0.5)
            pi.set_servo_pulsewidth(GPIO_PIN, CENTER)
            time.sleep(0.5)
            
        print("\n✅ TESTE CONCLUÍDO!")
        print("Valores de calibração confirmados:")
        print(f"  servo_min_pulse_width: {LEFT_MAX}")
        print(f"  servo_center_pulse_width: {CENTER}")
        print(f"  servo_max_pulse_width: {RIGHT_MAX}")
        
    except KeyboardInterrupt:
        print("\nTeste interrompido pelo usuário")
    except Exception as e:
        print(f"Erro durante teste: {e}")
    finally:
        # Centralizar e desligar
        pi.set_servo_pulsewidth(GPIO_PIN, CENTER)
        time.sleep(0.5)
        pi.set_servo_pulsewidth(GPIO_PIN, 0)
        pi.stop()
        print("GPIO liberado.")
        
    return True

def main(args=None):
    """Entry point para ROS2"""
    return test_calibrated_values()

if __name__ == "__main__":
    test_calibrated_values() 