#!/usr/bin/env python3
"""
Script de Diagnóstico YDLiDAR X4
================================
Detecta automaticamente a porta USB do YDLiDAR e testa conectividade

Uso:
    python3 scripts/diagnose_lidar.py
"""

import os
import sys
import glob
import serial
import time
import subprocess
from pathlib import Path

def check_usb_devices():
    """Verifica dispositivos USB conectados"""
    print("🔍 VERIFICANDO DISPOSITIVOS USB...")
    
    try:
        result = subprocess.run(['lsusb'], capture_output=True, text=True)
        usb_devices = result.stdout.strip().split('\n')
        
        print(f"📋 Dispositivos USB detectados ({len(usb_devices)}):")
        for device in usb_devices:
            print(f"  - {device}")
            
        # Procurar por YDLiDAR específico
        ydlidar_found = False
        for device in usb_devices:
            if any(keyword in device.lower() for keyword in ['ydlidar', 'lidar', 'laser']):
                print(f"✅ YDLiDAR detectado: {device}")
                ydlidar_found = True
                
        if not ydlidar_found:
            print("❌ YDLiDAR não detectado via lsusb")
            
    except Exception as e:
        print(f"❌ Erro ao verificar USB: {e}")

def check_serial_ports():
    """Verifica portas seriais disponíveis"""
    print("\n🔍 VERIFICANDO PORTAS SERIAIS...")
    
    # Verificar portas comuns
    common_ports = ['/dev/ttyUSB*', '/dev/ttyACM*', '/dev/ttyS*']
    found_ports = []
    
    for pattern in common_ports:
        ports = glob.glob(pattern)
        found_ports.extend(ports)
    
    print(f"📋 Portas seriais encontradas ({len(found_ports)}):")
    for port in sorted(found_ports):
        try:
            # Verificar permissões
            stat = os.stat(port)
            permissions = oct(stat.st_mode)[-3:]
            print(f"  - {port} (permissões: {permissions})")
        except Exception as e:
            print(f"  - {port} (erro: {e})")
    
    return found_ports

def test_serial_connection(port, baudrates=[115200, 128000, 230400]):
    """Testa conexão serial com diferentes baudrates"""
    print(f"\n🔍 TESTANDO CONEXÃO: {port}")
    
    for baudrate in baudrates:
        try:
            print(f"  📡 Testando baudrate {baudrate}...")
            
            # Tentar abrir porta
            with serial.Serial(port, baudrate, timeout=2) as ser:
                print(f"    ✅ Porta aberta com sucesso")
                
                # Tentar comunicação básica
                ser.write(b'b')  # Comando básico YDLiDAR
                time.sleep(0.1)
                
                if ser.in_waiting > 0:
                    response = ser.read(ser.in_waiting)
                    print(f"    ✅ Resposta recebida: {len(response)} bytes")
                    return True, baudrate
                else:
                    print(f"    ⚠️ Nenhuma resposta")
                    
        except serial.SerialException as e:
            print(f"    ❌ Erro serial: {e}")
        except Exception as e:
            print(f"    ❌ Erro geral: {e}")
    
    return False, None

def check_permissions():
    """Verifica permissões do usuário"""
    print("\n🔍 VERIFICANDO PERMISSÕES...")
    
    user = os.getenv('USER')
    print(f"👤 Usuário atual: {user}")
    
    # Verificar grupos
    try:
        result = subprocess.run(['groups'], capture_output=True, text=True)
        groups = result.stdout.strip().split()
        print(f"👥 Grupos do usuário: {', '.join(groups)}")
        
        required_groups = ['dialout', 'tty']
        for group in required_groups:
            if group in groups:
                print(f"  ✅ Grupo {group}: OK")
            else:
                print(f"  ❌ Grupo {group}: AUSENTE")
                print(f"     Solução: sudo usermod -a -G {group} {user}")
                
    except Exception as e:
        print(f"❌ Erro ao verificar grupos: {e}")

def suggest_fixes():
    """Sugere correções baseadas no diagnóstico"""
    print("\n🔧 SUGESTÕES DE CORREÇÃO:")
    
    print("1. 📱 VERIFICAR CONEXÃO FÍSICA:")
    print("   - Cabo USB conectado firmemente")
    print("   - YDLiDAR ligado (LED indicador)")
    print("   - Testar em outra porta USB")
    
    print("\n2. 🔌 VERIFICAR ALIMENTAÇÃO:")
    print("   - YDLiDAR X4 precisa 5V/1A")
    print("   - Usar hub USB com alimentação se necessário")
    
    print("\n3. 🛠️ COMANDOS DE CORREÇÃO:")
    print("   # Adicionar usuário aos grupos necessários")
    print("   sudo usermod -a -G dialout $USER")
    print("   sudo usermod -a -G tty $USER")
    print()
    print("   # Dar permissões temporárias")
    print("   sudo chmod 666 /dev/ttyUSB*")
    print("   sudo chmod 666 /dev/ttyACM*")
    print()
    print("   # Regra udev permanente")
    print("   echo 'KERNEL==\"ttyUSB*\", MODE=\"0666\"' | sudo tee /etc/udev/rules.d/99-ydlidar.rules")
    print("   sudo udevadm control --reload-rules")
    
    print("\n4. 📋 TESTAR SISTEMA SEM LIDAR:")
    print("   ros2 launch f1tenth_control f1tenth_system_no_lidar.launch.py")

def main():
    """Função principal de diagnóstico"""
    print("🏎️ DIAGNÓSTICO YDLiDAR X4 - F1TENTH SYSTEM")
    print("=" * 50)
    
    # Verificações sequenciais
    check_usb_devices()
    ports = check_serial_ports()
    check_permissions()
    
    # Testar portas encontradas
    if ports:
        print(f"\n🧪 TESTANDO COMUNICAÇÃO COM {len(ports)} PORTAS...")
        
        working_ports = []
        for port in ports:
            success, baudrate = test_serial_connection(port)
            if success:
                working_ports.append((port, baudrate))
        
        if working_ports:
            print(f"\n✅ PORTAS FUNCIONAIS ENCONTRADAS:")
            for port, baudrate in working_ports:
                print(f"  - {port} @ {baudrate} baud")
                print(f"    Configuração: port: {port}")
                print(f"    Configuração: baudrate: {baudrate}")
        else:
            print(f"\n❌ NENHUMA PORTA FUNCIONAL ENCONTRADA")
    
    # Sugestões finais
    suggest_fixes()
    
    print("\n" + "=" * 50)
    print("🏁 DIAGNÓSTICO CONCLUÍDO")

if __name__ == "__main__":
    main() 