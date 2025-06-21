# 🏎️ F1TENTH Autonomous Racing System - Raspberry Pi

Sistema completo de corrida autônoma F1TENTH otimizado para **Raspberry Pi 4B** com ROS2 Humble.

## 🎯 Status do Sistema

- ✅ **Hardware VESC**: Funcional (controle direto de duty_cycle)
- ✅ **Servo GPIO**: Funcional (controle PWM via pigpio)
- ✅ **Sistema ROS2**: Integração completa entre nodes
- ✅ **Launch System**: Sistema unificado com configuração centralizada
- ✅ **Conversão Ackermann**: /drive → /commands/motor/duty_cycle funcionando
- 🔄 **LiDAR**: Driver instalado, integração em progresso
- 🔄 **Algoritmos**: SLAM e navegação autônoma (próxima fase)

## 🛠️ Configuração no Raspberry Pi

### 📥 1. Clonar e Atualizar o Repositório

```bash
# Clonar o repositório (primeira vez)
cd ~/Documents
git clone [URL_DO_SEU_REPOSITORIO] f1tenth_code_rasp

# OU atualizar repositório existente
cd ~/Documents/f1tenth_code_rasp
git pull origin main
```

### 🔧 2. Instalar Dependências do Sistema

```bash
# Tornar o script executável
chmod +x scripts/setup_raspberry_dependencies.sh

# Executar instalação completa (NÃO use sudo)
./scripts/setup_raspberry_dependencies.sh
```

**⚠️ IMPORTANTE**: Após a instalação das dependências:
```bash
# Fazer logout e login novamente para aplicar configurações de grupo
logout
# OU reiniciar o sistema
sudo reboot
```

### 🏗️ 3. Build do Sistema F1TENTH

```bash
# Navegar para o workspace
cd ~/Documents/f1tenth_code_rasp

# Executar build e validação completa
chmod +x scripts/build_and_test_f1tenth.sh
./scripts/build_and_test_f1tenth.sh
```

### 🚀 4. Teste do Sistema

#### **Teste Básico (Servo apenas)**
```bash
cd ~/Documents/f1tenth_code_rasp
source install/setup.bash
ros2 launch f1tenth_control f1tenth_control.launch.py
```

#### **Sistema Completo (Recomendado)**
```bash
cd ~/Documents/f1tenth_code_rasp
source install/setup.bash
ros2 launch f1tenth_control f1tenth_complete_system.launch.py
```

#### **Controle Manual (Joystick)**
```bash
# Em outro terminal, para controle manual
cd ~/Documents/f1tenth_code_rasp
source install/setup.bash
ros2 launch joy_converter launch_joy_ackerman_fixed.launch.py
```

## 📊 Monitoramento do Sistema

### **Verificar Comunicação ROS2**
```bash
# Listar nodes ativos
ros2 node list

# Listar tópicos
ros2 topic list

# Monitorar odometria
ros2 topic echo /ego_racecar/odom

# Monitorar comandos de controle
ros2 topic echo /drive

# Monitorar LiDAR (se ativo)
ros2 topic echo /scan

# Verificar frequência dos tópicos
ros2 topic hz /scan
ros2 topic hz /ego_racecar/odom
```

### **Diagnóstico de Hardware**
```bash
# Status do serviço pigpio
sudo systemctl status pigpiod

# Verificar grupos do usuário
groups $USER

# Verificar dispositivos USB (VESC, LiDAR)
lsusb
ls -la /dev/ttyUSB* /dev/ttyACM*

# Verificar YDLiDAR SDK
ls -la /usr/local/lib/libydlidar_sdk.so
```

### **Performance do Sistema**
```bash
# Uso de CPU e memória
htop

# Processos ROS2
ps aux | grep ros

# Uso de rede (tópicos ROS2)
ros2 topic bw /scan
ros2 topic bw /drive
```

## 🎮 Controle Manual

### **Usando Joystick 8BitDo**
1. Conectar joystick via Bluetooth
2. Pressionar **LB (botão esquerdo)** para ativar controle
3. **Analógico esquerdo (Y)**: Acelerar/freiar
4. **Analógico direito (X)**: Direção

### **Usando Teclado** (para testes)
```bash
# Executar conversor de teclado
ros2 run joy_converter joy_keyboard_converter
```

## 📋 Estrutura do Sistema

```
f1tenth_code_rasp/
├── src/
│   ├── f1tenth_control/          # Controle principal (servo + integração)
│   ├── joy_converter/            # Conversores de entrada (joystick/teclado)
│   ├── vesc-humble/              # Driver VESC para ROS2 Humble
│   └── ydlidar_ros2_driver/      # Driver LiDAR YDLIDAR
├── scripts/
│   ├── setup_raspberry_dependencies.sh    # Instalação de dependências
│   ├── build_and_test_f1tenth.sh         # Build e teste do sistema
│   └── f1tenth_startup.sh                # Inicialização automática
└── config/
    └── system_config.yaml        # Configuração centralizada
```

## 🔧 Arquitetura do Sistema

```mermaid
graph TB
    A[Joystick/Keyboard] --> B[Joy Converter]
    B --> C[/drive topic<br/>AckermannDriveStamped]
    C --> D[Ackermann to VESC]
    C --> E[Servo Control Node]
    D --> F[VESC Driver]
    F --> G[Motor]
    E --> H[Servo GPIO]
    I[VESC] --> J[Vesc to Odom]
    J --> K[/ego_racecar/odom]
    L[YDLiDAR] --> M[/scan topic]
    
    style C fill:#e1f5fe
    style G fill:#c8e6c9
    style H fill:#c8e6c9
    style K fill:#fff3e0
    style M fill:#f3e5f5
```

## ⚠️ Troubleshooting

### **Build Falha**
```bash
# Limpar build e tentar novamente
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
colcon build --symlink-install
```

### **Erro de Permissão (GPIO/Serial)**
```bash
# Verificar grupos
groups $USER

# Se necessário, adicionar aos grupos
sudo usermod -a -G dialout $USER
sudo usermod -a -G gpio $USER
# Fazer logout/login após adicionar aos grupos
```

### **YDLiDAR Não Detectado**
```bash
# Verificar se o SDK está instalado
ls -la /usr/local/lib/libydlidar_sdk.so

# Verificar conexão USB
lsusb
ls -la /dev/ttyUSB*

# Reinstalar dependências se necessário
./scripts/setup_raspberry_dependencies.sh
```

### **VESC Não Responde**
```bash
# Verificar conexão serial
ls -la /dev/ttyACM*

# Testar comunicação direta
ros2 topic pub /commands/motor/duty_cycle std_msgs/msg/Float64 "data: 0.1" --once

# Verificar parâmetros VESC
ros2 param list /vesc_driver
```

## 🎯 Próximos Desenvolvimentos

1. **SLAM e Mapeamento**: Integração completa do LiDAR para mapeamento
2. **Navegação Autônoma**: Algoritmos de path planning e obstacle avoidance
3. **Controle Avançado**: PID tuning e controle de velocidade
4. **Safety Systems**: Sistema de parada de emergência melhorado
5. **Performance**: Otimizações para real-time racing

## 📞 Suporte

Para problemas ou dúvidas:
1. Verificar logs: `ros2 launch` com `--ros-args --log-level debug`
2. Executar diagnóstico: `./scripts/build_and_test_f1tenth.sh`
3. Verificar hardware: Conexões físicas e alimentação

---

> 🏁 **F1TENTH Racing Team** - Sistema otimizado para corrida autônoma em escala 1:10
