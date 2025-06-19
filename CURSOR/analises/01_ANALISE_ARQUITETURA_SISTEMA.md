# 🏗️ ANÁLISE DA ARQUITETURA DO SISTEMA F1TENTH

**Documento**: Arquitetura Completa do Sistema
**Versão**: 1.0
**Data**: 2025-01-20
**Responsável**: Análise Automática de Código

---

## 📋 RESUMO EXECUTIVO

O sistema F1TENTH implementa uma arquitetura distribuída baseada em ROS2, projetada para operar em Raspberry Pi 4B. A arquitetura segue o padrão de decomposição por responsabilidades, com separação clara entre controle de motor, direção, interface de usuário e percepção.

### **Características Principais**
- **Paradigma**: Sistema distribuído com comunicação via tópicos ROS2
- **Hardware Alvo**: Raspberry Pi 4B com Ubuntu 22.04 + ROS2 Humble
- **Estratégia de Controle**: Híbrida (VESC para motor + GPIO para servo)
- **Interface**: Joystick + preparação para algoritmos autônomos
- **Escalabilidade**: Arquitetura preparada para extensões (LiDAR, navegação)

---

## 🎯 ARQUITETURA GERAL

### **Visão de Alto Nível**

```
┌─────────────────────────────────────────────────────────────────┐
│                     SISTEMA F1TENTH                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐    ┌──────────────┐    ┌─────────────────┐  │
│  │   USUÁRIO   │    │   PERCEPÇÃO  │    │   NAVEGAÇÃO     │  │
│  │             │    │              │    │   AUTÔNOMA      │  │
│  │ • Joystick  │    │ • LiDAR      │    │ • Path Planning │  │
│  │ • Gamepad   │    │ • Odometria  │    │ • SLAM          │  │
│  └─────────────┘    └──────────────┘    └─────────────────┘  │
│         │                   │                     │           │
│         ▼                   ▼                     ▼           │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │              CAMADA DE CONTROLE ROS2                   │  │
│  │                                                         │  │
│  │  ┌──────────────┐  ┌─────────────┐  ┌───────────────┐ │  │
│  │  │Joy_converter │  │f1tenth_     │  │vesc_ackermann │ │  │
│  │  │              │  │control      │  │               │ │  │
│  │  │• joy→acker   │  │• servo ctrl │  │• acker→vesc   │ │  │
│  │  │• joy→twist   │  │• odometry   │  │• vesc→odom    │ │  │
│  │  └──────────────┘  └─────────────┘  └───────────────┘ │  │
│  └─────────────────────────────────────────────────────────┘  │
│         │                   │                     │           │
│         ▼                   ▼                     ▼           │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │                CAMADA DE HARDWARE                      │  │
│  │                                                         │  │
│  │  ┌──────────────┐  ┌─────────────┐  ┌───────────────┐ │  │
│  │  │   GPIO       │  │    USB      │  │    USB/SPI    │ │  │
│  │  │              │  │             │  │               │ │  │
│  │  │• Servo PWM   │  │• VESC       │  │• LiDAR        │ │  │
│  │  │• Pino 18     │  │• /dev/ACM0  │  │• /dev/ttyUSB  │ │  │
│  │  └──────────────┘  └─────────────┘  └───────────────┘ │  │
│  └─────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🔧 COMPONENTES PRINCIPAIS

### **1. CAMADA DE HARDWARE**

#### **Hardware Computacional**
- **Raspberry Pi 4B**: Computador principal
  - CPU: ARM Cortex-A72 quad-core 1.5GHz
  - RAM: 4GB+ recomendado
  - Storage: MicroSD 32GB+ (Classe 10)
  - GPIO: 40 pinos para controle direto

#### **Interfaces de Hardware**
```
┌─────────────────────────────────────┐
│         RASPBERRY PI 4B             │
├─────────────────────────────────────┤
│                                     │
│  GPIO 18 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓│
│                                     ║│ PWM
│  USB 2.0 ━━━━━━━━━━━━━━━━━━━━━━━━━━━┓ ▼│
│                                   ┗━ SERVO RC
│  USB 3.0 ━━━━━━━━━━━━━━━━━━━━━━━━━━━┓ │
│                                   ┗━ JOYSTICK
│  USB-C ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓ │
│                                   ┗━ VESC (/dev/ttyACM0)
│                                     │
│  (Preparado: USB/SPI) ━━━━━━━━━━━━━┓ │
│                                   ┗━ YDLiDAR
└─────────────────────────────────────┘
```

### **2. CAMADA DE DRIVERS ROS2**

#### **A. Driver VESC (vesc-humble)**
**Funcionalidade**: Interface com controlador de motor VESC
```python
# Estrutura do pacote vesc-humble/
├── vesc_driver/          # Driver principal
│   ├── src/vesc_driver.cpp      # Interface serial VESC
│   ├── include/vesc_driver.hpp  # Headers
│   └── params/vesc_config.yaml  # Configurações
├── vesc_ackermann/       # Conversões
│   ├── src/ackermann_to_vesc.cpp    # Ackermann → VESC
│   └── src/vesc_to_odom.cpp         # VESC → Odometria
└── vesc_msgs/           # Mensagens ROS2
    └── msg/VescState.msg        # Estado do VESC
```

**Tópicos Principais**:
- `/commands/motor/speed` (Float64) - Comando velocidade
- `/sensors/core` (VescState) - Estado sensor VESC
- `/odom` (Odometry) - Odometria calculada

#### **B. Driver Servo (f1tenth_control)**
**Funcionalidade**: Controle direto via GPIO + PID opcional
```python
# Estrutura do pacote f1tenth_control/
├── servo_control_node.py          # Implementação básica
├── enhanced_servo_control_node.py # Implementação com PID
└── servo_calibration.py           # Ferramenta calibração
```

**Características Técnicas**:
- **Interface**: pigpio para controle PWM preciso
- **Frequência**: 50Hz (padrão servos RC)
- **Range PWM**: 1000-2000µs (configurável)
- **Controle**: PID opcional para suavização

#### **C. Interface Joystick (Joy_converter)**
**Funcionalidade**: Conversão comandos joystick
```python
# Implementações disponíveis:
├── joy_ackerman.py    # Joystick → AckermannDriveStamped
└── joy_twist.py       # Joystick → Twist (differential drive)
```

### **3. CAMADA DE COMUNICAÇÃO ROS2**

#### **Tópicos Principais**
| Tópico | Tipo | Direção | Função |
|--------|------|---------|---------|
| `/joy` | sensor_msgs/Joy | IN | Comandos joystick |
| `/drive` | ackermann_msgs/AckermannDriveStamped | INTERNAL | Comandos Ackermann |
| `/cmd_vel` | geometry_msgs/Twist | INTERNAL | Comandos Twist |
| `/commands/motor/speed` | std_msgs/Float64 | OUT→VESC | Velocidade motor |
| `/sensors/core` | vesc_msgs/VescState | IN←VESC | Estado VESC |
| `/odom` | nav_msgs/Odometry | INTERNAL | Odometria VESC |
| `/ego_racecar/odom` | nav_msgs/Odometry | OUT | Odometria F1TENTH |
| `/scan` | sensor_msgs/LaserScan | IN←LiDAR | Dados LiDAR |

#### **Frames TF**
```
map
 └── odom
     └── base_link
         ├── laser_frame (LiDAR)
         └── (outros sensores)
```

---

## ⚙️ FLUXO DE DADOS DETALHADO

### **Cenário 1: Controle Manual via Joystick**
```
1. ENTRADA: Usuário movimenta joystick
   ├── /joy (sensor_msgs/Joy)

2. CONVERSÃO: joy_ackerman.py processa
   ├── Eixo vertical → velocidade
   ├── Eixo horizontal → ângulo direção
   ├── /drive (AckermannDriveStamped)

3. DISTRIBUIÇÃO:
   ├── VELOCIDADE: ackermann_to_vesc.cpp
   │   ├── Converte velocidade → ERPM
   │   └── /commands/motor/speed → VESC
   │
   └── DIREÇÃO: servo_control_node.py
       ├── Converte ângulo → PWM
       └── GPIO 18 → Servo RC

4. FEEDBACK: vesc_to_odom.cpp
   ├── /sensors/core (VESC estado)
   ├── Calcula odometria
   ├── /odom (nav_msgs/Odometry)
   └── /ego_racecar/odom (republicação)
```

### **Cenário 2: Controle Autônomo (Futuro)**
```
1. ENTRADA: Algoritmo navegação
   ├── Recebe /scan (LiDAR)
   ├── Processa /ego_racecar/odom
   ├── Calcula trajetória

2. COMANDO: Mesmo fluxo do manual
   ├── /drive (AckermannDriveStamped)
   └── [Resto idêntico ao Cenário 1]
```

---

## 🚀 ASPECTOS DE PERFORMANCE

### **Frequências de Operação**
- **Controle Servo**: 100Hz (enhanced_servo_control)
- **PWM Servo**: 50Hz (hardware)
- **Comunicação VESC**: ~100Hz
- **Joystick**: ~50Hz
- **Odometria**: ~100Hz

### **Latências Estimadas**
- **Joystick → Ação**: <50ms
- **Comando → Motor**: <20ms
- **Comando → Servo**: <10ms (GPIO direto)

### **Recursos Computacionais**
- **CPU**: ~15-25% (Raspberry Pi 4B)
- **RAM**: ~200-300MB
- **Network**: Mínimo (local apenas)

---

## 🔒 ASPECTOS DE SEGURANÇA

### **Failsafes Implementados**
1. **Timeout de Comandos**: 1.0s (enhanced_servo_control)
2. **Limites Físicos**: Ângulos máximo/mínimo configuráveis
3. **Emergency Stop**: Parada via botão PS do joystick
4. **GPIO Cleanup**: Liberação automática de recursos

### **Monitoramento**
- **Estado VESC**: Monitoring contínuo via `/sensors/core`
- **Diagnósticos**: DiagnosticArray (enhanced mode)
- **Health Checks**: Heartbeat interno

---

## 📈 EXTENSIBILIDADE

### **Pontos de Extensão Identificados**
1. **Percepção**: Slot preparado para LiDAR
2. **Navegação**: Interface `/drive` padronizada
3. **Sensores**: Framework TF para novos sensores
4. **Controle**: Arquitetura permite controladores adicionais

### **Preparação para LiDAR**
- **Driver**: YDLiDAR suportado
- **TF**: Static transform `base_link → laser_frame`
- **Launch**: Configuração comentada pronta

---

## 💡 CONSIDERAÇÕES TÉCNICAS

### **Pontos Fortes da Arquitetura**
1. **Modularidade**: Componentes independentes
2. **Padrão F1TENTH**: Compatibilidade com simulador
3. **Flexibilidade**: Múltiplas interfaces (Ackermann/Twist)
4. **Robustez**: Failsafes e timeouts implementados

### **Limitações Identificadas**
1. **Hardware Single-Point**: Raspberry Pi único
2. **GPIO Dependency**: pigpio necessário para servo
3. **LiDAR Pendente**: Integração não testada
4. **Real-time**: ROS2 não é real-time OS

### **Recomendações de Otimização**
1. **CPU Isolation**: Reservar cores para controle crítico
2. **Memory Lock**: mlockall() para evitar swap
3. **Priority Scheduling**: SCHED_FIFO para nós críticos
4. **Network**: Configurar DDS para baixa latência

---

*Documento gerado via análise automática do código fonte - 2025-01-20*
