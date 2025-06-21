# 🚀 F1TENTH Launch Files - Sistema Organizado

Esta pasta contém os launch files organizados e simplificados do sistema F1TENTH, cada um projetado para diferentes cenários de teste e desenvolvimento.

## 📋 Arquivos Disponíveis

### 1. `f1tenth_complete_system.launch.py` - **Sistema Completo**
**Componentes:** VESC + Servo + LiDAR + Teclado  
**Uso:** Sistema completo para operação autônoma e manual

```bash
ros2 launch f1tenth_control f1tenth_complete_system.launch.py
```

**Inclui:**
- ✅ VESC Motor Controller (USB)
- ✅ Servo Steering (GPIO PWM)  
- ✅ YDLiDAR X4 (USB)
- ✅ Keyboard Control Interface
- ✅ Real-time Odometry & TF
- ✅ Ackermann ↔ VESC Conversion

---

### 2. `f1tenth_system_no_lidar.launch.py` - **Sistema Híbrido (Sem LiDAR)**
**Componentes:** VESC + Servo + Teclado  
**Uso:** Sistema principal para desenvolvimento e testes manuais

```bash
ros2 launch f1tenth_control f1tenth_system_no_lidar.launch.py
```

**Inclui:**
- ✅ VESC Motor Controller (USB)
- ✅ Servo Steering (GPIO PWM)
- ❌ LiDAR (desabilitado)
- ✅ Keyboard Control Interface
- ✅ Real-time Odometry & TF
- ✅ Ackermann ↔ VESC Conversion

---

### 3. `f1tenth_vesc_only.launch.py` - **Motor Apenas**
**Componentes:** VESC + Teclado  
**Uso:** Testes isolados do motor e calibração VESC

```bash
ros2 launch f1tenth_control f1tenth_vesc_only.launch.py
```

**Inclui:**
- ✅ VESC Motor Controller (USB)
- ❌ Servo (desabilitado)
- ❌ LiDAR (desabilitado)
- ✅ Keyboard Control Interface (W/S apenas)
- ✅ Basic Odometry
- ✅ Ackermann → VESC Conversion

**Controles:** `W/S` = Acelerar/Frear | `A/D` = Sem efeito

---

### 4. `f1tenth_servo_only.launch.py` - **Servo Apenas**
**Componentes:** Servo + Teclado  
**Uso:** Testes de direção e calibração do servo

```bash
ros2 launch f1tenth_control f1tenth_servo_only.launch.py
```

**Inclui:**
- ❌ VESC (desabilitado)
- ✅ Servo Steering (GPIO PWM)
- ❌ LiDAR (desabilitado)
- ✅ Keyboard Control Interface (A/D apenas)
- ✅ Static TF Publishers

**Controles:** `A/D` = Esquerda/Direita | `W/S` = Sem efeito

---

## 🎮 Controles de Teclado Unificados

Todos os launch files incluem interface de teclado:

| Tecla | Função | Disponível Em |
|-------|--------|---------------|
| **W** | Acelerar | Complete, No LiDAR, VESC Only |
| **S** | Frear/Ré | Complete, No LiDAR, VESC Only |
| **A** | Virar Esquerda | Complete, No LiDAR, Servo Only |
| **D** | Virar Direita | Complete, No LiDAR, Servo Only |
| **Espaço** | Parar/Centralizar | Todos |
| **Q** | Sair | Todos |

## 🔧 Configurações

### Arquivos de Configuração Utilizados:
- **Control Config:** `config/system_config.yaml`
- **VESC Config:** `vesc_config/config/vesc_config.yaml`  
- **LiDAR Config:** `ydlidar_ros2_driver/params/X4.yaml`

### Parâmetros Importantes:
- **Namespace:** `ego_racecar` (padrão F1TENTH)
- **Max Speed:** 2.0-3.0 m/s (configurável)
- **Max Steering:** ±0.4 rad (±23°)
- **Control Frequency:** 50-100Hz
- **Safety Timeout:** 1.0s

## 🚨 Sequência de Inicialização

Todos os launch files seguem sequência otimizada:

1. **t=0s:** Hardware drivers (VESC, Servo, LiDAR)
2. **t=3s:** Conversion nodes (Ackermann ↔ VESC)  
3. **t=5s:** Control interface (Keyboard)
4. **Continuous:** Transform publishers

## 📊 Monitoramento

### Verificar Sistema Ativo:
```bash
# Listar nós
ros2 node list

# Verificar tópicos
ros2 topic list

# Monitorar odometria
ros2 topic echo /ego_racecar/odom

# Monitorar comandos
ros2 topic echo /drive

# Verificar LiDAR (se ativo)
ros2 topic echo /scan
```

### Diagnóstico de Frequência:
```bash
ros2 topic hz /scan          # LiDAR: ~10Hz
ros2 topic hz /ego_racecar/odom  # Odometry: ~50Hz
ros2 topic hz /drive         # Commands: variável
```

## ⚠️ Segurança

- **Emergency Stop:** Todos os sistemas incluem `Espaço` para parada
- **Timeouts:** Monitoramento de comunicação (1s)
- **Speed Limits:** Limitação de velocidade configurável
- **Respawn:** Restart automático de nós críticos

## 🛠️ Desenvolvimento

### Adicionar Novo Componente:
1. Adicionar hardware driver em `hardware_drivers`
2. Configurar conversion nodes se necessário
3. Atualizar transform publishers
4. Testar com keyboard control

### Debug Mode:
```bash
ros2 launch f1tenth_control [launch_file] debug_mode:=true
```

---

## 📦 Dependências

- **ROS2 Humble**
- **vesc_driver** (VESC communication)
- **vesc_ackermann** (Ackermann conversion)
- **joy_converter** (Keyboard interface)
- **f1tenth_control** (Servo control)
- **ydlidar_ros2_driver** (LiDAR - apenas sistema completo)

---

> 🏎️ **F1TENTH Racing Platform**  
> ⚡ **Real-time Performance:** <20ms control latency  
> 🛡️ **Safety First:** Emergency stop sempre ativo  
> 🎯 **Competition Ready:** Padrão F1TENTH compliant 