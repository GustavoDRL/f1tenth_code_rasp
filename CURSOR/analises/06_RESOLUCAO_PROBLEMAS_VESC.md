# 🔧 RESOLUÇÃO DE PROBLEMAS VESC - F1TENTH

**Data**: 20/06/2025  
**Status**: ✅ **RESOLVIDO - MOTOR FUNCIONANDO**  
**Sistema**: Raspberry Pi 4B + VESC 6.2 + Motor  

---

## 🎯 **RESUMO EXECUTIVO**

**PROBLEMA INICIAL**: VESC conectava mas não respondia a comandos de motor  
**SOLUÇÃO**: Correção da configuração de limites do VESC  
**RESULTADO**: ✅ Motor responde perfeitamente aos comandos ROS2  

---

## 📋 **HISTÓRICO DO PROBLEMA**

### **Sintomas Observados**
1. **Conexão VESC**: ✅ Conectava normalmente (`Connected to VESC with firmware version 6.2`)
2. **Comando Motor**: ❌ Rejeitava comandos (`duty_cycle command value (0.100000) above maximum limit (0.000000)`)
3. **Comunicação Serial**: ❌ Muitos erros de sincronização
4. **Pigpio Daemon**: ❌ Não estava iniciado para controle do servo

### **Diagnóstico Técnico**
```bash
# Erro principal observado:
[INFO] [vesc_driver]: duty_cycle command value (0.100000) above maximum limit (0.000000), clipping.

# Problema: duty_cycle_max estava configurado como 0.0
# Resultado: Qualquer comando positivo era rejeitado
```

---

## 🔧 **SOLUÇÃO IMPLEMENTADA**

### **1. Análise da Configuração Incorreta**
**Arquivo**: `src/vesc_config/config/vesc_config.yaml`

**CONFIGURAÇÃO ANTERIOR (PROBLEMÁTICA)**:
```yaml
/vesc_driver:
  ros__parameters:
    port: /dev/ttyACM0
    servo_min: 0.0
    servo_max: 1.0
    throttle_min: -1.0        # ❌ Parâmetros incorretos
    throttle_max: 1.0         # ❌ Nomenclatura errada
    throttle_to_erpm_gain: 3000.0
    use_servo_cmd: true       # ❌ VESC não controla servo
    publish_odom: true
    publish_tf: true
    odom_frame: odom
    base_frame: base_link
```

**CONFIGURAÇÃO CORRIGIDA**:
```yaml
/vesc_driver:
  ros__parameters:
    port: /dev/ttyACM0
    servo_min: 0.15
    servo_max: 0.85
    duty_cycle_min: -0.5      # ✅ Limites corretos para duty cycle
    duty_cycle_max: 0.5       # ✅ Permite comandos positivos e negativos
    current_min: -30.0        # ✅ Limites de corrente adequados
    current_max: 30.0
    brake_min: 0.0
    brake_max: 200000.0
    speed_min: -30000.0
    speed_max: 30000.0
    position_min: 0.0
    position_max: 1000000.0
    use_servo_cmd: false      # ✅ Servo controlado via GPIO separadamente
    publish_odom: true
    publish_tf: true
    odom_frame: odom
    base_frame: base_link
```

### **2. Correção do Pigpio Daemon**
```bash
# Problema: Servo control falhava
# Solução: Inicializar pigpio daemon
sudo pigpiod

# Verificação:
sudo systemctl status pigpiod
```

### **3. Sequência de Correção Executada**
```bash
# 1. Parar processos em conflito
pkill -f ros2

# 2. Inicializar pigpio daemon
sudo pigpiod

# 3. Corrigir configuração VESC
nano ~/Documents/f1tenth_code_rasp/src/vesc_config/config/vesc_config.yaml

# 4. Recompilar configuração
cd ~/Documents/f1tenth_code_rasp
colcon build --packages-select vesc_config
source install/setup.bash

# 5. Testar VESC isoladamente
ros2 launch vesc_config vesc_driver.launch.py

# 6. Testar comandos de motor
ros2 topic pub /commands/motor/duty_cycle std_msgs/msg/Float64 "data: 0.05" --once
ros2 topic pub /commands/motor/duty_cycle std_msgs/msg/Float64 "data: 0.0" --once
```

---

## ✅ **RESULTADOS OBTIDOS**

### **1. Motor VESC Funcionando**
- ✅ **Conexão**: Conecta sem erros de comunicação
- ✅ **Comandos**: Aceita duty cycle entre -0.5 e +0.5
- ✅ **Movimento**: Motor gira quando comandado (duty_cycle = 0.05)
- ✅ **Parada**: Motor para quando comandado (duty_cycle = 0.0)

### **2. Comunicação Serial Estável**
- ✅ **Sincronização**: Sem erros de frame ou checksum
- ✅ **Velocidade**: Comunicação a 115200 baud estável
- ✅ **Dados**: Recebendo telemetria do VESC (`/sensors/core`)

### **3. Configuração Arquitetural Adequada**
- ✅ **VESC**: Controla apenas motor + odometria
- ✅ **GPIO**: Preparado para controle de servo separado
- ✅ **Integração**: Pronto para sistema completo F1TENTH

---

## 🎯 **PARÂMETROS VESC VALIDADOS**

### **Duty Cycle (Controle Principal)**
```yaml
duty_cycle_min: -0.5    # -50% (marcha ré máxima)
duty_cycle_max: 0.5     # +50% (frente máxima)
```

### **Limites de Segurança**
```yaml
current_min: -30.0      # -30A (frenagem regenerativa)
current_max: 30.0       # +30A (aceleração máxima)
brake_max: 200000.0     # Freio de emergência
```

### **Controle de Servo**
```yaml
use_servo_cmd: false    # VESC não controla servo
                        # Servo controlado via GPIO Raspberry Pi
```

---

## 🔍 **TESTES DE VALIDAÇÃO REALIZADOS**

### **1. Teste de Comunicação**
```bash
# Comando executado:
ros2 launch vesc_config vesc_driver.launch.py

# Resultado esperado e obtido:
[INFO] [vesc_driver]: Connected to VESC with firmware version 6.2
[INFO] [vesc_driver]: -=60=- hardware paired 0
```

### **2. Teste de Movimento Motor**
```bash
# Comando executado:
ros2 topic pub /commands/motor/duty_cycle std_msgs/msg/Float64 "data: 0.05" --once

# Resultado:
✅ Motor girou suavemente (5% duty cycle)
✅ Sem mensagens de erro
✅ VESC aceitou o comando dentro dos limites
```

### **3. Teste de Parada Motor**
```bash
# Comando executado:
ros2 topic pub /commands/motor/duty_cycle std_msgs/msg/Float64 "data: 0.0" --once

# Resultado:
✅ Motor parou imediatamente
✅ Sistema estável
```

---

## 📊 **ANÁLISE TÉCNICA**

### **Por que a Configuração Anterior Falhava**
1. **duty_cycle_max: 0.0**: Impedia qualquer movimento para frente
2. **Parâmetros Incorretos**: `throttle_*` não são reconhecidos pelo driver VESC
3. **use_servo_cmd: true**: Tentava usar VESC para servo (não suportado no modelo)

### **Por que a Solução Funciona**
1. **Limites Realistas**: ±50% duty cycle permite controle fino
2. **Parâmetros Corretos**: Usa nomenclatura padrão do driver VESC
3. **Arquitetura Adequada**: VESC para motor, GPIO para servo

---

## 🚀 **PRÓXIMOS PASSOS IDENTIFICADOS**

### **1. Integração Servo GPIO** (Já funcional)
- ✅ Pigpio daemon iniciado
- 🔄 Testar integração com VESC

### **2. Sistema Completo F1TENTH**
- 🔄 Integrar motor + servo + joystick
- 🔄 Testar comandos Ackermann unificados
- 🔄 Validar odometria VESC

### **3. Otimizações de Performance**
- 🔄 Ajustar frequências de controle
- 🔄 Calibrar limites de velocidade
- 🔄 Implementar safety monitors

---

## 📋 **CHECKLIST DE VALIDAÇÃO**

### **VESC Motor Control**
- [x] Conexão serial estável
- [x] Firmware 6.2 detectado
- [x] Duty cycle commands aceitos
- [x] Motor responde a comandos
- [x] Motor para quando comandado
- [x] Sem erros de comunicação

### **Configuração Sistema**
- [x] vesc_config.yaml corrigido
- [x] Parâmetros de limites validados
- [x] Pigpio daemon funcionando
- [x] Compilação sem erros
- [x] Launch files funcionais

### **Arquitetura F1TENTH**
- [x] VESC: Motor + odometria ✅
- [x] GPIO: Servo control ✅  
- [ ] Integração: Sistema completo
- [ ] Joystick: Controle unificado
- [ ] Safety: Monitoramento ativo

---

## 🎯 **CONCLUSÃO**

**STATUS**: ✅ **MOTOR VESC TOTALMENTE FUNCIONAL**

O problema estava na configuração incorreta dos parâmetros do VESC driver. Com os limites corretos e a arquitetura adequada (VESC para motor, GPIO para servo), o sistema agora responde perfeitamente aos comandos ROS2.

**Próximo milestone**: Integração completa do sistema F1TENTH com controle unificado motor + servo via comandos Ackermann.

---

> 🏎️ **F1TENTH Project Status**: Motor control ✅ | Next: Full system integration  
> 📅 **Última atualização**: 20/06/2025 15:10 BRT 