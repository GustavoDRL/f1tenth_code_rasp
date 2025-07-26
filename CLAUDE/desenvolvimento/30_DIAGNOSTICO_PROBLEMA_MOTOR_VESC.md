# 🔍 DIAGNÓSTICO: MOTOR VESC NÃO GIRA AS RODAS

**Data**: 2025-06-21  
**Status**: ✅ PROBLEMA IDENTIFICADO - CORREÇÃO EM ANDAMENTO  
**Sistema**: Raspberry Pi 4B + VESC 6.2  

## 🚨 **SINTOMAS OBSERVADOS**

### ✅ **FUNCIONANDO CORRETAMENTE**
- VESC conectado e comunicando
- Servo motor respondendo perfeitamente
- Sistema ROS2 operacional
- Conversores ackermann_to_vesc e vesc_to_odom rodando
- Controle de teclado enviando comandos
- Tópicos `/drive` e `/commands/motor/duty_cycle` existem

### ❌ **PROBLEMA IDENTIFICADO**
- **Motor VESC não gira as rodas**
- Comandos de velocidade não produzem movimento
- Sistema hibrido funcional apenas para servo

---

## 🔧 **ANÁLISE TÉCNICA**

### **1. CONFIGURAÇÕES CONFLITANTES ENCONTRADAS**

#### **Arquivo Atual**: `src/vesc_config/config/vesc_config.yaml`
```yaml
/vesc_driver:
  ros__parameters:
    duty_cycle_min: -0.5    # ✅ Permite movimento
    duty_cycle_max: 0.5     # ✅ Permite movimento
```

#### **Arquivo Padrão**: `src/vesc-humble/vesc_driver/params/vesc_config.yaml`
```yaml
/**:
  ros__parameters:
    duty_cycle_max: 0.0     # ❌ IMPEDE MOVIMENTO!
    duty_cycle_min: 0.0     # ❌ IMPEDE MOVIMENTO!
```

### **2. FLUXO DE COMUNICAÇÃO VERIFICADO**
```
Teclado → joy_keyboard (✅)
       ↓
    /drive topic (✅)
       ↓
ackermann_to_vesc (✅)
       ↓
/commands/motor/duty_cycle (✅)
       ↓
   vesc_driver (❓ PROBLEMA AQUI)
       ↓
   Hardware VESC (❌ NÃO RECEBE)
```

---

## 🧪 **COMANDOS DE DIAGNÓSTICO**

### **FASE 1: Verificação de Comunicação**
```bash
# 1. Verificar se comandos chegam ao VESC
ros2 topic echo /commands/motor/duty_cycle

# 2. Monitorar estado do VESC
ros2 topic echo /sensors/core

# 3. Verificar parâmetros do driver
ros2 param list /vesc_driver
ros2 param get /vesc_driver duty_cycle_max
ros2 param get /vesc_driver duty_cycle_min
```

### **FASE 2: Teste Direto do Motor**
```bash
# Teste com duty cycle baixo (5%)
ros2 topic pub /commands/motor/duty_cycle std_msgs/msg/Float64 "data: 0.05" --once

# Se não funcionar, testar com speed
ros2 topic pub /commands/motor/speed std_msgs/msg/Float64 "data: 1000" --once

# Teste de current
ros2 topic pub /commands/motor/current std_msgs/msg/Float64 "data: 5.0" --once
```

### **FASE 3: Verificação de Hardware**
```bash
# Verificar conexão física
ls -la /dev/ttyACM*
dmesg | grep -i vesc
dmesg | grep -i usb

# Verificar se VESC está recebendo dados
ros2 topic hz /sensors/core
```

---

## 💡 **HIPÓTESES DO PROBLEMA**

### **1. PARÂMETROS RESTRITIVOS** (Mais Provável)
- Driver VESC pode ter parâmetros que limitam duty_cycle
- Configuração de segurança muito restritiva

### **2. PROBLEMA DE ESCALA** (Possível)
- Comandos ackermann podem não estar sendo convertidos corretamente
- Valores muito baixos para produzir movimento

### **3. CONFIGURAÇÃO DO FIRMWARE VESC** (Menos Provável)
- VESC configurado para não aceitar comandos duty_cycle
- Configuração interna do VESC bloqueando movimento

---

## 🔧 **PLANO DE CORREÇÃO**

### **ETAPA 1: Diagnóstico Imediato**
1. Executar comandos de verificação
2. Testar comando direto ao motor
3. Verificar logs do driver VESC

### **ETAPA 2: Correção de Configuração**
1. Ajustar parâmetros do VESC
2. Testar diferentes tipos de comando (duty_cycle vs speed)
3. Verificar limitadores de segurança

### **ETAPA 3: Validação**
1. Teste motor isolado
2. Teste integração com sistema completo
3. Validação de segurança

---

## 📋 **PRÓXIMOS PASSOS**

1. **AGORA**: Executar diagnóstico de comunicação
2. **SEGUINTE**: Testar comando direto ao motor
3. **FINAL**: Ajustar configurações conforme necessário

---

> 🎯 **OBJETIVO**: Motor VESC girando as rodas com controle de teclado funcional
> ⚡ **FOCO**: Diagnóstico sistemático antes de modificar configurações 