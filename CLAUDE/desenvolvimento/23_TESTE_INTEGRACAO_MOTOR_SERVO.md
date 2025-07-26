# 🎯 TESTE INTEGRAÇÃO MOTOR + SERVO - F1TENTH

**Data**: 20/06/2025  
**Status**: 🚀 **PRONTO PARA TESTE COMPLETO**  
**Componentes**: Motor VESC ✅ + Servo GPIO ✅  

---

## 🎯 **OBJETIVO DO TESTE**

Agora que temos:
- ✅ **Motor VESC**: Funcionando perfeitamente (gira e para via duty_cycle)
- ✅ **Servo GPIO**: Funcionando perfeitamente (gira esquerda/direita)

**PRÓXIMO MARCO**: Testar ambos os componentes trabalhando juntos em um sistema integrado F1TENTH.

---

## 🧪 **PLANO DE TESTE INTEGRADO**

### **1. Teste Sistema Completo via Launch**
Vamos testar o sistema completo com motor + servo rodando simultaneamente via arquivo de launch.

### **2. Teste Comandos Ackermann Unificados**
Usar comandos Ackermann que controlam velocidade (motor) e ângulo (servo) de forma unificada.

### **3. Validação Coordenação**
Verificar se motor e servo respondem simultaneamente sem conflitos.

---

## 🔧 **COMANDOS PARA EXECUÇÃO NO RASPBERRY PI**

Execute os seguintes comandos em ordem:

### **1. Teste do Sistema Completo**
```bash
# Ir para o workspace
cd ~/Documents/f1tenth_code_rasp
source install/setup.bash

# Verificar se pigpio está rodando (necessário para servo)
sudo systemctl status pigpiod
# Se não estiver rodando:
sudo pigpiod

# Lançar sistema completo (motor + servo + conversores)
ros2 launch f1tenth_control f1tenth_control.launch.py
```

### **2. Em Outro Terminal - Verificar Tópicos Ativos**
```bash
# Sourcing
cd ~/Documents/f1tenth_code_rasp
source install/setup.bash

# Ver todos os tópicos disponíveis
ros2 topic list

# Verificar especificamente os principais:
ros2 topic info /drive
ros2 topic info /commands/motor/duty_cycle  
ros2 topic info /commands/servo/position
```

### **3. Teste Comandos Individuais (Verificação)**
```bash
# Teste servo (deve funcionar como antes)
ros2 topic pub /commands/servo/position std_msgs/msg/Float64 "data: 0.15" --once
ros2 topic pub /commands/servo/position std_msgs/msg/Float64 "data: 0.085" --once

# Teste motor (deve funcionar como antes)
ros2 topic pub /commands/motor/duty_cycle std_msgs/msg/Float64 "data: 0.05" --once  
ros2 topic pub /commands/motor/duty_cycle std_msgs/msg/Float64 "data: 0.0" --once
```

### **4. TESTE PRINCIPAL - Comandos Ackermann Unificados**
```bash
# ⚠️ ATENÇÃO: Manter o carro elevado para este teste!

# Teste 1: Movimento reto lento
ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'base_link'
drive:
  steering_angle: 0.0
  speed: 0.5
" --once

# Aguardar 2 segundos, depois parar
ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'base_link'
drive:
  steering_angle: 0.0
  speed: 0.0
" --once
```

### **5. Teste Curva (Motor + Servo Simultaneamente)**
```bash
# ⚠️ ATENÇÃO: Carro elevado!

# Teste 2: Curva esquerda com movimento
ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'base_link'
drive:
  steering_angle: 0.3
  speed: 0.5
" --once

# Aguardar 2 segundos, depois parar e centralizar
ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'base_link'
drive:
  steering_angle: 0.0
  speed: 0.0
" --once
```

---

## 📋 **CHECKLIST DE VALIDAÇÃO**

### **Sistema Launch**
- [ ] Sistema inicia sem erros
- [ ] Todos os nós ficam ativos
- [ ] Tópicos `/drive`, `/commands/motor/*`, `/commands/servo/*` disponíveis
- [ ] Sem mensagens de erro nos logs

### **Testes Individuais (Confirmação)**
- [ ] Servo responde normalmente
- [ ] Motor responde normalmente  
- [ ] Ambos funcionam independentemente

### **Teste Ackermann Unificado**
- [ ] Comando steering_angle move servo
- [ ] Comando speed move motor
- [ ] Ambos respondem simultaneamente
- [ ] Sistema para quando comandado
- [ ] Sem conflitos entre componentes

### **Performance e Estabilidade**
- [ ] Resposta em tempo real (<100ms)
- [ ] Sistema estável durante operação
- [ ] CPU e memória em níveis normais
- [ ] Comunicação ROS2 fluida

---

## 🎯 **RESULTADOS ESPERADOS**

### **✅ Sucesso Total**
- Motor gira quando `speed > 0`
- Motor para quando `speed = 0`
- Servo gira quando `steering_angle != 0`
- Servo centraliza quando `steering_angle = 0`
- Ambos respondem simultaneamente ao comando Ackermann
- Sistema estável e sem erros

### **❌ Problemas Possíveis**
1. **Conflito de recursos**: GPIO vs USB serial
2. **Timing**: Comandos não sincronizados
3. **Conversão**: Ackermann não convertendo corretamente
4. **Performance**: Sistema sobrecarregado

---

## 🚀 **PRÓXIMOS PASSOS APÓS SUCESSO**

### **1. Integração Joystick**
- Testar controle manual via joystick
- Validar interface Ackermann completa

### **2. Otimização**
- Ajustar parâmetros de resposta
- Calibrar limites de velocidade e ângulo
- Otimizar frequências de controle

### **3. Sistema Autônomo Básico**
- Comandos programáticos simples
- Sequências de movimento pré-definidas
- Preparação para algoritmos de navegação

---

## 📊 **DOCUMENTAÇÃO DOS RESULTADOS**

Por favor, documente aqui os resultados:

### **Teste Sistema Launch**
```
[ ] Sucesso / [ ] Falha
Observações: ___________________
```

### **Teste Ackermann Reto**
```
[ ] Sucesso / [ ] Falha
Motor girou: [ ] Sim / [ ] Não
Servo manteve centro: [ ] Sim / [ ] Não
```

### **Teste Ackermann Curva**
```
[ ] Sucesso / [ ] Falha  
Motor girou: [ ] Sim / [ ] Não
Servo girou: [ ] Sim / [ ] Não
Coordenação: [ ] Perfeita / [ ] Problemas
```

---

## 🎯 **CONCLUSÃO**

Este teste representa o **marco final da Fase 1** do projeto F1TENTH. Com motor e servo funcionando de forma coordenada via comandos Ackermann, teremos um **carro F1TENTH básico completamente funcional**.

**Próximo grande marco**: Integração LiDAR para navegação autônoma.

---

> 🏎️ **F1TENTH Project**: Motor ✅ + Servo ✅ = **Sistema Básico Completo!**  
> 📅 **Teste**: 20/06/2025 | **Meta**: Sistema integrado funcionando  
> 🚀 **Próximo**: LiDAR + Navegação Autônoma 