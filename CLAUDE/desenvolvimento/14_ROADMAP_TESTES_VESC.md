# 🏎️ ROADMAP TESTES VESC - F1TENTH

**Data de Criação**: 20 de Janeiro de 2025  
**Status**: Diagnosticando Segmentation Fault  
**Hardware**: Raspberry Pi 4B + VESC Motor Controller  

## 🚨 **PROBLEMA ATUAL**

**Segmentation Fault** ao executar o driver VESC:
```bash
ros2 run vesc_driver vesc_driver_node --ros-args --params-file src/vesc_config/config/vesc_config.yaml
[ros2run]: Segmentation fault
```

## 🔍 **DIAGNÓSTICO ESTRUTURADO**

### **Fase 1: Verificação de Hardware e Conectividade**

#### 🔌 **1.1 Verificar Hardware VESC**
- [ ] VESC fisicamente conectado via USB
- [ ] LED de status do VESC ativo
- [ ] Cabo USB funcional
- [ ] Alimentação do VESC adequada

#### 🖥️ **1.2 Verificar Porta Serial**
- [ ] Dispositivo detectado pelo sistema
- [ ] Permissões de acesso à porta
- [ ] Porta correta no arquivo de configuração

### **Fase 2: Configuração e Dependências**

#### ⚙️ **2.1 Validar Configuração VESC**
- [ ] Arquivo de configuração sintaxe correta
- [ ] Parâmetros dentro dos limites válidos
- [ ] Namespace ROS2 adequado

#### 📦 **2.2 Verificar Dependências**
- [ ] Pacotes VESC corretamente compilados
- [ ] Bibliotecas de dependência instaladas
- [ ] Compatibilidade ROS2 Humble

### **Fase 3: Testes Progressivos**

#### 🧪 **3.1 Testes Básicos**
- [ ] Teste sem hardware (modo simulação)
- [ ] Teste com configuração mínima
- [ ] Teste com hardware conectado

#### 🔧 **3.2 Testes de Integração**
- [ ] Comunicação serial funcionando
- [ ] Publicação de tópicos ROS2
- [ ] Recepção de comandos

#### 🏁 **3.3 Testes de Funcionalidade**
- [ ] Controle de motor básico
- [ ] Leitura de telemetria
- [ ] Integração com ackermann

## 🛠️ **COMANDOS DE DIAGNÓSTICO**

### **Etapa 1: Verificação de Hardware**

```bash
# 1. Verificar dispositivos USB conectados
lsusb | grep -i vesc
dmesg | tail -20

# 2. Verificar portas seriais disponíveis
ls -la /dev/ttyACM* /dev/ttyUSB*
sudo dmesg | grep -i usb

# 3. Verificar permissões do usuário
groups $USER
id $USER
```

### **Etapa 2: Teste de Conectividade Serial**

```bash
# 4. Instalar ferramentas de teste serial
sudo apt update
sudo apt install minicom screen

# 5. Testar comunicação serial básica
sudo minicom -D /dev/ttyACM0 -b 115200
# Pressione Ctrl+A, depois Z, depois Q para sair

# 6. Verificar se porta está em uso
sudo lsof /dev/ttyACM0
```

### **Etapa 3: Configuração ROS2**

```bash
# 7. Verificar instalação dos pacotes VESC
ros2 pkg list | grep vesc
ros2 interface list | grep vesc

# 8. Teste com configuração padrão
ros2 run vesc_driver vesc_driver_node --ros-args --params-file src/vesc-humble/vesc_driver/params/vesc_config.yaml

# 9. Teste sem arquivo de configuração
ros2 run vesc_driver vesc_driver_node
```

### **Etapa 4: Debug Avançado**

```bash
# 10. Executar com debug detalhado
ros2 run vesc_driver vesc_driver_node --ros-args --log-level debug

# 11. Verificar logs do sistema
journalctl -f | grep vesc

# 12. Testar com valgrind (se instalado)
sudo apt install valgrind
valgrind ros2 run vesc_driver vesc_driver_node
```

## 📊 **CONFIGURAÇÕES ALTERNATIVAS**

### **Config Segura (Teste Básico)**
```yaml
/**:
  ros__parameters:
    port: "/dev/ttyACM0"
    servo_min: 0.15
    servo_max: 0.85
    speed_min: -1000.0
    speed_max: 1000.0
    current_min: 0.0
    current_max: 10.0
```

### **Config Mínima (Diagnóstico)**
```yaml
/**:
  ros__parameters:
    port: "/dev/ttyACM0"
```

## 🎯 **CRITÉRIOS DE SUCESSO**

### **Sucesso Básico**
- [ ] Driver VESC inicia sem segmentation fault
- [ ] Conexão serial estabelecida
- [ ] Tópicos ROS2 publicados

### **Sucesso Intermediário**
- [ ] Telemetria do VESC recebida
- [ ] Comandos básicos aceitos
- [ ] Odometria publicada

### **Sucesso Completo**
- [ ] Controle total do motor
- [ ] Integração com ackermann
- [ ] Sistema estável em operação

## 🚨 **SAFETY FIRST**

### **Precauções Obrigatórias**
- [ ] Motor desconectado durante testes iniciais
- [ ] Valores de corrente limitados
- [ ] Emergency stop sempre disponível
- [ ] Testes em ambiente controlado

### **Limits de Segurança**
```yaml
current_max: 10.0    # Máximo 10A durante testes
speed_max: 1000.0    # Velocidade reduzida
servo_min: 0.15      # Limites conservadores
servo_max: 0.85      # Limites conservadores
```

## 📋 **PRÓXIMOS PASSOS**

1. **Imediato**: Executar diagnóstico de hardware
2. **Curto Prazo**: Resolver segmentation fault
3. **Médio Prazo**: Validar comunicação VESC
4. **Longo Prazo**: Integração completa com sistema F1TENTH

---

> 🏎️ **Meta**: VESC operacional e integrado ao sistema F1TENTH  
> ⚡ **Performance**: Comunicação confiável <50ms  
> 🛡️ **Safety**: Limites de segurança sempre ativos 