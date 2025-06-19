# 🔗 REGRAS OBRIGATÓRIAS - WORKFLOW SSH RASPBERRY PI F1TENTH

**Categoria**: Workflow de Desenvolvimento  
**Prioridade**: CRÍTICA - Aplicação Obrigatória  
**Hardware Target**: Raspberry Pi 4B via SSH  
**Ambiente de Desenvolvimento**: WSL + SSH Remote

---

## 🎯 **REGRA FUNDAMENTAL - DUAL ENVIRONMENT WORKFLOW**

### ✅ **AMBIENTE OBRIGATÓRIO DE DESENVOLVIMENTO**
```
🖥️ WSL (Windows Subsystem for Linux)
├── 📝 Desenvolvimento de código
├── 🧪 Testes unitários e mockados
├── 📚 Documentação e análise
├── 🔄 Controle de versão Git
└── 🚀 Push para sincronização

     ⬇️ SSH Connection ⬇️

🏎️ RASPBERRY PI 4B (192.168.0.123)
├── 🔄 Pull das alterações Git
├── 🏗️ Build real dos pacotes ROS2
├── 🔧 Execução da aplicação F1TENTH
├── 🧪 Testes hardware-in-loop
└── 📊 Validação de performance real
```

---

## 🔐 **CONFIGURAÇÃO SSH OBRIGATÓRIA**

### 📋 **Credenciais de Conexão**
- **Host**: `192.168.0.123`
- **Usuário**: `disney`
- **Senha**: `qgnidakq`
- **Comando SSH**: `ssh disney@192.168.0.123`

### ⚙️ **Configuração Automática SSH**
```bash
# No WSL - Configurar SSH sem senha (recomendado)
ssh-keygen -t rsa -b 4096 -C "f1tenth-development"
ssh-copy-id disney@192.168.0.123

# Testar conexão
ssh disney@192.168.0.123 "echo 'SSH funcionando corretamente'"

# Configurar SSH config (opcional)
cat >> ~/.ssh/config << EOF
Host f1tenth-pi
    HostName 192.168.0.123
    User disney
    IdentityFile ~/.ssh/id_rsa
EOF
```

---

## 🔄 **WORKFLOW OBRIGATÓRIO DE DESENVOLVIMENTO**

### 🚀 **FASE 1: DESENVOLVIMENTO NO WSL**
```bash
# 1. Desenvolvimento local no WSL
cd ~/Documents/f1tenth_code_rasp/

# 2. Edição de código e documentação
# - Usar Cursor/VSCode no WSL
# - Implementar funcionalidades
# - Escrever testes unitários

# 3. Testes locais básicos (sem hardware)
python3 -m pytest tests/unit/
python3 -m pytest tests/mock/

# 4. Commit das alterações
git add .
git commit -m "feat(scope): implementação [descrição] [F1TENTH:categoria]"
git push origin main
```

### 🏎️ **FASE 2: EXECUÇÃO NO RASPBERRY PI**
```bash
# 1. Conectar via SSH
ssh disney@192.168.0.123

# 2. Sincronizar código
cd ~/Documents/f1tenth_code_rasp/
git pull origin main

# 3. Build dos pacotes ROS2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 4. Executar aplicação real
source install/setup.bash
ros2 launch f1tenth_bringup f1tenth_system.launch.py

# 5. Testes hardware-in-loop
ros2 run f1tenth_control test_servo_control
ros2 topic hz /scan  # Verificar LiDAR
ros2 topic hz /ego_racecar/odom  # Verificar odometria
```

### 📊 **FASE 3: VALIDAÇÃO E DOCUMENTAÇÃO**
```bash
# No Raspberry Pi - Coleta de métricas
ros2 run rqt_graph rqt_graph
ros2 topic bw /drive  # Bandwidth
top -p $(pgrep -f f1tenth)  # CPU usage

# Voltar ao WSL para documentação
exit  # Sair do SSH
cd ~/Documents/f1tenth_code_rasp/
# Documentar resultados em CURSOR/
git add . && git commit -m "docs: resultados teste raspberry pi"
git push origin main
```

---

## ⚡ **COMANDOS ESSENCIAIS SSH**

### 🔗 **Conexão e Navegação**
```bash
# Conexão básica
ssh disney@192.168.0.123

# Execução de comando remoto (sem entrar no SSH)
ssh disney@192.168.0.123 "cd ~/Documents/f1tenth_code_rasp && git pull"

# Transferência de arquivos (se necessário)
scp arquivo.py disney@192.168.0.123:~/Documents/f1tenth_code_rasp/src/

# Túnel SSH para interfaces gráficas (se necessário)
ssh -X disney@192.168.0.123
```

### 🔄 **Sincronização Git Automática**
```bash
# Script para sincronização rápida (criar no WSL)
#!/bin/bash
# sync_to_pi.sh
set -e

echo "🔄 Sincronizando código para Raspberry Pi..."

# Commit local
git add .
git commit -m "sync: alterações para teste no raspberry pi" || true
git push origin main

# Sincronizar no Raspberry Pi
ssh disney@192.168.0.123 "cd ~/Documents/f1tenth_code_rasp && git pull origin main"

echo "✅ Sincronização concluída!"
```

---

## 🧪 **ESTRATÉGIA DE TESTES DUAL**

### 🖥️ **TESTES NO WSL (Desenvolvimento)**
- ✅ **Testes Unitários**: Lógica de algoritmos
- ✅ **Testes Mockados**: Simulação de hardware
- ✅ **Validação de Código**: Syntax, linting, type checking
- ✅ **Performance Simulada**: Algoritmos computacionais
- ✅ **Documentação**: Geração de docs e análises

### 🏎️ **TESTES NO RASPBERRY PI (Real)**
- ✅ **Hardware-in-Loop**: VESC, servo, LiDAR real
- ✅ **Performance Real**: Timing, CPU, memória
- ✅ **Integração ROS2**: Comunicação entre nós
- ✅ **Safety Systems**: Emergency stop, timeouts
- ✅ **End-to-End**: Sistema completo funcionando

---

## 📋 **CHECKLIST OBRIGATÓRIO ANTES DO COMMIT**

### ✅ **No WSL - Desenvolvimento**
- [ ] Código compila sem erros
- [ ] Testes unitários passando
- [ ] Documentação atualizada
- [ ] Commit message seguindo padrão F1TENTH
- [ ] Push para repositório remoto

### ✅ **No Raspberry Pi - Validação**
- [ ] SSH conectado com sucesso
- [ ] Git pull executado
- [ ] Colcon build bem-sucedido
- [ ] ROS2 launch funcional
- [ ] Hardware respondendo corretamente
- [ ] Performance dentro dos limites
- [ ] Nenhum erro crítico nos logs

---

## 🚨 **REGRAS DE SEGURANÇA SSH**

### ⚠️ **PRECAUÇÕES OBRIGATÓRIAS**
1. **Nunca deixar servo em movimento** ao desconectar SSH
2. **Sempre executar emergency stop** antes de sair
3. **Verificar status do hardware** antes de testes
4. **Monitorar CPU/memória** durante execução prolongada
5. **Backup do código** antes de mudanças críticas

### 🛡️ **Comandos de Emergência**
```bash
# Emergency stop via SSH
ssh disney@192.168.0.123 "ros2 service call /emergency_stop std_srvs/srv/Trigger"

# Kill todos os processos ROS2
ssh disney@192.168.0.123 "pkill -f ros2"

# Verificar status do sistema
ssh disney@192.168.0.123 "systemctl status"
```

---

## 📊 **MONITORAMENTO DE PERFORMANCE**

### 🔍 **Métricas Essenciais (No Raspberry Pi)**
```bash
# CPU e memória
htop

# Frequência dos tópicos ROS2
ros2 topic hz /scan
ros2 topic hz /ego_racecar/odom
ros2 topic hz /drive

# Bandwidth dos tópicos
ros2 topic bw /scan

# Latência de comunicação
ros2 topic echo /drive --once

# Status dos nós
ros2 node list
ros2 node info /servo_control_node
```

### 📈 **Limites de Performance**
- **Control Loop**: <20ms (50Hz)
- **CPU Usage**: <80% sustentado
- **Memory Usage**: <1.5GB total
- **SSH Latency**: <10ms para comandos
- **Git Sync**: <30s para sincronização completa

---

## 🔄 **ROTINA DIÁRIA DE DESENVOLVIMENTO**

### 🌅 **Início do Dia**
```bash
# 1. No WSL
cd ~/Documents/f1tenth_code_rasp/
git pull origin main

# 2. Verificar conexão com Raspberry Pi
ssh disney@192.168.0.123 "echo 'Raspberry Pi Online'"

# 3. Sincronizar se necessário
ssh disney@192.168.0.123 "cd ~/Documents/f1tenth_code_rasp && git pull"
```

### 🌆 **Final do Dia**
```bash
# 1. Commit final no WSL
git add .
git commit -m "feat: trabalho do dia [data]"
git push origin main

# 2. Shutdown seguro do Raspberry Pi
ssh disney@192.168.0.123 "sudo shutdown -h now"
```

---

## 📝 **TEMPLATES DE COMANDOS**

### 🔧 **Build e Test Remoto**
```bash
# Template completo de build remoto
ssh disney@192.168.0.123 << 'EOF'
cd ~/Documents/f1tenth_code_rasp
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch f1tenth_bringup f1tenth_system.launch.py &
sleep 5
ros2 topic hz /scan --window 10
pkill -f ros2
EOF
```

### 📊 **Coleta de Logs Remota**
```bash
# Coletar logs do Raspberry Pi
ssh disney@192.168.0.123 "cd ~/Documents/f1tenth_code_rasp && tar -czf logs_$(date +%Y%m%d_%H%M%S).tar.gz log/"
scp disney@192.168.0.123:~/Documents/f1tenth_code_rasp/logs_*.tar.gz ./
```

---

## ✅ **VALIDAÇÃO DE CONFORMIDADE**

Antes de cada sessão de desenvolvimento, verificar:
- [ ] SSH funciona sem problemas
- [ ] Git está sincronizado em ambos os ambientes
- [ ] ROS2 Humble está ativo no Raspberry Pi
- [ ] Hardware F1TENTH está conectado
- [ ] Workspace pode ser compilado com sucesso
- [ ] Launch files executam sem erros críticos

---

> 🔗 **SSH Connection**: Gateway obrigatório para F1TENTH real testing  
> 🏎️ **Raspberry Pi**: Ambiente de execução e validação definitiva  
> 🔄 **Git Sync**: Sincronização automática entre WSL e Raspberry Pi  
> ⚡ **Real-time**: Validação de performance apenas no hardware real 