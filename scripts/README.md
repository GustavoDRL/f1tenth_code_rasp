# 🏎️ Scripts de Automatização F1TENTH

Este diretório contém scripts para automatizar completamente o sistema F1TENTH no Raspberry Pi.

## 📁 Arquivos

| Script | Função | Uso |
|--------|--------|-----|
| `build_f1tenth.sh` | Build completo com correções | `bash scripts/build_f1tenth.sh` |
| `post_build_setup.sh` | Configuração pós-build | Automático após build |
| `f1tenth_startup.sh` | Inicialização do sistema | Automático no boot |
| `install_service.sh` | Instalar serviço systemd | `sudo bash scripts/install_service.sh` |
| `f1tenth.service` | Configuração systemd | Usado pelo install_service.sh |

## 🚀 Instalação Completa (Uma Vez)

### **Passo 1: Build Inicial**
```bash
cd ~/Documents/f1tenth_code_rasp
bash scripts/build_f1tenth.sh
```

### **Passo 2: Instalar Serviço Automático**
```bash
sudo bash scripts/install_service.sh
```

### **Passo 3: Testar Sistema**
```bash
sudo systemctl start f1tenth.service
sudo systemctl status f1tenth.service
```

## ✅ Resultado da Automação

Após a instalação, o sistema:

- ✅ **Inicia automaticamente** no boot do Raspberry Pi
- ✅ **Configura pigpiod** automaticamente
- ✅ **Corrige links simbólicos ROS2** automaticamente
- ✅ **Carrega workspace** automaticamente
- ✅ **Monitora serviços** com restart automático

## 📋 Comandos de Gerenciamento

### **Status do Sistema**
```bash
# Status geral
sudo systemctl status f1tenth.service

# Logs em tempo real
sudo journalctl -u f1tenth.service -f

# Status pigpiod
sudo systemctl status pigpiod
```

### **Controle Manual**
```bash
# Parar sistema
sudo systemctl stop f1tenth.service

# Iniciar sistema
sudo systemctl start f1tenth.service

# Reiniciar sistema
sudo systemctl restart f1tenth.service

# Desabilitar inicialização automática
sudo systemctl disable f1tenth.service
```

### **Teste Manual do Sistema**
```bash
# Source ambiente
source ~/Documents/f1tenth_code_rasp/install/setup.bash

# Testar nó básico
ros2 launch f1tenth_control f1tenth_control.launch.py

# Testar nó avançado
ros2 launch f1tenth_control f1tenth_control.launch.py use_enhanced_control:=true

# Testar comando do servo
ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {steering_angle: 0.2, speed: 0.0}}" --once
```

## 🔧 Rebuilds e Atualizações

### **Rebuild Completo**
```bash
cd ~/Documents/f1tenth_code_rasp
bash scripts/build_f1tenth.sh
```

### **Atualização do Código**
```bash
cd ~/Documents/f1tenth_code_rasp
git pull origin main
bash scripts/build_f1tenth.sh
sudo systemctl restart f1tenth.service
```

## 📊 Monitoramento

### **Logs do Sistema**
```bash
# Logs de inicialização
sudo tail -f /var/log/f1tenth_startup.log

# Logs do serviço systemd
sudo journalctl -u f1tenth.service --since "1 hour ago"

# Status ROS2
ros2 node list
ros2 topic list
```

### **Performance**
```bash
# CPU e memória
top -p $(pgrep -f ros2)

# Frequência dos tópicos
ros2 topic hz /drive
ros2 topic hz /ego_racecar/odom
```

## ❓ Troubleshooting

### **Serviço não inicia**
```bash
# Verificar logs
sudo journalctl -u f1tenth.service -n 50

# Testar script manualmente
sudo bash scripts/f1tenth_startup.sh

# Verificar permissões
ls -la scripts/
```

### **pigpiod não conecta**
```bash
# Verificar se está rodando
sudo systemctl status pigpiod

# Reiniciar pigpiod
sudo systemctl restart pigpiod

# Testar manualmente
pigpiod
```

### **Executáveis não encontrados**
```bash
# Rebuild com correções
bash scripts/build_f1tenth.sh

# Verificar links
ls -la install/f1tenth_control/lib/f1tenth_control/
```

## 🎯 Próximas Fases

Com a automatização completa, o sistema está pronto para:

1. **✅ Controle básico** - Servo + VESC funcionais
2. **🔄 Integração LiDAR** - Próxima fase
3. **🔄 SLAM e navegação** - Após LiDAR
4. **🔄 Algoritmos de racing** - Fase final

---

*Sistema F1TENTH - Totalmente automatizado para Raspberry Pi* 