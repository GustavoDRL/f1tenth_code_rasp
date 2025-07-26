# 🍓 SETUP COMPLETO RASPBERRY PI - SISTEMA F1TENTH

**Documento**: Guia Completo de Configuração
**Versão**: 1.0
**Data**: 2025-01-20
**Hardware Alvo**: Raspberry Pi 4B
**Sistema**: Ubuntu Server 22.04 LTS + ROS2 Humble

---

## 📋 PRÉ-REQUISITOS

### **Hardware Necessário**
- **Raspberry Pi 4B** (4GB+ RAM recomendado)
- **MicroSD Card** (32GB+ Classe 10 ou superior)
- **Fonte USB-C** (5V/3A oficial recomendada)
- **Cabo Ethernet** (para setup inicial)
- **Cabo USB para VESC** (USB-A para USB-C)
- **Servo RC** (compatível PWM 50Hz)
- **Jumpers GPIO** (para conexão servo)
- **VESC 6.0+** configurado
- **Joystick/Gamepad** (PS4/Xbox compatível)

### **Software Base**
- **Ubuntu Server 22.04 LTS** (ARM64)
- **ROS2 Humble Hawksbill**
- **Python 3.10+**
- **pigpio** (para controle GPIO)

---

## 🚀 PARTE 1: INSTALAÇÃO DO SISTEMA BASE

### **1.1 Preparação do MicroSD**

#### **Download Ubuntu Server 22.04 LTS ARM64**
```bash
# Em seu computador principal
wget https://ubuntu.com/download/raspberry-pi/
# Ou use Raspberry Pi Imager
```

#### **Flash do Sistema**
```bash
# Opção 1: Raspberry Pi Imager (recomendado)
# 1. Download: https://rpi.org/imager
# 2. Selecione Ubuntu Server 22.04 LTS (64-bit)
# 3. Configure SSH, WiFi, usuário via Advanced Options
# 4. Flash para MicroSD

# Opção 2: Manual via dd
sudo dd if=ubuntu-22.04-server-arm64+raspi.img of=/dev/sdX bs=4M status=progress
sync
```

#### **Configuração Inicial (Headless)**
```bash
# Monte a partição boot
sudo mount /dev/sdX1 /mnt

# Habilite SSH
sudo touch /mnt/ssh

# Configure WiFi (opcional)
sudo nano /mnt/wpa_supplicant.conf
```

**Conteúdo wpa_supplicant.conf**:
```
country=BR
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={
    ssid="SUA_REDE_WIFI"
    psk="SUA_SENHA_WIFI"
    key_mgmt=WPA-PSK
}
```

### **1.2 Primeiro Boot e Configuração**

#### **Conexão SSH Inicial**
```bash
# Conecte via Ethernet inicialmente
ssh ubuntu@raspberry-pi-ip

# Senha padrão: ubuntu (será solicitada alteração)
```

#### **Configuração Básica do Sistema**
```bash
# Update inicial
sudo apt update && sudo apt upgrade -y

# Configure timezone
sudo timedatectl set-timezone America/Sao_Paulo

# Configure hostname
sudo hostnamectl set-hostname f1tenth-pi

# Configure usuário para GPIO
sudo usermod -a -G dialout,gpio,i2c,spi $USER

# Reboot necessário
sudo reboot
```

---

## 🔧 PARTE 2: INSTALAÇÃO ROS2 HUMBLE

### **2.1 Configuração Repositórios ROS2**

```bash
# Instale dependências
sudo apt install -y software-properties-common curl

# Adicione chave GPG ROS2
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Adicione repositório
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package list
sudo apt update
```

### **2.2 Instalação ROS2 Humble**

```bash
# Instalação base ROS2
sudo apt install -y ros-humble-desktop

# Dependências adicionais para F1TENTH
sudo apt install -y \
  ros-humble-ackermann-msgs \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-tf2-ros \
  ros-humble-diagnostic-msgs \
  ros-humble-joy \
  ros-humble-joy-teleop

# Build tools
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool

# Inicialize rosdep
sudo rosdep init
rosdep update
```

### **2.3 Configuração Environment ROS2**

```bash
# Adicione ao .bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc

# Configure workspace
echo "export ROS_WORKSPACE=~/f1tenth_ws" >> ~/.bashrc
echo "source ~/f1tenth_ws/install/setup.bash" >> ~/.bashrc

# Reload terminal
source ~/.bashrc
```

---

## ⚡ PARTE 3: CONFIGURAÇÃO GPIO E PIGPIO

### **3.1 Instalação pigpio**

```bash
# Instale pigpio system-wide
sudo apt install -y pigpio python3-pigpio

# Habilite serviço pigpiod
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

# Verifique status
sudo systemctl status pigpiod
```

### **3.2 Configuração Permissões GPIO**

```bash
# Adicione usuário aos grupos necessários
sudo usermod -a -G gpio $USER

# Configure udev rules para GPIO
sudo nano /etc/udev/rules.d/99-gpio.rules
```

**Conteúdo 99-gpio.rules**:
```
SUBSYSTEM=="gpio", KERNEL=="gpio*", ACTION=="add", PROGRAM="/bin/sh -c 'chown root:gpio /sys/class/gpio/export /sys/class/gpio/unexport; chmod 220 /sys/class/gpio/export /sys/class/gpio/unexport'"
SUBSYSTEM=="gpio", KERNEL=="gpio*", ACTION=="add", PROGRAM="/bin/sh -c 'chown root:gpio /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value; chmod 660 /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value'"
```

### **3.3 Teste GPIO**

```bash
# Teste básico pigpio
python3 -c "
import pigpio
pi = pigpio.pi()
if pi.connected:
    print('pigpio conectado com sucesso!')
    pi.stop()
else:
    print('Erro: pigpio não conectou')
"
```

---

## 🔌 PARTE 4: CONFIGURAÇÃO HARDWARE VESC

### **4.1 Configuração Serial VESC**

```bash
# Identifique porta VESC
ls -la /dev/ttyACM*
# Deve aparecer /dev/ttyACM0 quando VESC conectado

# Configure permissões serial
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyACM0

# Teste comunicação serial
sudo apt install -y minicom
minicom -D /dev/ttyACM0 -b 115200
# Ctrl+A -> X para sair
```

### **4.2 Configuração Persistente VESC**

```bash
# Configure udev rule para VESC
sudo nano /etc/udev/rules.d/99-vesc.rules
```

**Conteúdo 99-vesc.rules**:
```
# VESC USB device
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="vesc", MODE="0666", GROUP="dialout"
```

```bash
# Recarregue regras udev
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

## 🎮 PARTE 5: CONFIGURAÇÃO JOYSTICK

### **5.1 Instalação Drivers Joystick**

```bash
# Instale suporte joystick
sudo apt install -y joystick jstest-gtk

# Teste joystick conectado
ls /dev/input/js*
# Deve mostrar /dev/input/js0

# Teste funcionalidade
jstest /dev/input/js0
# Use Ctrl+C para parar
```

### **5.2 Configuração Permissões Joystick**

```bash
# Configure udev para joystick
sudo nano /etc/udev/rules.d/99-joystick.rules
```

**Conteúdo 99-joystick.rules**:
```
# Generic joystick/gamepad devices
KERNEL=="js[0-9]*", MODE="0666", GROUP="input"
SUBSYSTEM=="input", ATTRS{name}=="*Controller*", MODE="0666", GROUP="input"
```

---

## 🏗️ PARTE 6: CRIAÇÃO WORKSPACE F1TENTH

### **6.1 Setup Workspace**

```bash
# Crie workspace
mkdir -p ~/f1tenth_ws/src
cd ~/f1tenth_ws/src

# Clone repositórios necessários
git clone https://github.com/f1tenth/vesc.git -b ros2
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git

# Clone seu projeto (ajuste URL)
git clone <URL_DO_SEU_PROJETO> f1tenth_code_rasp
```

### **6.2 Instalação Dependências**

```bash
# Volte para raiz do workspace
cd ~/f1tenth_ws

# Instale dependências via rosdep
rosdep install --from-paths src --ignore-src -r -y

# Dependências Python adicionais
pip3 install --user \
  numpy \
  matplotlib \
  pynput \
  dataclasses
```

### **6.3 Build Inicial**

```bash
# Build todos os pacotes
cd ~/f1tenth_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Teste build
ros2 pkg list | grep -E "(f1tenth|joy_converter|vesc)"
```

---

## 📡 PARTE 7: CONFIGURAÇÃO REDE E PERFORMANCE

### **7.1 Otimização Network**

```bash
# Configure DDS para melhor performance
echo "export FASTRTPS_DEFAULT_PROFILES_FILE=~/f1tenth_ws/fastrtps_profile.xml" >> ~/.bashrc

# Crie profile FastRTPS
nano ~/f1tenth_ws/fastrtps_profile.xml
```

**Conteúdo fastrtps_profile.xml**:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <builtin>
                <discovery_config>
                    <leaseDuration>
                        <sec>10</sec>
                        <nanosec>0</nanosec>
                    </leaseDuration>
                </discovery_config>
            </builtin>
        </rtps>
    </participant>
</profiles>
```

### **7.2 Otimização Sistema**

```bash
# Configure CPU governor para performance
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils

# Configure swappiness para melhor real-time
echo 'vm.swappiness=1' | sudo tee -a /etc/sysctl.conf

# Configure limites de memória
echo "$USER soft memlock unlimited" | sudo tee -a /etc/security/limits.conf
echo "$USER hard memlock unlimited" | sudo tee -a /etc/security/limits.conf

# Configure nice priority para ROS
echo "$USER soft nice -10" | sudo tee -a /etc/security/limits.conf
echo "$USER hard nice -10" | sudo tee -a /etc/security/limits.conf
```

---

## 🔧 PARTE 8: CONFIGURAÇÃO ESPECÍFICA F1TENTH

### **8.1 Calibração Servo**

```bash
# Execute calibração servo
cd ~/f1tenth_ws
source install/setup.bash

# Lance ferramenta calibração
ros2 run f1tenth_control servo_calibration

# Siga instruções para encontrar valores PWM min/max
# Anote valores para configuração
```

### **8.2 Configuração Parâmetros**

```bash
# Edite parâmetros conforme sua calibração
nano ~/f1tenth_ws/src/f1tenth_code_rasp/src/f1tenth_control/config/control_params.yaml

# Ajuste valores encontrados na calibração:
# servo_min_pulse_width: [SEU_VALOR_MIN]
# servo_max_pulse_width: [SEU_VALOR_MAX]
```

### **8.3 Teste Sistema Completo**

```bash
# Terminal 1: Launch sistema completo
ros2 launch f1tenth_control f1tenth_full.launch.py

# Terminal 2: Monitore tópicos
ros2 topic list
ros2 topic echo /drive

# Terminal 3: Teste joystick
ros2 topic echo /joy

# Verifique se servo responde aos comandos do joystick
```

---

## 🛡️ PARTE 9: SCRIPTS DE AUTOMAÇÃO

### **9.1 Script Startup Automático**

```bash
# Crie script de inicialização
nano ~/start_f1tenth.sh
```

**Conteúdo start_f1tenth.sh**:
```bash
#!/bin/bash

# F1TENTH Startup Script
echo "Iniciando sistema F1TENTH..."

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/f1tenth_ws/install/setup.bash

# Verifique hardware
echo "Verificando hardware..."
if [ ! -e "/dev/ttyACM0" ]; then
    echo "ERRO: VESC não encontrado em /dev/ttyACM0"
    exit 1
fi

if ! pgrep -x "pigpiod" > /dev/null; then
    echo "Iniciando pigpiod..."
    sudo systemctl start pigpiod
    sleep 2
fi

# Lance sistema
echo "Lançando sistema F1TENTH..."
ros2 launch f1tenth_control f1tenth_full.launch.py
```

```bash
# Torne executável
chmod +x ~/start_f1tenth.sh

# Teste script
./start_f1tenth.sh
```

### **9.2 Serviço Systemd (Opcional)**

```bash
# Crie serviço systemd
sudo nano /etc/systemd/system/f1tenth.service
```

**Conteúdo f1tenth.service**:
```ini
[Unit]
Description=F1TENTH Autonomous Vehicle System
After=network.target pigpiod.service
Requires=pigpiod.service

[Service]
Type=exec
User=ubuntu
Environment=ROS_DOMAIN_ID=42
ExecStart=/home/ubuntu/start_f1tenth.sh
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

```bash
# Habilite serviço (opcional)
sudo systemctl daemon-reload
sudo systemctl enable f1tenth.service

# NÃO inicie ainda - teste manual primeiro
# sudo systemctl start f1tenth.service
```

---

## ✅ PARTE 10: VERIFICAÇÃO FINAL

### **10.1 Checklist Funcionamento**

```bash
# 1. Teste ROS2
ros2 doctor

# 2. Teste GPIO
python3 -c "import pigpio; pi = pigpio.pi(); print('GPIO OK' if pi.connected else 'GPIO FAIL'); pi.stop()"

# 3. Teste VESC
ls -la /dev/ttyACM0

# 4. Teste Joystick
ls -la /dev/input/js0

# 5. Teste Packages
ros2 pkg list | grep -E "(f1tenth|joy_converter|vesc)"

# 6. Teste Launch
ros2 launch f1tenth_control f1tenth_control.launch.py --show-args
```

### **10.2 Script Diagnóstico**

```bash
# Crie script diagnóstico
nano ~/diagnostic_f1tenth.sh
```

**Conteúdo diagnostic_f1tenth.sh**:
```bash
#!/bin/bash

echo "=== DIAGNÓSTICO SISTEMA F1TENTH ==="
echo

echo "1. Sistema Base:"
echo "  Ubuntu: $(lsb_release -d | cut -f2)"
echo "  Kernel: $(uname -r)"
echo "  Arch: $(uname -m)"
echo

echo "2. ROS2:"
echo "  Distro: $(printenv ROS_DISTRO)"
echo "  Domain ID: $(printenv ROS_DOMAIN_ID)"
echo "  Workspace: $(printenv ROS_WORKSPACE)"
echo

echo "3. Hardware:"
echo -n "  VESC: "
[ -e "/dev/ttyACM0" ] && echo "OK (/dev/ttyACM0)" || echo "FAIL (não encontrado)"
echo -n "  Joystick: "
[ -e "/dev/input/js0" ] && echo "OK (/dev/input/js0)" || echo "FAIL (não encontrado)"
echo -n "  pigpiod: "
pgrep -x "pigpiod" > /dev/null && echo "OK (rodando)" || echo "FAIL (não rodando)"
echo

echo "4. Pacotes ROS2:"
ros2 pkg list | grep -E "(f1tenth|joy_converter|vesc)" | sed 's/^/  /'
echo

echo "5. Tópicos Ativos:"
timeout 5s ros2 topic list 2>/dev/null | sed 's/^/  /' || echo "  (ROS2 não ativo)"
echo

echo "=== FIM DIAGNÓSTICO ==="
```

```bash
# Torne executável e teste
chmod +x ~/diagnostic_f1tenth.sh
./diagnostic_f1tenth.sh
```

---

## 📚 RECURSOS ADICIONAIS

### **Comandos Úteis de Monitoramento**
```bash
# Monitor CPU/RAM
htop

# Monitor temperatura
vcgencmd measure_temp

# Monitor tópicos ROS2
ros2 topic hz /drive
ros2 topic bw /odom

# Visualizar grafo nós
ros2 run rqt_graph rqt_graph

# Monitor GPIO
gpio readall  # se gpio utils instalado
```

### **Backup Configuração**
```bash
# Backup workspace
tar -czf f1tenth_backup_$(date +%Y%m%d).tar.gz ~/f1tenth_ws/

# Backup configurações sistema
sudo tar -czf system_config_backup.tar.gz /etc/udev/rules.d/99-*
```

---

## 🚨 TROUBLESHOOTING COMUM

| Problema | Causa Provável | Solução |
|----------|----------------|---------|
| pigpio não conecta | Daemon não rodando | `sudo systemctl start pigpiod` |
| VESC não detectado | Cabo/permissions | Verificar `/dev/ttyACM0` e permissões |
| Servo não responde | GPIO/calibração | Verificar GPIO 18 e calibração |
| ROS2 não funciona | Environment | `source /opt/ros/humble/setup.bash` |
| Build falha | Dependências | `rosdep install --from-paths src --ignore-src -r -y` |

---

**SISTEMA CONFIGURADO COM SUCESSO!** 🎉

O Raspberry Pi está agora configurado para executar o sistema F1TENTH completo. Use `./start_f1tenth.sh` para iniciar o sistema ou consulte a documentação adicional para operação avançada.

---

*Guia atualizado em 2025-01-20 - Testado com Raspberry Pi 4B + Ubuntu 22.04*
