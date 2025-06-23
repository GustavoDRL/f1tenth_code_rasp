#!/bin/bash

# =============================================================================
# F1TENTH Raspberry Pi - Setup de Dependências Completo
# Este script instala todas as dependências necessárias para o sistema F1TENTH
# =============================================================================

set -e  # Para em caso de erro

echo "🏎️ F1TENTH - Configuração de Dependências do Raspberry Pi"
echo "========================================================="

# Verificar se está rodando como usuário normal (não root)
if [ "$EUID" -eq 0 ]; then
    echo "❌ ERRO: Não execute este script como root (sudo)"
    echo "   Execute como usuário normal. O script pedirá sudo quando necessário."
    exit 1
fi

# Função para logging
log_info() {
    echo "ℹ️  $1"
}

log_success() {
    echo "✅ $1"
}

log_error() {
    echo "❌ $1"
}

log_warning() {
    echo "⚠️  $1"
}

# 1. CONFIGURAR MIDDLEWARE ROS2
log_info "Configurando middleware ROS2..."

# Adicionar configuração do middleware ao bashrc se não existir
if ! grep -q "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# F1TENTH ROS2 Configuration" >> ~/.bashrc
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
    log_success "Middleware ROS2 configurado no ~/.bashrc"
else
    log_info "Middleware ROS2 já configurado"
fi

# Configurar para a sessão atual
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 2. ATUALIZAR SISTEMA
log_info "Atualizando sistema..."
sudo apt update
sudo apt upgrade -y

# 3. INSTALAR DEPENDÊNCIAS BÁSICAS
log_info "Instalando dependências básicas..."
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-dev \
    libudev-dev \
    pkg-config \
    libusb-1.0-0-dev \
    python3-serial \
    python3-setuptools \
    wget \
    unzip

# 4. INSTALAR PIGPIO (para controle do servo)
log_info "Instalando pigpio..."
sudo apt install -y pigpio python3-pigpio

# Habilitar e iniciar serviço pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

log_success "pigpio instalado e configurado"

# 5. INSTALAR SDK DO YDLIDAR
log_info "Instalando YDLiDAR SDK..."

# Criar diretório temporário
TEMP_DIR=$(mktemp -d)
cd "$TEMP_DIR"

# Baixar SDK do YDLiDAR
log_info "Baixando YDLiDAR SDK..."
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK

# Criar diretório de build
mkdir -p build
cd build

# Configurar e compilar
log_info "Compilando YDLiDAR SDK..."
cmake ..
make -j$(nproc)

# Instalar
log_info "Instalando YDLiDAR SDK..."
sudo make install

# Configurar bibliotecas
sudo ldconfig

log_success "YDLiDAR SDK instalado com sucesso"

# 6. CONFIGURAR PERMISSÕES USB
log_info "Configurando permissões USB..."

# Adicionar usuário aos grupos necessários
sudo usermod -a -G dialout $USER
sudo usermod -a -G gpio $USER
sudo usermod -a -G plugdev $USER

# Criar regra udev para YDLiDAR
sudo tee /etc/udev/rules.d/99-ydlidar.rules > /dev/null << 'EOF'
# YDLiDAR USB permissions
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", KERNEL=="ttyACM*", MODE="0666", GROUP="dialout"
EOF

# Recarregar regras udev
sudo udevadm control --reload-rules
sudo udevadm trigger

log_success "Permissões USB configuradas"

# 7. CONFIGURAR VARIÁVEIS DE AMBIENTE PERMANENTES
log_info "Configurando variáveis de ambiente..."

# Adicionar configurações ao bashrc se não existirem
ENV_CONFIGS=(
    "export YDLIDAR_SDK_PATH=/usr/local"
    "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH"
    "export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:\$PKG_CONFIG_PATH"
)

for config in "${ENV_CONFIGS[@]}"; do
    if ! grep -q "$config" ~/.bashrc; then
        echo "$config" >> ~/.bashrc
    fi
done

# Aplicar configurações na sessão atual
export YDLIDAR_SDK_PATH=/usr/local
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH

log_success "Variáveis de ambiente configuradas"

# 8. VERIFICAR INSTALAÇÃO DO SDK
log_info "Verificando instalação do YDLiDAR SDK..."

if [ -f "/usr/local/lib/libydlidar_sdk.so" ] && [ -d "/usr/local/include/ydlidar_sdk" ]; then
    log_success "YDLiDAR SDK instalado corretamente"
else
    log_error "Problema na instalação do YDLiDAR SDK"
    exit 1
fi

# 9. LIMPAR ARQUIVOS TEMPORÁRIOS
log_info "Limpando arquivos temporários..."
cd ~
rm -rf "$TEMP_DIR"

# 10. CONFIGURAÇÃO FINAL
log_info "Configuração final..."

# Criar arquivo de ambiente F1TENTH
tee ~/.f1tenth_env > /dev/null << 'EOF'
#!/bin/bash
# F1TENTH Environment Configuration

# ROS2 Configuration
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash

# YDLiDAR Configuration
export YDLIDAR_SDK_PATH=/usr/local
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH

# F1TENTH Workspace
if [ -f "$HOME/Documents/f1tenth_code_rasp/install/setup.bash" ]; then
    source $HOME/Documents/f1tenth_code_rasp/install/setup.bash
fi

echo "✅ F1TENTH environment loaded"
EOF

chmod +x ~/.f1tenth_env

# Adicionar source do ambiente ao bashrc
if ! grep -q "source ~/.f1tenth_env" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# F1TENTH Environment" >> ~/.bashrc
    echo "source ~/.f1tenth_env" >> ~/.bashrc
fi

log_success "Arquivo de ambiente F1TENTH criado"

echo ""
echo "🎉 INSTALAÇÃO CONCLUÍDA COM SUCESSO!"
echo "=========================================="
echo ""
echo "📋 PRÓXIMOS PASSOS:"
echo "1. Faça logout e login novamente (ou reinicie)"
echo "2. Execute: cd ~/Documents/f1tenth_code_rasp"
echo "3. Execute: colcon build --symlink-install"
echo "4. Execute: source install/setup.bash"
echo "5. Teste o sistema: ros2 launch f1tenth_control f1tenth_complete_system.launch.py"
echo ""
echo "⚠️  IMPORTANTE: É necessário fazer logout/login para que as"
echo "   configurações de grupo (dialout, gpio) tenham efeito."
echo ""
echo "🔧 Para verificar se tudo está funcionando:"
echo "   - pigpio: sudo systemctl status pigpiod"
echo "   - YDLiDAR SDK: ls -la /usr/local/lib/libydlidar_sdk.so"
echo "   - Grupos: groups \$USER" 