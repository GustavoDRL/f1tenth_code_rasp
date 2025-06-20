#!/bin/bash
# Script de inicialização automática do sistema F1TENTH
# Para ser executado no boot do Raspberry Pi

set -e

# Configurações
WORKSPACE_DIR="/home/disney/Documents/f1tenth_code_rasp"
LOG_FILE="/var/log/f1tenth_startup.log"
USER="disney"

# Função de log
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"
}

log "🚀 Iniciando sistema F1TENTH..."

# Aguardar estabilização do sistema
log "⏳ Aguardando estabilização do sistema (10s)..."
sleep 10

# Verificar se workspace existe
if [ ! -d "$WORKSPACE_DIR" ]; then
    log "❌ Erro: Workspace F1TENTH não encontrado em $WORKSPACE_DIR"
    exit 1
fi

# Navegar para workspace
cd "$WORKSPACE_DIR"

# Configurar ambiente ROS2
log "🔧 Configurando ambiente ROS2..."
source /opt/ros/humble/setup.bash

# Source workspace se existir
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    log "✅ Workspace F1TENTH carregado"
else
    log "⚠️  Install não encontrado - executando build..."
    
    # Build do workspace como usuário correto
    sudo -u "$USER" bash -c "
        cd '$WORKSPACE_DIR'
        source /opt/ros/humble/setup.bash
        colcon build --packages-select f1tenth_control --symlink-install
    "
    
    # Executar post-build setup
    if [ -f "scripts/post_build_setup.sh" ]; then
        bash scripts/post_build_setup.sh
    fi
    
    source install/setup.bash
fi

# Iniciar pigpiod
log "🔧 Iniciando pigpiod..."
if command -v pigpiod >/dev/null 2>&1; then
    systemctl start pigpiod
    systemctl enable pigpiod
    log "✅ pigpiod iniciado e habilitado"
else
    log "❌ pigpiod não encontrado!"
    exit 1
fi

# Verificar se pigpiod está rodando
if systemctl is-active --quiet pigpiod; then
    log "✅ pigpiod está ativo"
else
    log "❌ Falha ao iniciar pigpiod"
    exit 1
fi

# Aguardar pigpiod estabilizar
sleep 2

# Verificar executáveis ROS2
log "🔍 Verificando executáveis ROS2..."
EXECUTABLES=("servo_control_node" "enhanced_servo_control_node")
for exe in "${EXECUTABLES[@]}"; do
    if ros2 pkg executables f1tenth_control | grep -q "$exe"; then
        log "  ✅ $exe encontrado"
    else
        log "  ❌ $exe não encontrado"
        exit 1
    fi
done

log "✅ Sistema F1TENTH inicializado com sucesso!"
log "📋 Para usar:"
log "   - Controle manual: ros2 launch f1tenth_control f1tenth_control.launch.py"
log "   - Status: systemctl status f1tenth.service"

exit 0 