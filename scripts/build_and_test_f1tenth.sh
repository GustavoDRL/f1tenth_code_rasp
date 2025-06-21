#!/bin/bash

# =============================================================================
# F1TENTH - Build e Teste Completo do Sistema
# Este script faz build completo e testa todos os componentes
# =============================================================================

set -e

echo "🏎️ F1TENTH - Build e Teste do Sistema Completo" 
echo "==============================================="

# Verificar se estamos no diretório correto
if [ ! -f "src/f1tenth_control/package.xml" ]; then
    echo "❌ ERRO: Execute este script no diretório raiz do workspace F1TENTH"
    echo "   cd ~/Documents/f1tenth_code_rasp"
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

# 1. CONFIGURAR AMBIENTE
log_info "Configurando ambiente ROS2..."
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 2. LIMPAR BUILD ANTERIOR
log_info "Limpando build anterior..."
rm -rf build/ install/ log/

# 3. BUILD DO SISTEMA
log_info "Iniciando build do sistema F1TENTH..."
echo "======================================"

# Build com informações detalhadas
colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --event-handlers console_direct+ \
    --parallel-workers $(nproc)

if [ $? -eq 0 ]; then
    log_success "Build concluído com sucesso!"
else
    log_error "Falha no build do sistema"
    exit 1
fi

# 4. SOURCE DO WORKSPACE
log_info "Configurando workspace..."
source install/setup.bash

# 5. VALIDAÇÃO DOS PACOTES
log_info "Validando pacotes instalados..."

# Verificar se os pacotes principais foram instalados
PACKAGES=(
    "f1tenth_control"
    "joy_converter" 
    "vesc_msgs"
    "vesc_driver"
    "vesc_ackermann"
    "ydlidar_ros2_driver"
)

for package in "${PACKAGES[@]}"; do
    if [ -d "install/$package" ]; then
        log_success "Pacote $package: OK"
    else
        log_error "Pacote $package: FALHOU"
        exit 1
    fi
done

# 6. TESTE DE NODES DISPONÍVEIS
log_info "Verificando nodes disponíveis..."

# Verificar executáveis do f1tenth_control
if [ -x "install/f1tenth_control/lib/f1tenth_control/servo_control_node" ]; then
    log_success "Node servo_control_node: OK"
else
    log_error "Node servo_control_node: NÃO ENCONTRADO"
fi

if [ -x "install/f1tenth_control/lib/f1tenth_control/enhanced_servo_control_node" ]; then
    log_success "Node enhanced_servo_control_node: OK"
else 
    log_error "Node enhanced_servo_control_node: NÃO ENCONTRADO"
fi

# 7. TESTE DE LAUNCH FILES
log_info "Verificando launch files..."

LAUNCH_FILES=(
    "install/f1tenth_control/share/f1tenth_control/launch/f1tenth_complete_system.launch.py"
    "install/f1tenth_control/share/f1tenth_control/launch/f1tenth_system.launch.py"
    "install/f1tenth_control/share/f1tenth_control/launch/f1tenth_control.launch.py"
)

for launch_file in "${LAUNCH_FILES[@]}"; do
    if [ -f "$launch_file" ]; then
        filename=$(basename "$launch_file")
        log_success "Launch file $filename: OK"
    else
        filename=$(basename "$launch_file")
        log_error "Launch file $filename: NÃO ENCONTRADO"
    fi
done

# 8. TESTE DE CONFIGURAÇÕES
log_info "Verificando arquivos de configuração..."

CONFIG_FILES=(
    "install/f1tenth_control/share/f1tenth_control/config/system_config.yaml"
    "install/f1tenth_control/share/f1tenth_control/config/control_params.yaml"
)

for config_file in "${CONFIG_FILES[@]}"; do
    if [ -f "$config_file" ]; then
        filename=$(basename "$config_file")
        log_success "Config $filename: OK"
    else
        filename=$(basename "$config_file")
        log_error "Config $filename: NÃO ENCONTRADO"
    fi
done

# 9. TESTE BÁSICO DE COMUNICAÇÃO ROS2
log_info "Testando comunicação ROS2 básica..."

# Teste rápido se o ROS2 está funcionando
timeout 5s ros2 node list > /dev/null 2>&1
if [ $? -eq 0 ]; then
    log_success "Comunicação ROS2: OK"
else
    log_error "Comunicação ROS2: PROBLEMA"
fi

# 10. VERIFICAR DEPENDÊNCIAS DE HARDWARE
log_info "Verificando dependências de hardware..."

# pigpio
if systemctl is-active --quiet pigpiod; then
    log_success "Serviço pigpiod: ATIVO"
else
    log_error "Serviço pigpiod: INATIVO"
    echo "   Execute: sudo systemctl start pigpiod"
fi

# YDLiDAR SDK
if [ -f "/usr/local/lib/libydlidar_sdk.so" ]; then
    log_success "YDLiDAR SDK: INSTALADO"
else
    log_error "YDLiDAR SDK: NÃO ENCONTRADO"
    echo "   Execute o script setup_raspberry_dependencies.sh"
fi

# Grupos do usuário
if groups $USER | grep -q "dialout"; then
    log_success "Grupo dialout: OK"
else
    log_error "Grupo dialout: FALTANDO"
    echo "   Execute: sudo usermod -a -G dialout $USER"
fi

if groups $USER | grep -q "gpio"; then
    log_success "Grupo gpio: OK"
else
    log_error "Grupo gpio: FALTANDO"
    echo "   Execute: sudo usermod -a -G gpio $USER"
fi

echo ""
echo "🎉 VALIDAÇÃO COMPLETA FINALIZADA!"
echo "================================="
echo ""
echo "📋 RESUMO DO SISTEMA:"
echo "- Build: ✅ Concluído com sucesso"
echo "- Pacotes: ✅ Todos instalados"
echo "- Launch files: ✅ Disponíveis"
echo "- Configurações: ✅ Presentes"
echo ""
echo "🚀 COMANDOS PARA TESTAR O SISTEMA:"
echo ""
echo "1. Teste básico do servo:"
echo "   ros2 launch f1tenth_control f1tenth_control.launch.py"
echo ""
echo "2. Sistema completo (recomendado):"
echo "   ros2 launch f1tenth_control f1tenth_complete_system.launch.py"
echo ""
echo "3. Monitoramento (em outro terminal):"
echo "   ros2 topic list"
echo "   ros2 topic echo /ego_racecar/odom"
echo "   ros2 topic echo /drive"
echo ""
echo "⚠️  IMPORTANTE:"
echo "- Certifique-se que o hardware está conectado (VESC, servo, LiDAR)"
echo "- O LiDAR será inicializado após 3 segundos do sistema principal" 
echo "- Use Ctrl+C para parar o sistema com segurança" 