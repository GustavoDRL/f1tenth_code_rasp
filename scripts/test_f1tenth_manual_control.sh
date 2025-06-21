#!/bin/bash
# Script de teste completo para controle manual F1TENTH
# Valida joystick → drive → hardware em sequência

set -e

echo "🎮 TESTE CONTROLE MANUAL F1TENTH"
echo "================================="
echo "Data: $(date)"
echo

# Configurações
WORKSPACE_DIR="/home/disney/Documents/f1tenth_code_rasp"
LOG_FILE="~/test_manual_control_$(date +%Y%m%d_%H%M%S).log"
TEST_DURATION=30

# Navegar para workspace
cd $WORKSPACE_DIR

# Função de log
log() {
    echo "[$(date '+%H:%M:%S')] $1" | tee -a "$LOG_FILE"
}

log "🚀 Iniciando teste controle manual F1TENTH"

# FASE 1: Verificação pré-requisitos
echo "FASE 1: Verificação pré-requisitos"
echo "=================================="

# Verificar joystick
if ls /dev/input/js* >/dev/null 2>&1; then
    JOYSTICK_DEV=$(ls /dev/input/js* | head -1)
    log "✅ Joystick detectado: $JOYSTICK_DEV"
else
    log "❌ ERRO: Nenhum joystick detectado"
    echo "Execute primeiro: bash scripts/detect_8bitdo_controller.sh"
    exit 1
fi

# Verificar permissões
if [ -r "$JOYSTICK_DEV" ] && [ -w "$JOYSTICK_DEV" ]; then
    log "✅ Permissões joystick OK"
else
    log "❌ ERRO: Permissões insuficientes para $JOYSTICK_DEV"
    echo "Execute: sudo chmod 666 $JOYSTICK_DEV"
    exit 1
fi

# Verificar ROS2
if command -v ros2 >/dev/null 2>&1; then
    log "✅ ROS2 disponível"
else
    log "❌ ERRO: ROS2 não encontrado"
    echo "Execute: source /opt/ros/humble/setup.bash"
    exit 1
fi

# Verificar workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    log "✅ Workspace F1TENTH carregado"
else
    log "❌ ERRO: Workspace não compilado"
    echo "Execute: colcon build --symlink-install"
    exit 1
fi

# FASE 2: Teste joystick básico
echo -e "\nFASE 2: Teste joystick básico"
echo "============================="

log "🔍 Testando resposta básica do joystick"
timeout 3s sudo jstest --non-interactive "$JOYSTICK_DEV" > /dev/null 2>&1
if [ $? -eq 0 ]; then
    log "✅ Joystick responde aos comandos"
else
    log "⚠️  Joystick pode não estar respondendo adequadamente"
fi

# FASE 3: Teste ROS2 joy_node
echo -e "\nFASE 3: Teste ROS2 joy_node"
echo "==========================="

log "🚀 Iniciando joy_node em background"
ros2 run joy joy_node --ros-args -p device_id:=0 > /dev/null 2>&1 &
JOY_NODE_PID=$!
sleep 3

# Verificar se joy_node está publicando
log "🔍 Verificando publicação no tópico /joy"
if timeout 5s ros2 topic echo /joy --once > /dev/null 2>&1; then
    log "✅ Tópico /joy publicando dados"
else
    log "❌ ERRO: Tópico /joy não está publicando"
    kill $JOY_NODE_PID 2>/dev/null || true
    exit 1
fi

# Matar joy_node
kill $JOY_NODE_PID 2>/dev/null || true
sleep 1

# FASE 4: Teste sistema completo manual
echo -e "\nFASE 4: Teste sistema completo manual"
echo "====================================="

log "🚀 Iniciando sistema completo manual"
ros2 launch joy_converter launch_joy_ackerman_fixed.py > "$LOG_FILE.launch" 2>&1 &
LAUNCH_PID=$!
sleep 5

# Verificar se todos os nós estão rodando
log "🔍 Verificando nós ativos"
ACTIVE_NODES=$(ros2 node list 2>/dev/null | wc -l)
if [ "$ACTIVE_NODES" -ge 2 ]; then
    log "✅ Sistema iniciado: $ACTIVE_NODES nós ativos"
    ros2 node list | while read node; do
        log "   - $node"
    done
else
    log "❌ ERRO: Sistema não iniciou corretamente"
    kill $LAUNCH_PID 2>/dev/null || true
    exit 1
fi

# FASE 5: Validação comunicação
echo -e "\nFASE 5: Validação comunicação"
echo "============================"

# Testar tópico /joy
log "🔍 Testando tópico /joy"
if timeout 3s ros2 topic echo /joy --once > /dev/null 2>&1; then
    log "✅ Tópico /joy OK"
    JOY_HZ=$(timeout 5s ros2 topic hz /joy 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "N/A")
    log "   Frequência /joy: $JOY_HZ Hz"
else
    log "❌ Tópico /joy com problemas"
fi

# Testar tópico /drive
log "🔍 Testando tópico /drive"
if timeout 3s ros2 topic echo /drive --once > /dev/null 2>&1; then
    log "✅ Tópico /drive OK"
    DRIVE_HZ=$(timeout 5s ros2 topic hz /drive 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "N/A")
    log "   Frequência /drive: $DRIVE_HZ Hz"
else
    log "❌ Tópico /drive com problemas"
fi

# FASE 6: Teste interativo
echo -e "\nFASE 6: Teste interativo ($TEST_DURATION segundos)"
echo "==============================================="

log "🎯 TESTE INTERATIVO INICIADO"
echo ""
echo "INSTRUÇÕES PARA TESTE:"
echo "1. Mova o stick ESQUERDO para cima/baixo (velocidade)"
echo "2. Mova o stick DIREITO para esquerda/direita (direção)"
echo "3. Pressione o botão L1/LB para ativar o controle"
echo "4. Observe o movimento físico do servo/motor"
echo ""
echo "Teste será executado por $TEST_DURATION segundos..."
echo "Pressione Ctrl+C para parar antecipadamente"
echo ""

# Capturar dados de teste
timeout ${TEST_DURATION}s ros2 topic echo /drive > "$LOG_FILE.drive_data" 2>&1 &
ECHO_PID=$!

# Aguardar teste
sleep $TEST_DURATION
kill $ECHO_PID 2>/dev/null || true

# FASE 7: Análise resultados
echo -e "\nFASE 7: Análise resultados"
echo "=========================="

DRIVE_COMMANDS=$(grep -c "steering_angle\|speed" "$LOG_FILE.drive_data" 2>/dev/null || echo "0")
log "📊 Comandos /drive capturados: $DRIVE_COMMANDS"

if [ "$DRIVE_COMMANDS" -gt 10 ]; then
    log "✅ Sistema respondeu aos comandos do joystick"
    
    # Analisar variação dos comandos
    UNIQUE_ANGLES=$(grep "steering_angle" "$LOG_FILE.drive_data" 2>/dev/null | sort -u | wc -l || echo "0")
    UNIQUE_SPEEDS=$(grep "speed" "$LOG_FILE.drive_data" 2>/dev/null | sort -u | wc -l || echo "0")
    
    log "   Variações steering_angle: $UNIQUE_ANGLES"
    log "   Variações speed: $UNIQUE_SPEEDS"
    
    if [ "$UNIQUE_ANGLES" -gt 3 ] && [ "$UNIQUE_SPEEDS" -gt 3 ]; then
        log "✅ SUCESSO: Controle manual está funcionando corretamente!"
    else
        log "⚠️  Variação limitada - pode precisar mover mais o joystick"
    fi
else
    log "❌ Sistema não respondeu adequadamente aos comandos"
fi

# FASE 8: Cleanup
echo -e "\nFASE 8: Finalização"
echo "=================="

log "🛑 Parando sistema"
kill $LAUNCH_PID 2>/dev/null || true
sleep 2

# Matar processos remanescentes
pkill -f "joy_node" 2>/dev/null || true
pkill -f "joy_ackerman" 2>/dev/null || true

log "✅ Teste finalizado"

# RESUMO FINAL
echo -e "\n📋 RESUMO DO TESTE"
echo "=================="
echo "Log completo: $LOG_FILE"
echo "Dados drive: $LOG_FILE.drive_data"
echo "Log launch: $LOG_FILE.launch"
echo ""

if [ "$DRIVE_COMMANDS" -gt 10 ] && [ "$UNIQUE_ANGLES" -gt 3 ]; then
    echo "🎉 RESULTADO: CONTROLE MANUAL FUNCIONANDO!"
    echo "✅ Próximo passo: Integrar com sistema completo"
else
    echo "⚠️  RESULTADO: PRECISA AJUSTES"
    echo "🔧 Verifique joystick e tente novamente"
fi

echo ""
echo "Para ver detalhes:"
echo "  cat $LOG_FILE"
echo "  tail -f $LOG_FILE.drive_data"
echo "" 