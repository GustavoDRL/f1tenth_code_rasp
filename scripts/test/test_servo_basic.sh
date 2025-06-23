#!/bin/bash
# Script de teste F1TENTH - Teste simples do servo
# Move roda: centro → esquerda → direita → fim

set -e

echo "🧪 F1TENTH Servo Test"
echo "====================="

# Verificar se estamos no workspace
if [ ! -f "src/f1tenth_control/package.xml" ]; then
    echo "❌ Execute no diretório do workspace F1TENTH"
    exit 1
fi

# Source ambiente
echo "🔧 Carregando ambiente..."
source /opt/ros/humble/setup.bash
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✅ Workspace carregado"
else
    echo "❌ Workspace não encontrado - execute build primeiro"
    exit 1
fi

# Verificar se pigpiod está rodando
echo "🔍 Verificando pigpiod..."
if ! systemctl is-active --quiet pigpiod 2>/dev/null; then
    echo "⚠️  Iniciando pigpiod..."
    sudo systemctl start pigpiod
    sleep 2
fi

# Iniciar nó de controle em background
echo "🚀 Iniciando sistema de controle..."
ros2 run f1tenth_control servo_control_node &
CONTROL_PID=$!

# Aguardar nó inicializar
echo "⏳ Aguardando inicialização (3s)..."
sleep 3

# Função para parar tudo ao sair
cleanup() {
    echo ""
    echo "🛑 Parando sistema..."
    if [ ! -z "$CONTROL_PID" ]; then
        kill $CONTROL_PID 2>/dev/null || true
    fi
    # Comando final para centro e parada
    ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped \
        "{drive: {steering_angle: 0.0, speed: 0.0}}" --once 2>/dev/null || true
    echo "✅ Teste finalizado"
}

# Capturar sinais para cleanup
trap cleanup EXIT INT TERM

echo ""
echo "🎯 Iniciando teste do servo..."
echo "==============================="

# 1. Centro (posição neutra)
echo "1️⃣  Movendo para CENTRO..."
ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped \
    "{drive: {steering_angle: 0.0, speed: 0.0}}" --once
sleep 2

# 2. Esquerda
echo "2️⃣  Movendo para ESQUERDA..."
ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped \
    "{drive: {steering_angle: 0.3, speed: 0.0}}" --once
sleep 2

# 3. Direita  
echo "3️⃣  Movendo para DIREITA..."
ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped \
    "{drive: {steering_angle: -0.3, speed: 0.0}}" --once
sleep 2

# 4. Centro novamente
echo "4️⃣  Retornando ao CENTRO..."
ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped \
    "{drive: {steering_angle: 0.0, speed: 0.0}}" --once
sleep 1

echo ""
echo "✅ Teste do servo concluído!"
echo "   Se você viu a roda se mover, o sistema está funcionando!"
echo ""

# Cleanup será chamado automaticamente pelo trap 