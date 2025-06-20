#!/bin/bash
# Script de teste F1TENTH - Validação sem travamento
# Testa componentes de forma não-bloqueante

set -e

echo "🧪 F1TENTH Test Script"
echo "======================"

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

# Teste 1: Verificar executáveis
echo ""
echo "📋 Teste 1: Verificando executáveis..."
EXECUTABLES=("servo_control_node" "enhanced_servo_control_node" "servo_calibration")
for exe in "${EXECUTABLES[@]}"; do
    if [ -f "install/f1tenth_control/lib/f1tenth_control/$exe" ]; then
        echo "  ✅ $exe encontrado"
    else
        echo "  ❌ $exe não encontrado"
    fi
done

# Teste 2: ROS2 Package Discovery
echo ""
echo "📋 Teste 2: ROS2 Package Discovery..."
if ros2 pkg list | grep -q "f1tenth_control"; then
    echo "  ✅ Pacote f1tenth_control reconhecido"
else
    echo "  ❌ Pacote não reconhecido"
fi

# Teste 3: Executáveis ROS2
echo ""
echo "📋 Teste 3: Executáveis ROS2..."
if ros2 pkg executables f1tenth_control | grep -q "servo_control_node"; then
    echo "  ✅ servo_control_node registrado"
else
    echo "  ❌ servo_control_node não registrado"
fi

# Teste 4: pigpiod Status
echo ""
echo "📋 Teste 4: pigpiod Status..."
if systemctl is-active --quiet pigpiod 2>/dev/null; then
    echo "  ✅ pigpiod ativo"
elif command -v pigpiod >/dev/null 2>&1; then
    echo "  ⚠️  pigpiod instalado mas não ativo"
    echo "     Execute: sudo systemctl start pigpiod"
else
    echo "  ❌ pigpiod não instalado"
fi

# Teste 5: Launch file syntax
echo ""
echo "📋 Teste 5: Launch file syntax..."
if python3 -m py_compile src/f1tenth_control/launch/f1tenth_control.launch.py 2>/dev/null; then
    echo "  ✅ Launch file syntax OK"
else
    echo "  ❌ Launch file com erro de sintaxe"
fi

# Teste 6: Serviço systemd
echo ""
echo "📋 Teste 6: Serviço systemd..."
if systemctl is-enabled --quiet f1tenth.service 2>/dev/null; then
    if systemctl is-active --quiet f1tenth.service 2>/dev/null; then
        echo "  ✅ Serviço f1tenth ativo e habilitado"
    else
        echo "  ⚠️  Serviço habilitado mas não ativo"
        echo "     Status: $(systemctl is-active f1tenth.service 2>/dev/null || echo 'failed')"
    fi
else
    echo "  ❌ Serviço não encontrado ou não habilitado"
fi

echo ""
echo "📋 Teste Manual Sugerido:"
echo "   1. Start pigpiod: sudo systemctl start pigpiod"
echo "   2. Test launch: ros2 launch f1tenth_control f1tenth_control.launch.py"
echo "   3. Test servo: ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped \\"
echo "      \"{drive: {steering_angle: 0.1, speed: 0.0}}\" --once"
echo ""
echo "🔍 Para ver logs do serviço:"
echo "   sudo journalctl -u f1tenth.service -f" 