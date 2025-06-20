#!/bin/bash
# Script de configuração pós-build para F1TENTH
# Garante que os links simbólicos ROS2 sejam criados corretamente

set -e

echo "🔧 F1TENTH Post-Build Setup"
echo "================================"

# Definir diretórios
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_DIR="$WORKSPACE_DIR/install/f1tenth_control"
BIN_DIR="$INSTALL_DIR/bin"
LIB_DIR="$INSTALL_DIR/lib/f1tenth_control"

echo "📁 Workspace: $WORKSPACE_DIR"
echo "📁 Install: $INSTALL_DIR"

# Verificar se diretório bin existe
if [ ! -d "$BIN_DIR" ]; then
    echo "❌ Erro: Diretório bin não encontrado em $BIN_DIR"
    echo "   Execute primeiro: colcon build --packages-select f1tenth_control"
    exit 1
fi

# Criar diretório lib se não existir
echo "📂 Criando diretório lib..."
mkdir -p "$LIB_DIR"

# Lista de executáveis
EXECUTABLES=(
    "servo_control_node"
    "enhanced_servo_control_node"
    "servo_calibration"
)

# Criar links simbólicos
echo "🔗 Criando links simbólicos ROS2..."
for exe in "${EXECUTABLES[@]}"; do
    BIN_PATH="$BIN_DIR/$exe"
    LIB_PATH="$LIB_DIR/$exe"
    
    if [ -f "$BIN_PATH" ]; then
        # Remover link existente se houver
        [ -L "$LIB_PATH" ] && rm "$LIB_PATH"
        
        # Criar link simbólico relativo
        REL_PATH="../../bin/$exe"
        ln -sf "$REL_PATH" "$LIB_PATH"
        echo "  ✅ $exe -> $REL_PATH"
    else
        echo "  ⚠️  $exe não encontrado em $BIN_PATH"
    fi
done

# Verificar links criados
echo ""
echo "🔍 Verificando links criados:"
ls -la "$LIB_DIR/" || echo "❌ Erro ao listar diretório lib"

# Configurar pigpiod
echo ""
echo "🔧 Configurando pigpiod..."
if command -v pigpiod >/dev/null 2>&1; then
    sudo systemctl enable pigpiod
    sudo systemctl start pigpiod
    echo "  ✅ pigpiod habilitado e iniciado"
else
    echo "  ⚠️  pigpiod não encontrado - instale com: sudo apt install pigpio"
fi

echo ""
echo "✅ Setup pós-build concluído!"
echo ""
echo "📋 Para testar o sistema:"
echo "   source install/setup.bash"
echo "   ros2 launch f1tenth_control f1tenth_control.launch.py" 