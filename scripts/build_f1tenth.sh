#!/bin/bash
# Script de build automatizado para F1TENTH
# Inclui todas as correções e configurações necessárias

set -e

echo "🏗️  F1TENTH Build Script"
echo "========================"

# Verificar se estamos no diretório correto
if [ ! -f "src/f1tenth_control/package.xml" ]; then
    echo "❌ Erro: Execute este script no diretório raiz do workspace F1TENTH"
    echo "   Esperado: ~/Documents/f1tenth_code_rasp"
    exit 1
fi

# Limpar build anterior
echo "🧹 Limpando build anterior..."
rm -rf build/ install/ log/

# Configurar ambiente ROS2
echo "🔧 Configurando ambiente ROS2..."
source /opt/ros/humble/setup.bash

# Build do workspace
echo "🏗️  Compilando workspace..."
colcon build --packages-select f1tenth_control --symlink-install

# Verificar se build foi bem-sucedido
if [ ! -f "install/f1tenth_control/bin/servo_control_node" ]; then
    echo "❌ Erro: Build falhou - executáveis não encontrados"
    exit 1
fi

# Executar post-build setup
echo "🔧 Executando configuração pós-build..."
bash scripts/post_build_setup.sh

# Source workspace
echo "📁 Carregando workspace..."
source install/setup.bash

# Testar executáveis (validação simples - não executa)
echo "🧪 Testando executáveis..."
EXECUTABLES=("servo_control_node" "enhanced_servo_control_node" "servo_calibration")
for exe in "${EXECUTABLES[@]}"; do
    if [ -f "install/f1tenth_control/lib/f1tenth_control/$exe" ] && [ -x "install/f1tenth_control/lib/f1tenth_control/$exe" ]; then
        echo "  ✅ $exe encontrado e executável"
    else
        echo "  ⚠️  $exe não encontrado, mas continuando..."
    fi
done

# Verificar se ROS2 reconhece os executáveis
echo "🔍 Verificando reconhecimento ROS2..."
if ros2 pkg executables f1tenth_control | grep -q "servo_control_node"; then
    echo "  ✅ ROS2 reconhece os executáveis"
else
    echo "  ❌ ROS2 não reconhece os executáveis"
    exit 1
fi

# Verificar launch files (sintaxe apenas)
echo "🚀 Verificando launch files..."
if python3 -m py_compile src/f1tenth_control/launch/f1tenth_control.launch.py; then
    echo "  ✅ Launch files com sintaxe válida"
else
    echo "  ❌ Launch files com erro de sintaxe"
    exit 1
fi

echo ""
echo "✅ Build F1TENTH concluído com sucesso!"
echo ""
echo "📋 Próximos passos:"
echo "   1. Instalar serviço: sudo bash scripts/install_service.sh"
echo "   2. Testar manualmente: ros2 launch f1tenth_control f1tenth_control.launch.py"
echo "   3. Status: systemctl status f1tenth.service" 