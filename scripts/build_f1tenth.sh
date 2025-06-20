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

# Testar executáveis
echo "🧪 Testando executáveis..."
EXECUTABLES=("servo_control_node" "enhanced_servo_control_node" "servo_calibration")
for exe in "${EXECUTABLES[@]}"; do
    if ros2 run f1tenth_control "$exe" --help >/dev/null 2>&1; then
        echo "  ✅ $exe funcional"
    else
        echo "  ❌ $exe com problemas"
        exit 1
    fi
done

# Verificar launch files
echo "🚀 Testando launch files..."
if ros2 launch f1tenth_control f1tenth_control.launch.py --help >/dev/null 2>&1; then
    echo "  ✅ Launch files funcionais"
else
    echo "  ❌ Launch files com problemas"
    exit 1
fi

echo ""
echo "✅ Build F1TENTH concluído com sucesso!"
echo ""
echo "📋 Próximos passos:"
echo "   1. Testar: ros2 launch f1tenth_control f1tenth_control.launch.py"
echo "   2. Instalar serviço: sudo bash scripts/install_service.sh"
echo "   3. Status: systemctl status f1tenth.service" 