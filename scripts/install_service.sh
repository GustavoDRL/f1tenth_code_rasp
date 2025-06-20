#!/bin/bash
# Script de instalação do serviço systemd F1TENTH
# Configura inicialização automática no boot

set -e

echo "🔧 Instalação do Serviço F1TENTH"
echo "================================="

# Verificar se é executado como root
if [ "$EUID" -ne 0 ]; then
    echo "❌ Erro: Execute este script como root (sudo)"
    exit 1
fi

# Verificar se arquivo de serviço existe
SERVICE_FILE="scripts/f1tenth.service"
if [ ! -f "$SERVICE_FILE" ]; then
    echo "❌ Erro: Arquivo de serviço não encontrado: $SERVICE_FILE"
    exit 1
fi

# Verificar se script de inicialização existe
STARTUP_SCRIPT="scripts/f1tenth_startup.sh"
if [ ! -f "$STARTUP_SCRIPT" ]; then
    echo "❌ Erro: Script de inicialização não encontrado: $STARTUP_SCRIPT"
    exit 1
fi

# Tornar scripts executáveis
echo "🔧 Configurando permissões..."
chmod +x scripts/*.sh

# Copiar arquivo de serviço para systemd
echo "📋 Instalando serviço systemd..."
cp "$SERVICE_FILE" /etc/systemd/system/

# Recarregar systemd
echo "🔄 Recarregando systemd..."
systemctl daemon-reload

# Habilitar serviço para iniciar no boot
echo "✅ Habilitando serviço F1TENTH..."
systemctl enable f1tenth.service

# Verificar status do pigpiod
echo "🔧 Configurando pigpiod..."
if command -v pigpiod >/dev/null 2>&1; then
    systemctl enable pigpiod
    echo "  ✅ pigpiod habilitado para iniciar no boot"
else
    echo "  ⚠️  pigpiod não encontrado - instale com: apt install pigpio"
fi

# Adicionar usuário disney ao grupo gpio (se necessário)
echo "👤 Configurando permissões de usuário..."
if ! groups disney | grep -q gpio; then
    usermod -a -G gpio disney
    echo "  ✅ Usuário disney adicionado ao grupo gpio"
fi

# Criar arquivo de configuração de ambiente
echo "📝 Criando configuração de ambiente..."
cat > /etc/environment.d/f1tenth.conf << EOF
# F1TENTH Environment Configuration
ROS_DOMAIN_ID=0
ROS_LOCALHOST_ONLY=1
AMENT_PREFIX_PATH=/home/disney/Documents/f1tenth_code_rasp/install:/opt/ros/humble
PYTHONPATH=/home/disney/Documents/f1tenth_code_rasp/install/f1tenth_control/lib/python3.10/site-packages:/opt/ros/humble/lib/python3.10/site-packages
EOF

echo ""
echo "✅ Instalação do serviço concluída!"
echo ""
echo "📋 Comandos úteis:"
echo "   • Status: sudo systemctl status f1tenth.service"
echo "   • Logs: sudo journalctl -u f1tenth.service -f"
echo "   • Iniciar: sudo systemctl start f1tenth.service"
echo "   • Parar: sudo systemctl stop f1tenth.service"
echo "   • Reiniciar: sudo systemctl restart f1tenth.service"
echo ""
echo "🔄 Para ativar agora: sudo systemctl start f1tenth.service"
echo "🚀 O sistema iniciará automaticamente no próximo boot!" 