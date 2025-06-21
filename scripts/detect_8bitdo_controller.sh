#!/bin/bash

echo "🎮 DETECÇÃO AUTOMÁTICA 8BitDo CONTROLLER F1TENTH"
echo "================================================"
echo "Data: $(date '+%Y-%m-%d %H:%M:%S')"
echo

# Cores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Função para logs coloridos
log_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

log_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

log_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

log_error() {
    echo -e "${RED}❌ $1${NC}"
}

# Verificar dispositivos USB
echo "1. DISPOSITIVOS USB CONECTADOS:"
echo "=============================="
USB_8BITDO=$(lsusb | grep -i "8bitdo\|2dc8")
if [ -n "$USB_8BITDO" ]; then
    log_success "Controle 8BitDo detectado via USB:"
    echo "$USB_8BITDO"
else
    log_error "Controle 8BitDo NÃO detectado via USB"
    echo "Dispositivos USB disponíveis:"
    lsusb | head -5
fi
echo

# Verificar dispositivos joystick
echo "2. DISPOSITIVOS JOYSTICK (/dev/input/js*):"
echo "=========================================="
if ls /dev/input/js* 1> /dev/null 2>&1; then
    log_success "Dispositivos joystick encontrados:"
    ls -la /dev/input/js*
    
    # Testar cada dispositivo joystick
    for js_device in /dev/input/js*; do
        echo "Testando $js_device..."
        if timeout 2s jstest "$js_device" --version > /dev/null 2>&1; then
            log_success "$js_device está funcional"
        else
            log_warning "$js_device não responde ou sem permissão"
        fi
    done
else
    log_error "NENHUM dispositivo joystick (/dev/input/js*) encontrado"
    log_warning "Controle pode estar em modo teclado"
fi
echo

# Verificar dispositivos event
echo "3. DISPOSITIVOS EVENT (/dev/input/event*):"
echo "========================================="
echo "Primeiros 5 dispositivos event:"
ls -la /dev/input/event* | head -5
echo

# Procurar especificamente pelo 8BitDo nos eventos
echo "4. DETECÇÃO 8BitDo EM DISPOSITIVOS EVENT:"
echo "========================================"
FOUND_8BITDO=false
for event in /dev/input/event*; do
    if sudo timeout 1s evtest "$event" < /dev/null 2>&1 | grep -q "8BitDo"; then
        log_success "Controle 8BitDo encontrado em: $event"
        echo "Informações do dispositivo:"
        sudo evtest "$event" < /dev/null 2>&1 | grep -E "Input device name|vendor|product|version" | head -4
        FOUND_8BITDO=true
        BITDO_EVENT_DEVICE="$event"
        break
    fi
done

if [ "$FOUND_8BITDO" = false ]; then
    log_error "Controle 8BitDo NÃO encontrado em dispositivos event"
fi
echo

# Verificar módulos do kernel
echo "5. MÓDULOS DO KERNEL:"
echo "===================="
MODULES=$(lsmod | grep -E "joydev|uinput|input")
if [ -n "$MODULES" ]; then
    log_success "Módulos relacionados a joystick carregados:"
    echo "$MODULES"
else
    log_warning "Módulos de joystick podem não estar carregados"
    echo "Para carregar: sudo modprobe joydev uinput"
fi
echo

# Verificar grupos do usuário
echo "6. PERMISSÕES DO USUÁRIO:"
echo "========================"
USER_GROUPS=$(groups $USER)
if echo "$USER_GROUPS" | grep -q "input"; then
    log_success "Usuário $USER está no grupo 'input'"
else
    log_warning "Usuário $USER NÃO está no grupo 'input'"
    echo "Para adicionar: sudo usermod -a -G input $USER"
fi

if echo "$USER_GROUPS" | grep -q "dialout"; then
    log_success "Usuário $USER está no grupo 'dialout'"
else
    log_warning "Usuário $USER NÃO está no grupo 'dialout'"
    echo "Para adicionar: sudo usermod -a -G dialout $USER"
fi
echo

# Verificar regras udev
echo "7. REGRAS UDEV:"
echo "=============="
UDEV_RULES="/etc/udev/rules.d/99-8bitdo-joystick.rules"
if [ -f "$UDEV_RULES" ]; then
    log_success "Regras udev para 8BitDo encontradas: $UDEV_RULES"
else
    log_warning "Regras udev para 8BitDo NÃO encontradas"
    echo "Sugestão: Criar $UDEV_RULES com conteúdo:"
    echo '# 8BitDo Ultimate 2C Wireless Controller'
    echo 'SUBSYSTEM=="input", ATTRS{idVendor}=="2dc8", ATTRS{idProduct}=="310a", MODE="0666", GROUP="input"'
    echo 'KERNEL=="js[0-9]*", ATTRS{idVendor}=="2dc8", ATTRS{idProduct}=="310a", MODE="0666", GROUP="input"'
fi
echo

# Testar ROS2 joy_node
echo "8. TESTE ROS2 JOY_NODE:"
echo "======================"
if command -v ros2 > /dev/null 2>&1; then
    log_info "ROS2 detectado, testando joy_node..."
    
    # Verificar se joy_node está rodando
    if pgrep -f "joy_node" > /dev/null; then
        log_success "joy_node já está rodando"
        
        # Verificar se está publicando
        log_info "Verificando publicação no tópico /joy..."
        if timeout 3s ros2 topic echo /joy --once > /dev/null 2>&1; then
            log_success "Tópico /joy está publicando dados"
        else
            log_error "Tópico /joy NÃO está publicando ou não existe"
        fi
    else
        log_warning "joy_node NÃO está rodando"
    fi
else
    log_warning "ROS2 não detectado ou não configurado"
fi
echo

# Sugestões de correção
echo "🔧 SUGESTÕES DE CORREÇÃO:"
echo "========================"

if [ "$FOUND_8BITDO" = true ] && ! ls /dev/input/js* 1> /dev/null 2>&1; then
    echo "1. PROBLEMA: Controle detectado como teclado, não como joystick"
    echo "   SOLUÇÃO: Configurar modo joystick no controle 8BitDo"
    echo "   - Tente diferentes modos no controle (X+Start, A+Start, etc.)"
    echo "   - Instalar driver específico: sudo apt install xboxdrv"
    echo
fi

if ! echo "$USER_GROUPS" | grep -q "input"; then
    echo "2. PROBLEMA: Usuário sem permissão para acessar dispositivos input"
    echo "   SOLUÇÃO: sudo usermod -a -G input $USER && logout/login"
    echo
fi

if [ ! -f "$UDEV_RULES" ]; then
    echo "3. PROBLEMA: Regras udev não configuradas"
    echo "   SOLUÇÃO: Criar regras udev personalizadas"
    echo "   sudo nano /etc/udev/rules.d/99-8bitdo-joystick.rules"
    echo
fi

if [ -z "$MODULES" ]; then
    echo "4. PROBLEMA: Módulos do kernel não carregados"
    echo "   SOLUÇÃO: sudo modprobe joydev uinput"
    echo
fi

echo "5. TESTE MANUAL RECOMENDADO:"
echo "   # Testar detecção manual"
echo "   sudo jstest /dev/input/js0"
echo "   "
echo "   # Testar ROS2 joy_node"
echo "   ros2 run joy joy_node --ros-args -p device_id:=0"
echo "   "
echo "   # Em outro terminal:"
echo "   ros2 topic echo /joy"
echo

# Resumo final
echo "📊 RESUMO DIAGNÓSTICO:"
echo "====================="
if [ "$FOUND_8BITDO" = true ]; then
    if ls /dev/input/js* 1> /dev/null 2>&1; then
        log_success "STATUS: Controle 8BitDo OK - Pronto para usar"
    else
        log_warning "STATUS: Controle detectado mas precisa configuração de modo"
    fi
else
    log_error "STATUS: Controle 8BitDo NÃO detectado - Verificar conexão"
fi
echo

echo "🏁 PRÓXIMOS PASSOS RECOMENDADOS:"
echo "1. Aplicar correções sugeridas acima"
echo "2. Testar: ros2 launch joy_converter launch_joy_ackerman_fixed.py"
echo "3. Validar: ros2 topic echo /drive"
echo
echo "Fim do diagnóstico - $(date '+%H:%M:%S')" 