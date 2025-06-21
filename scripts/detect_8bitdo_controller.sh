#!/bin/bash

echo "🎮 DETECÇÃO AUTOMÁTICA 8BitDo CONTROLLER"
echo "========================================"

# Verificar dispositivos USB
echo "1. Dispositivos USB conectados:"
lsusb | grep -i "8bitdo\|2dc8\|xbox\|controller" || echo "   Nenhum controle detectado via USB"

echo -e "\n2. Dispositivos input disponíveis:"
# Verificar dispositivos joystick
if ls /dev/input/js* >/dev/null 2>&1; then
    echo "   ✅ Dispositivos joystick encontrados:"
    ls -la /dev/input/js*
else
    echo "   ❌ Nenhum dispositivo joystick (/dev/input/js*)"
fi

# Verificar dispositivos event
echo -e "\n3. Dispositivos event (primeiros 10):"
ls -la /dev/input/event* | head -10

# Verificar se o controle está sendo reconhecido
echo -e "\n4. Procurando controles 8BitDo:"
found_8bitdo=false
for event in /dev/input/event*; do
    if sudo timeout 2s evtest "$event" < /dev/null 2>&1 | grep -q -i "8BitDo"; then
        echo "   ✅ Controle 8BitDo encontrado em: $event"
        sudo timeout 2s evtest "$event" < /dev/null 2>&1 | grep -E "Input device name|vendor|product"
        found_8bitdo=true
    fi
done

if [ "$found_8bitdo" = false ]; then
    echo "   ❌ Controle 8BitDo não encontrado nos dispositivos event"
fi

# Testar dispositivos joystick se existirem
echo -e "\n5. Teste dispositivos joystick:"
for js_dev in /dev/input/js*; do
    if [ -e "$js_dev" ]; then
        echo "   Testando $js_dev:"
        if sudo timeout 3s jstest --non-interactive "$js_dev" 2>/dev/null | head -2; then
            echo "   ✅ $js_dev respondendo"
        else
            echo "   ❌ $js_dev não responde"
        fi
    fi
done 2>/dev/null

# Verificar permissões
echo -e "\n6. Verificação de permissões:"
for js_dev in /dev/input/js*; do
    if [ -e "$js_dev" ]; then
        if [ -r "$js_dev" ] && [ -w "$js_dev" ]; then
            echo "   ✅ $js_dev - permissões OK"
        else
            echo "   ❌ $js_dev - permissões insuficientes"
            echo "      Execute: sudo chmod 666 $js_dev"
        fi
    fi
done 2>/dev/null

# Verificar grupos do usuário
echo -e "\n7. Grupos do usuário atual:"
groups | grep -E "input|dialout|gpio" || echo "   ⚠️  Usuário não está nos grupos necessários (input, dialout, gpio)"

echo -e "\n8. Comandos para correção (se necessário):"
echo "   sudo usermod -a -G input,dialout,gpio \$USER"
echo "   sudo chmod 666 /dev/input/js*"
echo "   # Depois faça logout/login ou:"
echo "   newgrp input"

echo -e "\n=== FIM DIAGNÓSTICO ===" 