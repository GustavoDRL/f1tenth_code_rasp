# 🛠️ Scripts F1TENTH - Raspberry Pi

Scripts automatizados para instalação, build e operação do sistema F1TENTH no Raspberry Pi.

## 📂 Scripts Disponíveis

### 🔧 **setup_raspberry_dependencies.sh**
**Propósito**: Instalação completa de dependências no Raspberry Pi
- Instala YDLiDAR SDK
- Configura middleware ROS2
- Instala pigpio para controle GPIO
- Configura permissões USB e grupos de usuário
- Cria ambiente F1TENTH

**Uso**:
```bash
chmod +x scripts/setup_raspberry_dependencies.sh
./scripts/setup_raspberry_dependencies.sh
```

### 🏗️ **build_and_test_f1tenth.sh**
**Propósito**: Build completo e validação do sistema
- Build de todos os pacotes ROS2
- Validação de instalação
- Testes de comunicação
- Verificação de hardware
- Diagnóstico completo

**Uso**:
```bash
chmod +x scripts/build_and_test_f1tenth.sh
./scripts/build_and_test_f1tenth.sh
```

### 🚀 **f1tenth_startup.sh**
**Propósito**: Inicialização do sistema F1TENTH
- Carregamento do ambiente ROS2
- Launch do sistema completo
- Monitoramento de status

**Uso**:
```bash
chmod +x scripts/f1tenth_startup.sh
./scripts/f1tenth_startup.sh
```

### 🧪 **test_f1tenth.sh**
**Propósito**: Testes básicos do sistema
- Teste de movimento do servo
- Verificação de comunicação ROS2
- Validação de hardware

**Uso**:
```bash
chmod +x scripts/test_f1tenth.sh
./scripts/test_f1tenth.sh
```

### 🎮 **test_f1tenth_manual_control.sh**
**Propósito**: Teste de controle manual
- Teste com joystick
- Validação de comandos manuais

**Uso**:
```bash
chmod +x scripts/test_f1tenth_manual_control.sh
./scripts/test_f1tenth_manual_control.sh
```

### 🔍 **detect_8bitdo_controller.sh**
**Propósito**: Detecção e configuração de joystick 8BitDo
- Verificação de conexão Bluetooth
- Configuração automática

**Uso**:
```bash
chmod +x scripts/detect_8bitdo_controller.sh
./scripts/detect_8bitdo_controller.sh
```

## 📋 Sequência de Configuração no Raspberry Pi

### 1️⃣ **Primeira Instalação**
```bash
# 1. Clonar repositório
cd ~/Documents
git clone [URL_REPOSITORIO] f1tenth_code_rasp
cd f1tenth_code_rasp

# 2. Instalar dependências (pode demorar 10-15 min)
chmod +x scripts/setup_raspberry_dependencies.sh
./scripts/setup_raspberry_dependencies.sh

# 3. Fazer logout/login ou reiniciar
logout
# ou: sudo reboot
```

### 2️⃣ **Build e Teste**
```bash
# 4. Build do sistema (3-5 min)
cd ~/Documents/f1tenth_code_rasp
chmod +x scripts/build_and_test_f1tenth.sh
./scripts/build_and_test_f1tenth.sh

# 5. Teste básico
chmod +x scripts/test_f1tenth.sh
./scripts/test_f1tenth.sh
```

### 3️⃣ **Operação**
```bash
# 6. Iniciar sistema completo
chmod +x scripts/f1tenth_startup.sh
./scripts/f1tenth_startup.sh

# 7. Controle manual (outro terminal)
chmod +x scripts/test_f1tenth_manual_control.sh
./scripts/test_f1tenth_manual_control.sh
```

## ⚠️ Notas Importantes

- **Não execute com sudo**: Os scripts verificam permissões e pedem sudo quando necessário
- **Logout necessário**: Após instalar dependências, é necessário logout/login para grupos de usuário
- **Hardware conectado**: Certifique-se que VESC, servo e LiDAR estão conectados antes dos testes
- **Tempo de execução**: setup_raspberry_dependencies.sh pode demorar 10-15 minutos na primeira vez

## 🔧 Troubleshooting

### Script não executa
```bash
# Verificar permissões
ls -la scripts/
# Se necessário:
chmod +x scripts/*.sh
```

### Dependências faltando
```bash
# Re-executar instalação de dependências
./scripts/setup_raspberry_dependencies.sh
```

### Build falha
```bash
# Limpar e tentar novamente
rm -rf build/ install/ log/
./scripts/build_and_test_f1tenth.sh
```

## 🎯 Próximas Fases

Com a automatização completa, o sistema está pronto para:

1. **✅ Controle básico** - Servo + VESC funcionais
2. **🔄 Integração LiDAR** - Próxima fase
3. **🔄 SLAM e navegação** - Após LiDAR
4. **🔄 Algoritmos de racing** - Fase final

---

*Sistema F1TENTH - Totalmente automatizado para Raspberry Pi* 