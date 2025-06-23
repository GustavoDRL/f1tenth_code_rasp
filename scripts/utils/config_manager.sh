#!/bin/bash
# =============================================================================
# F1TENTH Config Manager - Configurações Centralizadas
# Versão: 2.0.0
# Descrição: Gerenciamento de configurações e parâmetros globais
# =============================================================================

# Proteção contra sourcing múltiplo
[[ "${F1TENTH_CONFIG_MANAGER_LOADED:-}" == "true" ]] && return 0
readonly F1TENTH_CONFIG_MANAGER_LOADED="true"

# =============================================================================
# CONFIGURAÇÕES GLOBAIS F1TENTH
# =============================================================================

# Informações do projeto
readonly F1TENTH_PROJECT_NAME="F1TENTH Autonomous Racing System"
readonly F1TENTH_VERSION="2.0.0"
readonly F1TENTH_DESCRIPTION="Sistema autônomo de corrida F1TENTH para Raspberry Pi"

# Configurações específicas adicionais (não definidas em common_functions.sh)
F1TENTH_BUILD_TYPE="${F1TENTH_BUILD_TYPE:-Release}"
F1TENTH_CMAKE_ARGS="${F1TENTH_CMAKE_ARGS:--DCMAKE_BUILD_TYPE=Release}"
F1TENTH_BUILD_TIMEOUT="${F1TENTH_BUILD_TIMEOUT:-300}"
F1TENTH_TEST_DURATION="${F1TENTH_TEST_DURATION:-30}"
F1TENTH_VERBOSE="${F1TENTH_VERBOSE:-false}"
F1TENTH_LOG_LEVEL="${F1TENTH_LOG_LEVEL:-INFO}"
F1TENTH_PARALLEL_WORKERS="${F1TENTH_PARALLEL_WORKERS:-$(nproc)}"
F1TENTH_DEBUG_MODE="${F1TENTH_DEBUG_MODE:-false}"

# Configurações ROS2 específicas
F1TENTH_RMW_IMPLEMENTATION="${F1TENTH_RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
F1TENTH_ROS_DOMAIN_ID="${F1TENTH_ROS_DOMAIN_ID:-0}"
F1TENTH_ROS_LOCALHOST_ONLY="${F1TENTH_ROS_LOCALHOST_ONLY:-1}"

# Configurações de hardware
F1TENTH_SERVO_GPIO_PIN="${F1TENTH_SERVO_GPIO_PIN:-18}"
F1TENTH_SERVO_PWM_FREQ="${F1TENTH_SERVO_PWM_FREQ:-50}"
F1TENTH_SERVO_MIN_PULSE="${F1TENTH_SERVO_MIN_PULSE:-1000}"
F1TENTH_SERVO_MAX_PULSE="${F1TENTH_SERVO_MAX_PULSE:-2000}"
F1TENTH_MAX_STEERING_ANGLE="${F1TENTH_MAX_STEERING_ANGLE:-0.4189}"

# Configurações de dispositivos
F1TENTH_VESC_DEVICE="${F1TENTH_VESC_DEVICE:-/dev/ttyACM0}"
F1TENTH_LIDAR_DEVICE="${F1TENTH_LIDAR_DEVICE:-/dev/ttyUSB0}"
F1TENTH_JOYSTICK_DEVICE="${F1TENTH_JOYSTICK_DEVICE:-/dev/input/js0}"

# Configurações de teste
F1TENTH_TEST_TYPE="${F1TENTH_TEST_TYPE:-servo}"
F1TENTH_INTERACTIVE_MODE="${F1TENTH_INTERACTIVE_MODE:-true}"
F1TENTH_LOG_TESTS="${F1TENTH_LOG_TESTS:-true}"
F1TENTH_RUN_TESTS="${F1TENTH_RUN_TESTS:-false}"
F1TENTH_QUICK_MODE="${F1TENTH_QUICK_MODE:-false}"

# Configurações de startup
F1TENTH_STARTUP_MODE="${F1TENTH_STARTUP_MODE:-complete}"
F1TENTH_AUTO_BUILD="${F1TENTH_AUTO_BUILD:-false}"
F1TENTH_STARTUP_DELAY="${F1TENTH_STARTUP_DELAY:-5}"
F1TENTH_MAX_STARTUP_ATTEMPTS="${F1TENTH_MAX_STARTUP_ATTEMPTS:-3}"

# Configurações derivadas (baseadas nas variáveis do common_functions.sh)
F1TENTH_HOME_DIR="${F1TENTH_HOME_DIR:-/home/$F1TENTH_USER}"
F1TENTH_CONFIG_DIR="${F1TENTH_CONFIG_DIR:-$F1TENTH_WORKSPACE/.config}"
F1TENTH_SRC_DIR="$F1TENTH_WORKSPACE/src"
F1TENTH_BUILD_DIR="$F1TENTH_WORKSPACE/build"
F1TENTH_INSTALL_DIR="$F1TENTH_WORKSPACE/install"
F1TENTH_SCRIPTS_DIR="$F1TENTH_WORKSPACE/scripts"

# =============================================================================
# CONFIGURAÇÕES DE PACOTES ROS2
# =============================================================================

# Lista de pacotes principais
readonly F1TENTH_ROS2_PACKAGES=(
    "f1tenth_control"
    "joy_converter"
    "vesc_msgs"
    "vesc_driver"
    "vesc_ackermann"
    "ydlidar_ros2_driver"
)

# Executáveis principais
readonly F1TENTH_EXECUTABLES=(
    "servo_control_node"
    "enhanced_servo_control_node"
    "servo_calibration"
)

# Launch files principais
readonly F1TENTH_LAUNCH_FILES=(
    "f1tenth_control.launch.py"
    "f1tenth_complete_system.launch.py"
    "f1tenth_system_no_lidar.launch.py"
    "f1tenth_servo_only.launch.py"
)

# Tópicos ROS2 principais
readonly F1TENTH_ROS2_TOPICS=(
    "/joy"
    "/drive"
    "/cmd_vel"
    "/ego_racecar/odom"
    "/scan"
    "/servo_angle"
    "/commands/motor/speed"
    "/sensors/core"
)

# =============================================================================
# CONFIGURAÇÕES DE DEPENDÊNCIAS
# =============================================================================

# Dependências do sistema
readonly F1TENTH_SYSTEM_DEPS=(
    "build-essential"
    "cmake"
    "git"
    "python3-pip"
    "python3-dev"
    "libudev-dev"
    "pkg-config"
    "libusb-1.0-0-dev"
    "python3-serial"
    "python3-setuptools"
    "wget"
    "unzip"
    "pigpio"
    "python3-pigpio"
)

# Grupos de usuário necessários
readonly F1TENTH_USER_GROUPS=(
    "dialout"
    "gpio"
    "plugdev"
)

# =============================================================================
# FUNÇÕES DE CONFIGURAÇÃO
# =============================================================================

# Exibir configuração atual
show_f1tenth_config() {
    echo "🔧 F1TENTH Configuration v$F1TENTH_VERSION"
    echo "=========================================="
    echo
    echo "📂 Paths:"
    echo "   User: $F1TENTH_USER"
    echo "   Workspace: $F1TENTH_WORKSPACE"
    echo "   Logs: $F1TENTH_LOG_DIR"
    echo "   Scripts: $F1TENTH_SCRIPTS_DIR"
    echo
    echo "🏗️ Build:"
    echo "   Type: $F1TENTH_BUILD_TYPE"
    echo "   Jobs: $F1TENTH_BUILD_JOBS"
    echo "   Timeout: $F1TENTH_BUILD_TIMEOUT s"
    echo
    echo "🤖 Hardware:"
    echo "   Servo GPIO: $F1TENTH_SERVO_GPIO_PIN"
    echo "   VESC Device: $F1TENTH_VESC_DEVICE"
    echo "   LiDAR Device: $F1TENTH_LIDAR_DEVICE"
    echo
    echo "🎮 Test:"
    echo "   Type: $F1TENTH_TEST_TYPE"
    echo "   Duration: $F1TENTH_TEST_DURATION s"
    echo "   Interactive: $F1TENTH_INTERACTIVE_MODE"
    echo
}

# Validar configuração
validate_f1tenth_config() {
    local errors=0
    
    # Verificar workspace
    if [[ ! -d "$F1TENTH_WORKSPACE" ]]; then
        echo "❌ Workspace não existe: $F1TENTH_WORKSPACE"
        ((errors++))
    fi
    
    # Verificar diretórios principais
    local required_dirs=("$F1TENTH_SRC_DIR" "$F1TENTH_SCRIPTS_DIR")
    for dir in "${required_dirs[@]}"; do
        if [[ ! -d "$dir" ]]; then
            echo "❌ Diretório não existe: $dir"
            ((errors++))
        fi
    done
    
    # Verificar valores válidos
    if [[ ! "$F1TENTH_BUILD_TYPE" =~ ^(Release|Debug|RelWithDebInfo)$ ]]; then
        echo "❌ Build type inválido: $F1TENTH_BUILD_TYPE"
        ((errors++))
    fi
    
    if [[ $errors -eq 0 ]]; then
        echo "✅ Configuração válida"
        return 0
    else
        echo "❌ $errors erro(s) de configuração encontrado(s)"
        return 1
    fi
}

# Configurações para diferentes modos
set_f1tenth_build_mode() {
    local mode="$1"
    
    case "$mode" in
        "release")
            export F1TENTH_BUILD_TYPE="Release"
            export F1TENTH_DEBUG="false"
            export F1TENTH_VERBOSE="false"
            ;;
        "debug")
            export F1TENTH_BUILD_TYPE="Debug"
            export F1TENTH_DEBUG="true"
            export F1TENTH_VERBOSE="true"
            ;;
        "quick")
            export F1TENTH_QUICK_MODE="true"
            export F1TENTH_RUN_TESTS="false"
            ;;
        *)
            echo "❌ Modo inválido: $mode"
            echo "Modos disponíveis: release, debug, quick"
            return 1
            ;;
    esac
    
    echo "✅ Modo configurado: $mode"
}

# =============================================================================
# INICIALIZAÇÃO
# =============================================================================

# Criar diretórios necessários se não existirem
ensure_f1tenth_directories() {
    local dirs=("$F1TENTH_LOG_DIR" "$F1TENTH_CONFIG_DIR")
    
    for dir in "${dirs[@]}"; do
        if [[ ! -d "$dir" ]]; then
            mkdir -p "$dir" 2>/dev/null || true
        fi
    done
}

# Inicializar configuração
init_f1tenth_config() {
    ensure_f1tenth_directories
    
    # Log de inicialização
    if [[ "$F1TENTH_DEBUG" == "true" ]]; then
        echo "🔧 F1TENTH Config Manager loaded successfully"
    fi
}

# Executar inicialização automaticamente
init_f1tenth_config 