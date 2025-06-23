#!/bin/bash
# =============================================================================
# F1TENTH Common Functions - Utilitários Compartilhados
# Versão: 2.0.0
# Descrição: Funções reutilizáveis para todos os scripts F1TENTH
# =============================================================================

# Proteção contra sourcing múltiplo
[[ "${F1TENTH_COMMON_FUNCTIONS_LOADED:-}" == "true" ]] && return 0
readonly F1TENTH_COMMON_FUNCTIONS_LOADED="true"

# =============================================================================
# CONFIGURAÇÕES GLOBAIS PARAMETRIZADAS
# =============================================================================

# Configurações principais (podem ser sobrescritas por variáveis de ambiente)
# Configurações básicas (evitando conflitos com readonly)
F1TENTH_USER="${F1TENTH_USER:-disney}"
F1TENTH_WORKSPACE="${F1TENTH_WORKSPACE:-/home/$F1TENTH_USER/Documents/f1tenth_code_rasp}"
F1TENTH_LOG_DIR="${F1TENTH_LOG_DIR:-$HOME/logs/f1tenth}"
F1TENTH_BUILD_JOBS="${F1TENTH_BUILD_JOBS:-$(nproc)}"
ROS2_SETUP_PATH="${ROS2_SETUP_PATH:-/opt/ros/humble/setup.bash}"

# Configurações ROS2
readonly ROS2_DISTRO="${ROS2_DISTRO:-humble}"
readonly ROS2_SETUP_PATH="/opt/ros/$ROS2_DISTRO/setup.bash"
readonly F1TENTH_WORKSPACE_SETUP="$F1TENTH_WORKSPACE/install/setup.bash"

# Configurações de timeout e performance
F1TENTH_COMMAND_TIMEOUT="${F1TENTH_COMMAND_TIMEOUT:-30}"
F1TENTH_SERVICE_WAIT="${F1TENTH_SERVICE_WAIT:-5}"
F1TENTH_DEBUG="${F1TENTH_DEBUG:-false}"

# =============================================================================
# SISTEMA DE LOGGING ESTRUTURADO
# =============================================================================

# Variável global para arquivo de log atual
CURRENT_LOG="${CURRENT_LOG:-$F1TENTH_LOG_DIR/f1tenth_$(date +%Y%m%d_%H%M%S).log}"

# Garantir que diretório de logs existe
ensure_log_directory() {
    if [[ ! -d "$F1TENTH_LOG_DIR" ]]; then
        mkdir -p "$F1TENTH_LOG_DIR" 2>/dev/null || {
            # Fallback para /tmp se não conseguir criar no home
            CURRENT_LOG="/tmp/f1tenth_$(date +%Y%m%d_%H%M%S).log"
            F1TENTH_LOG_DIR="/tmp"
        }
    fi
}

# Funções de logging com timestamp e cores
log_info() {
    local message="$1"
    local timestamp="[$(date '+%H:%M:%S')]"
    echo -e "\033[34m$timestamp ℹ️  $message\033[0m" | tee -a "$CURRENT_LOG"
}

log_success() {
    local message="$1"
    local timestamp="[$(date '+%H:%M:%S')]"
    echo -e "\033[32m$timestamp ✅ $message\033[0m" | tee -a "$CURRENT_LOG"
}

log_error() {
    local message="$1"
    local timestamp="[$(date '+%H:%M:%S')]"
    echo -e "\033[31m$timestamp ❌ $message\033[0m" | tee -a "$CURRENT_LOG" >&2
}

log_warning() {
    local message="$1"
    local timestamp="[$(date '+%H:%M:%S')]"
    echo -e "\033[33m$timestamp ⚠️  $message\033[0m" | tee -a "$CURRENT_LOG"
}

log_debug() {
    local message="$1"
    if [[ "$F1TENTH_DEBUG" == "true" ]]; then
        local timestamp="[$(date '+%H:%M:%S')]"
        echo -e "\033[36m$timestamp 🐛 $message\033[0m" | tee -a "$CURRENT_LOG"
    fi
}

# Header padronizado para scripts
f1tenth_print_header() {
    local title="$1"
    local version="$2"
    
    echo ""
    echo -e "\033[36m===============================================================================\033[0m"
    echo -e "\033[37m🏎️  $title v$version\033[0m"
    echo -e "\033[36m===============================================================================\033[0m"
    echo ""
}

# Separador visual
f1tenth_separator() {
    echo -e "\033[34m-------------------------------------------------------------------------------\033[0m"
}

# Aliases para compatibilidade com scripts refatorados
f1tenth_log_info() { log_info "$@"; }
f1tenth_log_success() { log_success "$@"; }  
f1tenth_log_error() { log_error "$@"; }
f1tenth_log_warn() { log_warning "$@"; }

# Source ROS2 com verificação
f1tenth_source_ros2() {
    if [[ -f "/opt/ros/humble/setup.bash" ]]; then
        # Configurar todas as variáveis necessárias
        export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
        export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-/usr/bin/python3}"
        export COLCON_TRACE="${COLCON_TRACE:-}"
        export COLCON_PYTHON_EXECUTABLE="${COLCON_PYTHON_EXECUTABLE:-/usr/bin/python3}"
        export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
        
        # Source com proteção
        set +u
        source /opt/ros/humble/setup.bash 2>/dev/null
        set -u
        return 0
    else
        log_error "ROS2 Humble não encontrado"
        return 1
    fi
}

# =============================================================================
# VERIFICAÇÕES DE SISTEMA
# =============================================================================

# Verificar se ROS2 está disponível
check_ros2_available() {
    command -v ros2 >/dev/null 2>&1
}

# Verificar se pigpiod está rodando
check_pigpiod_running() {
    systemctl is-active --quiet pigpiod 2>/dev/null
}

# Verificar se workspace foi compilado
check_workspace_built() {
    [[ -f "$F1TENTH_WORKSPACE_SETUP" ]]
}

# Verificar se usuário está nos grupos necessários
check_user_groups() {
    local required_groups=("dialout" "gpio" "plugdev")
    local missing_groups=()
    
    for group in "${required_groups[@]}"; do
        if ! groups "$F1TENTH_USER" 2>/dev/null | grep -q "\b$group\b"; then
            missing_groups+=("$group")
        fi
    done
    
    if [[ ${#missing_groups[@]} -eq 0 ]]; then
        return 0
    else
        log_warning "Grupos faltando para $F1TENTH_USER: ${missing_groups[*]}"
        return 1
    fi
}

# Verificar se estamos no diretório correto do workspace
check_workspace_directory() {
    if [[ ! -f "src/f1tenth_control/package.xml" ]]; then
        log_error "Execute no diretório raiz do workspace F1TENTH"
        log_error "Esperado: $F1TENTH_WORKSPACE"
        log_error "Atual: $(pwd)"
        return 1
    fi
    return 0
}

# Verificar dependências básicas
check_basic_dependencies() {
    local dependencies=("git" "python3" "colcon" "pigpiod")
    local missing=()
    
    for dep in "${dependencies[@]}"; do
        if ! command -v "$dep" >/dev/null 2>&1; then
            missing+=("$dep")
        fi
    done
    
    if [[ ${#missing[@]} -eq 0 ]]; then
        return 0
    else
        log_error "Dependências faltando: ${missing[*]}"
        return 1
    fi
}

# Verificação completa de pré-requisitos
check_prerequisites() {
    local errors=0
    
    log_info "Verificando pré-requisitos do sistema..."
    
    # ROS2
    if check_ros2_available; then
        log_success "ROS2 disponível"
    else
        log_error "ROS2 não encontrado"
        ((errors++))
    fi
    
    # Workspace directory
    if check_workspace_directory; then
        log_success "Diretório workspace correto"
    else
        ((errors++))
    fi
    
    # pigpiod
    if check_pigpiod_running; then
        log_success "pigpiod rodando"
    else
        log_warning "pigpiod não está rodando"
        log_info "Tentando iniciar pigpiod..."
        if sudo systemctl start pigpiod 2>/dev/null; then
            sleep 2
            if check_pigpiod_running; then
                log_success "pigpiod iniciado com sucesso"
            else
                log_error "Falha ao iniciar pigpiod"
                ((errors++))
            fi
        else
            log_error "Não foi possível iniciar pigpiod"
            ((errors++))
        fi
    fi
    
    # User groups
    if check_user_groups; then
        log_success "Grupos do usuário corretos"
    else
        log_warning "Alguns grupos do usuário podem estar faltando"
        # Não conta como erro crítico
    fi
    
    # Workspace built
    if check_workspace_built; then
        log_success "Workspace compilado"
    else
        log_warning "Workspace não compilado"
        log_info "Execute: bash scripts/build/build_system.sh"
        # Não conta como erro crítico para scripts de build
    fi
    
    return $errors
}

# =============================================================================
# CONFIGURAÇÃO DE AMBIENTE
# =============================================================================

# Configurar ambiente ROS2
setup_ros2_environment() {
    log_debug "Configurando ambiente ROS2..."
    
    # Definir todas as variáveis necessárias para evitar erros
    export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
    export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-/usr/bin/python3}"
    export COLCON_TRACE="${COLCON_TRACE:-}"
    export COLCON_PYTHON_EXECUTABLE="${COLCON_PYTHON_EXECUTABLE:-/usr/bin/python3}"
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
    
    # Source ROS2 base com proteção de erro
    if [[ -f "$ROS2_SETUP_PATH" ]]; then
        set +u  # Temporariamente desabilitar verificação de unbound variables
        source "$ROS2_SETUP_PATH" 2>/dev/null || {
            log_error "Erro ao carregar ROS2"
            set -u
            return 1
        }
        set -u  # Reabilitar verificação
        log_debug "ROS2 base carregado"
    else
        log_error "ROS2 setup não encontrado: $ROS2_SETUP_PATH"
        return 1
    fi
    
    # Source workspace se existir
    if [[ -f "$F1TENTH_WORKSPACE_SETUP" ]]; then
        set +u
        source "$F1TENTH_WORKSPACE_SETUP" 2>/dev/null || {
            log_debug "Workspace F1TENTH não carregado completamente"
        }
        set -u
        log_debug "Workspace F1TENTH carregado"
    else
        log_debug "Workspace F1TENTH não compilado ainda"
    fi
    
    return 0
}

# Navegar para workspace
navigate_to_workspace() {
    if [[ ! -d "$F1TENTH_WORKSPACE" ]]; then
        log_error "Workspace não existe: $F1TENTH_WORKSPACE"
        return 1
    fi
    
    if [[ "$(pwd)" != "$F1TENTH_WORKSPACE" ]]; then
        log_debug "Navegando para workspace: $F1TENTH_WORKSPACE"
        cd "$F1TENTH_WORKSPACE" || {
            log_error "Falha ao navegar para workspace"
            return 1
        }
    fi
    
    return 0
}

# =============================================================================
# UTILITÁRIOS DE BUILD
# =============================================================================

# Limpeza de build anterior
clean_build() {
    local directories=("build" "install" "log")
    
    log_info "Limpando build anterior..."
    for dir in "${directories[@]}"; do
        if [[ -d "$dir" ]]; then
            rm -rf "$dir"
            log_debug "Removido: $dir/"
        fi
    done
}

# Verificar se pacotes foram instalados corretamente
verify_packages_installed() {
    local packages=("f1tenth_control" "joy_converter" "vesc_msgs" "vesc_driver" "vesc_ackermann")
    local missing=()
    
    log_info "Verificando pacotes instalados..."
    
    for package in "${packages[@]}"; do
        if [[ -d "install/$package" ]]; then
            log_success "Pacote $package: OK"
        else
            log_error "Pacote $package: FALTANDO"
            missing+=("$package")
        fi
    done
    
    if [[ ${#missing[@]} -eq 0 ]]; then
        return 0
    else
        log_error "Pacotes faltando: ${missing[*]}"
        return 1
    fi
}

# Verificar executáveis ROS2
verify_ros2_executables() {
    local package="f1tenth_control"
    local executables=("servo_control_node" "enhanced_servo_control_node")
    
    log_info "Verificando executáveis ROS2..."
    
    if ! ros2 pkg list | grep -q "$package"; then
        log_error "Pacote $package não reconhecido pelo ROS2"
        return 1
    fi
    
    for exe in "${executables[@]}"; do
        if ros2 pkg executables "$package" | grep -q "$exe"; then
            log_success "Executável $exe: OK"
        else
            log_warning "Executável $exe: NÃO ENCONTRADO"
        fi
    done
    
    return 0
}

# =============================================================================
# UTILITÁRIOS DE TESTE
# =============================================================================

# Verificar se joystick está conectado
check_joystick_connected() {
    if ls /dev/input/js* >/dev/null 2>&1; then
        local joystick_dev=$(ls /dev/input/js* | head -1)
        log_success "Joystick detectado: $joystick_dev"
        
        # Verificar permissões
        if [[ -r "$joystick_dev" && -w "$joystick_dev" ]]; then
            log_success "Permissões joystick OK"
            return 0
        else
            log_error "Permissões insuficientes para $joystick_dev"
            log_info "Execute: sudo chmod 666 $joystick_dev"
            return 1
        fi
    else
        log_error "Nenhum joystick detectado"
        log_info "Conecte um joystick ou execute: bash scripts/test/detect_controller.sh"
        return 1
    fi
}

# Aguardar nó ROS2 ficar ativo
wait_for_ros2_node() {
    local node_name="$1"
    local timeout="${2:-$F1TENTH_SERVICE_WAIT}"
    local count=0
    
    log_info "Aguardando nó $node_name (timeout: ${timeout}s)..."
    
    while [[ $count -lt $timeout ]]; do
        if ros2 node list 2>/dev/null | grep -q "$node_name"; then
            log_success "Nó $node_name ativo"
            return 0
        fi
        sleep 1
        ((count++))
    done
    
    log_error "Timeout aguardando nó $node_name"
    return 1
}

# Verificar se tópico está publicando
check_topic_publishing() {
    local topic="$1"
    local timeout="${2:-5}"
    
    log_info "Verificando tópico $topic..."
    
    if timeout "${timeout}s" ros2 topic echo "$topic" --once >/dev/null 2>&1; then
        log_success "Tópico $topic publicando"
        return 0
    else
        log_error "Tópico $topic não está publicando"
        return 1
    fi
}

# =============================================================================
# UTILITÁRIOS DE PROCESSO
# =============================================================================

# Executar comando com timeout e logging
execute_with_timeout() {
    local timeout="$1"
    local description="$2"
    shift 2
    local command=("$@")
    
    log_info "Executando: $description"
    log_debug "Comando: ${command[*]}"
    
    if timeout "${timeout}s" "${command[@]}"; then
        log_success "$description concluído"
        return 0
    else
        log_error "$description falhou ou timeout"
        return 1
    fi
}

# Cleanup de processos em background
cleanup_background_processes() {
    local pids=("$@")
    
    for pid in "${pids[@]}"; do
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            log_debug "Terminando processo: $pid"
            kill "$pid" 2>/dev/null || true
            sleep 1
            if kill -0 "$pid" 2>/dev/null; then
                log_debug "Forçando término do processo: $pid"
                kill -9 "$pid" 2>/dev/null || true
            fi
        fi
    done
}

# =============================================================================
# UTILITÁRIOS DE HARDWARE
# =============================================================================

# Verificar dispositivos USB
check_usb_devices() {
    log_info "Verificando dispositivos USB..."
    
    # VESC
    if lsusb | grep -i "silabs\|cp210x" >/dev/null; then
        log_success "VESC USB detectado"
    else
        log_warning "VESC USB não detectado"
    fi
    
    # LiDAR
    if ls /dev/ttyUSB* >/dev/null 2>&1; then
        log_success "Dispositivos USB serial disponíveis: $(ls /dev/ttyUSB*)"
    else
        log_warning "Nenhum dispositivo USB serial detectado"
    fi
}

# =============================================================================
# FUNÇÕES DE INICIALIZAÇÃO
# =============================================================================

# Inicialização padrão das funções comuns
initialize_common_functions() {
    # Garantir diretório de logs
    ensure_log_directory
    
    # Log de inicialização
    log_debug "F1TENTH Common Functions v2.0.0 carregadas"
    log_debug "Usuário: $F1TENTH_USER"
    log_debug "Workspace: $F1TENTH_WORKSPACE"
    log_debug "Log dir: $F1TENTH_LOG_DIR"
    log_debug "Build jobs: $F1TENTH_BUILD_JOBS"
    
    return 0
}

# =============================================================================
# AUTO-INICIALIZAÇÃO
# =============================================================================

# Executar inicialização automaticamente quando sourced
initialize_common_functions

# Export de funções principais para uso global
export -f log_info log_success log_error log_warning log_debug
export -f check_ros2_available check_pigpiod_running check_workspace_built
export -f check_prerequisites setup_ros2_environment navigate_to_workspace

log_debug "F1TENTH Common Functions inicializadas com sucesso" 