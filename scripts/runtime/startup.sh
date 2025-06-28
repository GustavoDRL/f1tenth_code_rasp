#!/bin/bash
# =============================================================================
# F1TENTH Startup Script - Inicialização Automática do Sistema
# Versão: 2.1.0
# Descrição: Script robusto para inicialização automática via systemd,
#            com lógica de repetição, validação e gerenciamento de processo.
# =============================================================================

# Proteções e configuração inicial
set -euo pipefail
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Carregar funções comuns e configurações
source "$SCRIPT_DIR/../utils/common_functions.sh"
source "$SCRIPT_DIR/../utils/config_manager.sh"

# =============================================================================
# CONFIGURAÇÕES DE STARTUP
# =============================================================================

# Configurações padrão (podem ser sobrescritas por variáveis de ambiente)
STARTUP_MODE="${F1TENTH_STARTUP_MODE:-minimal}" # complete|minimal|servo-only
AUTO_BUILD="${F1TENTH_AUTO_BUILD:-false}"
BUILD_TIMEOUT="${F1TENTH_BUILD_TIMEOUT:-300}"      # 5 minutos
STARTUP_DELAY="${F1TENTH_STARTUP_DELAY:-5}"        # 5 segundos
VALIDATION_TIMEOUT="${F1TENTH_VALIDATION_TIMEOUT:-20}" # 20 segundos
MAX_STARTUP_ATTEMPTS="${F1TENTH_MAX_STARTUP_ATTEMPTS:-3}"

# Estado interno
ROS_LAUNCH_PID=""
STARTUP_SUCCESS=false

# =============================================================================
# PROCESSAMENTO DE ARGUMENTOS
# =============================================================================

usage() {
    cat << EOF
🚀 F1TENTH Startup Script v2.1.0

USAGE:
    $0 [MODE] [OPTIONS]

MODES:
    minimal                   Sistema mínimo (servo + motor) [default]
    complete                  Sistema completo (com LiDAR)
    servo-only                Apenas controle do servo
    
OPTIONS:
    --auto-build              Executa build automático se o workspace não estiver compilado.
    --build-timeout SECONDS   Timeout para o build (default: $BUILD_TIMEOUT).
    --delay SECONDS           Delay inicial antes do primeiro lançamento (default: $STARTUP_DELAY).
    --validation-timeout SECONDS Timeout para validação dos nós ROS2 (default: $VALIDATION_TIMEOUT).
    --max-attempts N          Máximo de tentativas de inicialização (default: $MAX_STARTUP_ATTEMPTS).
    -h, --help                Mostrar esta ajuda.

EXAMPLES:
    $0                        # Startup padrão (modo minimal).
    $0 complete               # Inicia o sistema completo com LiDAR.
    $0 --auto-build           # Habilita o build automático antes de iniciar.
    systemctl start f1tenth   # Exemplo de uso via systemd.

EOF
}

# Processar argumentos
while [[ $# -gt 0 ]]; do
    case $1 in
        complete|minimal|servo-only)
            STARTUP_MODE="$1"
            shift
            ;;
        --auto-build)
            AUTO_BUILD=true
            shift
            ;;
        --build-timeout)
            BUILD_TIMEOUT="$2"
            shift 2
            ;;
        --delay)
            STARTUP_DELAY="$2"
            shift 2
            ;;
        --validation-timeout)
            VALIDATION_TIMEOUT="$2"
            shift 2
            ;;
        --max-attempts)
            MAX_STARTUP_ATTEMPTS="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            f1tenth_log_error "Argumento desconhecido: $1"
            usage
            exit 1
            ;;
    esac
done

# =============================================================================
# FUNÇÕES DE STARTUP
# =============================================================================

validate_startup_environment() {
    f1tenth_log_info "Validando ambiente de startup..."
    
    if [[ ! -f "$WORKSPACE_ROOT/src/f1tenth_control/package.xml" ]]; then
        f1tenth_log_error "Workspace F1TENTH não encontrado em: $WORKSPACE_ROOT"
        exit 1
    fi
    
    if ! command -v ros2 >/dev/null 2>&1; then
        f1tenth_log_error "Comando 'ros2' não encontrado. Faça o source do ambiente ROS2."
        exit 1
    fi
    
    if [[ ! -f "$WORKSPACE_ROOT/install/setup.bash" ]]; then
        if [[ "$AUTO_BUILD" == "true" ]]; then
            f1tenth_log_warn "Workspace não compilado. Iniciando build automático..."
            execute_auto_build
        else
            f1tenth_log_error "Workspace não compilado! Execute ./scripts/build/master_build.sh ou use --auto-build."
            exit 1
        fi
    fi
    
    f1tenth_log_success "Ambiente validado."
}

execute_auto_build() {
    f1tenth_log_info "Executando build automático com timeout de ${BUILD_TIMEOUT}s..."
    local build_script="$SCRIPT_DIR/../build/master_build.sh"
    
    if [[ ! -x "$build_script" ]]; then
        f1tenth_log_error "Script de build não encontrado ou não executável: $build_script"
        exit 1
    fi
    
    if ! timeout "$BUILD_TIMEOUT" "$build_script" --quick --no-test; then
        f1tenth_log_error "Build automático falhou ou excedeu o timeout."
        exit 1
    fi
    f1tenth_log_success "Build automático concluído."
}

setup_runtime_environment() {
    f1tenth_log_info "Configurando ambiente de runtime..."
    
    f1tenth_source_ros2
    source "$WORKSPACE_ROOT/install/setup.bash"
    export RMW_IMPLEMENTATION="${F1TENTH_RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
    
    cd "$WORKSPACE_ROOT"
    configure_hardware_services
    
    f1tenth_log_success "Ambiente de runtime configurado."
}

configure_hardware_services() {
    f1tenth_log_info "Verificando e configurando serviços de hardware..."
    
    if command -v pigpiod >/dev/null 2>&1; then
        if ! pgrep -x "pigpiod" > /dev/null; then
            f1tenth_log_info "Iniciando serviço 'pigpiod'..."
            if sudo pigpiod; then
                f1tenth_log_success "'pigpiod' iniciado."
            else
                f1tenth_log_warn "Falha ao iniciar 'pigpiod'. O controle do servo pode falhar."
            fi
        else
            f1tenth_log_info "'pigpiod' já está ativo."
        fi
    else
        f1tenth_log_warn "'pigpiod' não encontrado. O controle do servo não funcionará."
    fi
}

select_launch_file() {
    local launch_file=""
    case "$STARTUP_MODE" in
        "complete")   launch_file="f1tenth_complete_system.launch.py";;
        "minimal")    launch_file="f1tenth_system_no_lidar.launch.py";;
        "servo-only") launch_file="f1tenth_servo_only.launch.py";;
        *)
            f1tenth_log_error "Modo de startup inválido: $STARTUP_MODE"
            exit 1
            ;;
    esac
    
    local launch_path
    launch_path=$(ros2 pkg prefix f1tenth_control)/share/f1tenth_control/launch/$launch_file
    if [[ ! -f "$launch_path" ]]; then
        f1tenth_log_error "Arquivo de lançamento não encontrado: $launch_file"
        f1tenth_log_info "Verifique se o workspace foi compilado corretamente."
        exit 1
    fi
    echo "$launch_file"
}

attempt_system_startup() {
    local launch_file
    launch_file=$(select_launch_file)
    
    f1tenth_log_info "Iniciando: ros2 launch f1tenth_control $launch_file"
    
    # Executa o launch em segundo plano para que o script possa monitorá-lo
    ros2 launch f1tenth_control "$launch_file" &
    ROS_LAUNCH_PID=$!
    f1tenth_log_info "Processo de lançamento iniciado com PID: $ROS_LAUNCH_PID"
}

validate_system_startup() {
    f1tenth_log_info "Validando startup do sistema por até ${VALIDATION_TIMEOUT}s..."
    local elapsed=0
    
    while [[ $elapsed -lt $VALIDATION_TIMEOUT ]]; do
        # Verifica se o processo de launch ainda está vivo
        if ! ps -p "$ROS_LAUNCH_PID" > /dev/null; then
            f1tenth_log_error "Processo de lançamento (PID: $ROS_LAUNCH_PID) terminou prematuramente."
            return 1
        fi
        
        # Verifica se há nós ativos (mais de 1, pois o launch pode criar um nó próprio)
        local active_nodes
        active_nodes=$(ros2 node list 2>/dev/null | wc -l || echo "0")
        if [[ $active_nodes -gt 1 ]]; then
            f1tenth_log_success "Sistema validado com $active_nodes nós ativos."
            return 0
        fi
        
        sleep 1
        ((elapsed++))
    done
    
    f1tenth_log_error "Timeout de validação. Nenhum nó ROS2 funcional detectado."
    return 1
}

handle_startup_failure() {
    local attempt_num=$1
    f1tenth_log_error "Falha no startup (tentativa $attempt_num/$MAX_STARTUP_ATTEMPTS)"
    
    if [[ -n "$ROS_LAUNCH_PID" ]]; then
        f1tenth_log_info "Limpando processo de launch falho (PID: $ROS_LAUNCH_PID)..."
        # Mata o grupo de processos para encerrar todos os filhos
        kill -SIGTERM -- "-$ROS_LAUNCH_PID" 2>/dev/null || true
        sleep 2
        kill -SIGKILL -- "-$ROS_LAUNCH_PID" 2>/dev/null || true
        ROS_LAUNCH_PID=""
    fi
    
    cleanup_ros_processes # Fallback para garantir limpeza
}

cleanup_ros_processes() {
    f1tenth_log_info "Executando limpeza de processos ROS2 remanescentes..."
    # Abordagem mais direcionada para evitar matar processos não relacionados
    pkill -f "launch.main.py" 2>/dev/null || true
    pkill -f "f1tenth_control" 2>/dev/null || true
    sleep 1
    f1tenth_log_info "Cleanup concluído."
}

# =============================================================================
# TRATAMENTO DE SINAIS
# =============================================================================

graceful_shutdown() {
    f1tenth_log_info "Recebido sinal de shutdown. Encerrando F1TENTH..."
    
    if [[ -n "$ROS_LAUNCH_PID" ]]; then
        f1tenth_log_info "Enviando SIGTERM para o grupo de processos (PID: $ROS_LAUNCH_PID)..."
        kill -SIGTERM -- "-$ROS_LAUNCH_PID" 2>/dev/null || true
        sleep 2
    fi
    
    cleanup_ros_processes
    
    f1tenth_log_info "F1TENTH startup finalizado."
    exit 0
}

trap graceful_shutdown SIGTERM SIGINT

# =============================================================================
# FUNÇÃO PRINCIPAL
# =============================================================================

main() {
    f1tenth_log_info "F1TENTH Startup v2.1.0 - Modo: $STARTUP_MODE"
    
    validate_startup_environment
    setup_runtime_environment
    
    if [[ $STARTUP_DELAY -gt 0 ]]; then
        f1tenth_log_info "Aguardando ${STARTUP_DELAY}s antes da primeira tentativa..."
        sleep "$STARTUP_DELAY"
    fi
    
    local current_attempt=1
    while [[ $current_attempt -le $MAX_STARTUP_ATTEMPTS ]]; do
        f1tenth_log_info "--- Tentativa de Startup $current_attempt/$MAX_STARTUP_ATTEMPTS ---"
        
        attempt_system_startup
        
        if validate_system_startup; then
            STARTUP_SUCCESS=true
            break
        fi
        
        handle_startup_failure "$current_attempt"
        
        if [[ $current_attempt -lt $MAX_STARTUP_ATTEMPTS ]]; then
            f1tenth_log_info "Aguardando 5s para a próxima tentativa..."
            sleep 5
        fi
        ((current_attempt++))
    done
    
    if [[ "$STARTUP_SUCCESS" == "true" ]]; then
        f1tenth_log_success "F1TENTH sistema iniciado com sucesso! Monitorando processo (PID: $ROS_LAUNCH_PID)..."
        
        wait "$ROS_LAUNCH_PID"
        local exit_code=$?
        f1tenth_log_info "Processo ROS2 (PID: $ROS_LAUNCH_PID) finalizado com código de saída: $exit_code."
        exit $exit_code
    else
        f1tenth_log_error "Falha ao iniciar o sistema F1TENTH após $MAX_STARTUP_ATTEMPTS tentativas."
        exit 1
    fi
}

# =============================================================================
# EXECUÇÃO
# =============================================================================

main "$@" 