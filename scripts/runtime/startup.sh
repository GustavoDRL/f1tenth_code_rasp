#!/bin/bash
# =============================================================================
# F1TENTH Startup Script - Inicialização Automática do Sistema
# Versão: 2.0.0
# Descrição: Script otimizado para inicialização automática via systemd
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
STARTUP_MODE="${F1TENTH_STARTUP_MODE:-complete}"  # complete|minimal|servo-only
AUTO_BUILD="${F1TENTH_AUTO_BUILD:-false}"
BUILD_TIMEOUT="${F1TENTH_BUILD_TIMEOUT:-300}"      # 5 minutos
STARTUP_DELAY="${F1TENTH_STARTUP_DELAY:-5}"        # 5 segundos
MAX_STARTUP_ATTEMPTS="${F1TENTH_MAX_STARTUP_ATTEMPTS:-3}"

# Estado interno
CURRENT_ATTEMPT=1
STARTUP_SUCCESS=false

# =============================================================================
# PROCESSAMENTO DE ARGUMENTOS
# =============================================================================

usage() {
    cat << EOF
🚀 F1TENTH Startup Script v2.0.0

USAGE:
    $0 [MODE] [OPTIONS]

MODES:
    complete                  Sistema completo (default)
    minimal                   Sistema mínimo (sem LiDAR)
    servo-only                Apenas controle servo
    
OPTIONS:
    --auto-build              Build automático se necessário
    --build-timeout SECONDS   Timeout para build (default: $BUILD_TIMEOUT)
    --delay SECONDS           Delay inicial (default: $STARTUP_DELAY)
    --max-attempts N          Máximo tentativas (default: $MAX_STARTUP_ATTEMPTS)
    -h, --help                Mostrar esta ajuda

EXAMPLES:
    $0                        # Startup completo padrão
    $0 minimal                # Sistema sem LiDAR
    $0 --auto-build           # Com build automático
    systemctl start f1tenth  # Via systemd

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
    
    # Verificar workspace
    if [[ ! -f "$WORKSPACE_ROOT/src/f1tenth_control/package.xml" ]]; then
        f1tenth_log_error "Workspace F1TENTH não encontrado!"
        f1tenth_log_info "Workspace esperado: $WORKSPACE_ROOT"
        exit 1
    fi
    
    # Verificar se ROS2 está disponível
    if ! command -v ros2 >/dev/null 2>&1; then
        f1tenth_log_error "ROS2 não encontrado no PATH"
        f1tenth_log_info "Execute: source /opt/ros/humble/setup.bash"
        exit 1
    fi
    
    # Verificar se workspace foi compilado
    if [[ ! -f "$WORKSPACE_ROOT/install/setup.bash" ]]; then
        if [[ "$AUTO_BUILD" == "true" ]]; then
            f1tenth_log_warn "Workspace não compilado. Iniciando build automático..."
            execute_auto_build
        else
            f1tenth_log_error "Workspace não compilado!"
            f1tenth_log_info "Execute: ./scripts/build/master_build.sh"
            f1tenth_log_info "Ou use: $0 --auto-build"
            exit 1
        fi
    fi
    
    f1tenth_log_success "Ambiente validado"
}

execute_auto_build() {
    f1tenth_log_info "Executando build automático..."
    
    local build_script="$SCRIPT_DIR/../build/master_build.sh"
    
    if [[ ! -x "$build_script" ]]; then
        f1tenth_log_error "Script de build não encontrado: $build_script"
        exit 1
    fi
    
    # Executar build com timeout
    if timeout "$BUILD_TIMEOUT" "$build_script" --quick --no-test; then
        f1tenth_log_success "Build automático concluído"
    else
        f1tenth_log_error "Build automático falhou ou timeout ($BUILD_TIMEOUT s)"
        exit 1
    fi
}

setup_runtime_environment() {
    f1tenth_log_info "Configurando ambiente de runtime..."
    
    # Source ROS2
    f1tenth_source_ros2
    
    # Source workspace
    source "$WORKSPACE_ROOT/install/setup.bash"
    
    # Configurar middleware
    export RMW_IMPLEMENTATION="$F1TENTH_RMW_IMPLEMENTATION"
    
    # Entrar no workspace
    cd "$WORKSPACE_ROOT"
    
    # Verificar e configurar serviços de hardware
    configure_hardware_services
    
    f1tenth_log_success "Ambiente de runtime configurado"
}

configure_hardware_services() {
    f1tenth_log_info "Configurando serviços de hardware..."
    
    # pigpiod
    if command -v pigpiod >/dev/null 2>&1; then
        if ! systemctl is-active --quiet pigpiod; then
            f1tenth_log_info "Iniciando pigpiod..."
            if sudo systemctl start pigpiod; then
                f1tenth_log_success "pigpiod iniciado"
            else
                f1tenth_log_warn "Falha ao iniciar pigpiod"
            fi
        else
            f1tenth_log_info "pigpiod já ativo"
        fi
    else
        f1tenth_log_warn "pigpiod não instalado"
    fi
    
    # Verificar permissões
    local current_groups
    current_groups=$(groups)
    
    if ! echo "$current_groups" | grep -q "gpio"; then
        f1tenth_log_warn "Usuário não está no grupo gpio"
    fi
    
    if ! echo "$current_groups" | grep -q "dialout"; then
        f1tenth_log_warn "Usuário não está no grupo dialout"
    fi
}

select_launch_configuration() {
    local launch_file=""
    
    case "$STARTUP_MODE" in
        "complete")
            launch_file="f1tenth_complete_system.launch.py"
            f1tenth_log_info "Modo: Sistema completo (servo + motor + LiDAR)"
            ;;
        "minimal")
            launch_file="f1tenth_system_no_lidar.launch.py"
            f1tenth_log_info "Modo: Sistema mínimo (servo + motor)"
            ;;
        "servo-only")
            launch_file="f1tenth_servo_only.launch.py"
            f1tenth_log_info "Modo: Apenas servo"
            ;;
        *)
            f1tenth_log_error "Modo de startup inválido: $STARTUP_MODE"
            exit 1
            ;;
    esac
    
    # Verificar se launch file existe
    local launch_path="$WORKSPACE_ROOT/install/f1tenth_control/share/f1tenth_control/launch/$launch_file"
    
    if [[ ! -f "$launch_path" ]]; then
        f1tenth_log_error "Launch file não encontrado: $launch_file"
        f1tenth_log_info "Execute build: ./scripts/build/master_build.sh"
        exit 1
    fi
    
    echo "$launch_file"
}

attempt_system_startup() {
    local launch_file
    launch_file=$(select_launch_configuration)
    
    f1tenth_log_info "Tentativa $CURRENT_ATTEMPT/$MAX_STARTUP_ATTEMPTS"
    f1tenth_log_info "Iniciando: $launch_file"
    
    # Delay inicial se especificado
    if [[ $STARTUP_DELAY -gt 0 ]]; then
        f1tenth_log_info "Aguardando ${STARTUP_DELAY}s antes do startup..."
        sleep "$STARTUP_DELAY"
    fi
    
    # Iniciar sistema
    f1tenth_log_info "Executando ros2 launch..."
    
    # Usar exec para substituir o processo atual pelo launch
    # Isso é importante para systemd gerenciar corretamente o serviço
    exec ros2 launch f1tenth_control "$launch_file"
}

validate_system_startup() {
    f1tenth_log_info "Validando startup do sistema..."
    
    # Aguardar sistema inicializar
    sleep 10
    
    # Verificar nós ativos
    local active_nodes
    active_nodes=$(ros2 node list 2>/dev/null | wc -l || echo "0")
    
    if [[ $active_nodes -gt 0 ]]; then
        f1tenth_log_success "Sistema iniciado com $active_nodes nós"
        STARTUP_SUCCESS=true
        return 0
    else
        f1tenth_log_error "Nenhum nó ROS2 detectado"
        return 1
    fi
}

handle_startup_failure() {
    f1tenth_log_error "Falha no startup (tentativa $CURRENT_ATTEMPT)"
    
    # Cleanup de processos
    cleanup_ros_processes
    
    if [[ $CURRENT_ATTEMPT -lt $MAX_STARTUP_ATTEMPTS ]]; then
        ((CURRENT_ATTEMPT++))
        f1tenth_log_info "Tentando novamente em 5 segundos..."
        sleep 5
        attempt_system_startup
    else
        f1tenth_log_error "Máximo de tentativas atingido. Startup falhou."
        exit 1
    fi
}

cleanup_ros_processes() {
    f1tenth_log_info "Limpando processos ROS2..."
    
    # Matar processos ROS2 remanescentes
    pkill -f "ros2" 2>/dev/null || true
    pkill -f "f1tenth" 2>/dev/null || true
    
    # Aguardar cleanup
    sleep 2
    
    f1tenth_log_info "Cleanup concluído"
}

# =============================================================================
# TRATAMENTO DE SINAIS
# =============================================================================

# Função para shutdown graceful
graceful_shutdown() {
    f1tenth_log_info "Recebido sinal de shutdown..."
    
    # Cleanup
    cleanup_ros_processes
    
    f1tenth_log_info "F1TENTH startup finalizado"
    exit 0
}

# Registrar handlers de sinal
trap graceful_shutdown SIGTERM SIGINT

# =============================================================================
# FUNÇÃO PRINCIPAL
# =============================================================================

main() {
    # Não usar print_header aqui para não interferir com systemd logs
    f1tenth_log_info "F1TENTH Startup v2.0.0 - Modo: $STARTUP_MODE"
    
    # Validações e configuração
    validate_startup_environment
    setup_runtime_environment
    
    # Loop de tentativas de startup
    while [[ $CURRENT_ATTEMPT -le $MAX_STARTUP_ATTEMPTS ]] && [[ "$STARTUP_SUCCESS" == "false" ]]; do
        if attempt_system_startup; then
            validate_system_startup
        else
            handle_startup_failure
        fi
    done
    
    if [[ "$STARTUP_SUCCESS" == "true" ]]; then
        f1tenth_log_success "F1TENTH sistema iniciado com sucesso!"
        
        # Manter o script rodando para systemd
        while true; do
            sleep 30
            # Verificar se sistema ainda está rodando
            if [[ $(ros2 node list 2>/dev/null | wc -l) -eq 0 ]]; then
                f1tenth_log_error "Sistema ROS2 parou inesperadamente"
                exit 1
            fi
        done
    else
        f1tenth_log_error "Falha ao iniciar sistema F1TENTH"
        exit 1
    fi
}

# =============================================================================
# EXECUÇÃO
# =============================================================================

# Executar apenas se script for chamado diretamente
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi 