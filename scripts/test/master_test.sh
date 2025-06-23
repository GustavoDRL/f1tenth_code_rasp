#!/bin/bash
# =============================================================================
# F1TENTH Master Test Script - Testes Consolidados e Otimizados  
# Versão: 2.0.0
# Descrição: Script mestre que consolida todos os testes do sistema F1TENTH
# =============================================================================

# Proteções e configuração inicial
set -euo pipefail
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Carregar funções comuns e configurações
source "$SCRIPT_DIR/../utils/common_functions.sh"
source "$SCRIPT_DIR/../utils/config_manager.sh"

# =============================================================================
# CONFIGURAÇÕES DOS TESTES
# =============================================================================

# Tipos de teste disponíveis
readonly TEST_TYPE_SERVO="servo"
readonly TEST_TYPE_JOYSTICK="joystick"
readonly TEST_TYPE_SYSTEM="system"
readonly TEST_TYPE_HARDWARE="hardware"
readonly TEST_TYPE_ALL="all"

# Parâmetros dos testes (podem ser sobrescritos por argumentos)
TEST_TYPE="${F1TENTH_TEST_TYPE:-$TEST_TYPE_SERVO}"
TEST_DURATION="${F1TENTH_TEST_DURATION:-10}"
VERBOSE_MODE="${F1TENTH_DEBUG_MODE}"
INTERACTIVE_MODE="${F1TENTH_INTERACTIVE_MODE:-true}"
LOG_TESTS="${F1TENTH_LOG_TESTS:-true}"

# Diretórios e arquivos de log
readonly LOG_DIR="$WORKSPACE_ROOT/logs/tests"
readonly TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
readonly LOG_FILE="$LOG_DIR/f1tenth_test_$TIMESTAMP.log"

# =============================================================================
# PROCESSAMENTO DE ARGUMENTOS
# =============================================================================

usage() {
    cat << EOF
🧪 F1TENTH Master Test Script v2.0.0

USAGE:
    $0 [TEST_TYPE] [OPTIONS]

TEST TYPES:
    servo                     Teste básico do servo GPIO (default)
    joystick                  Teste controle manual com joystick
    system                    Teste sistema ROS2 completo
    hardware                  Teste dependências de hardware
    all                       Executar todos os testes

OPTIONS:
    -d, --duration SECONDS    Duração testes interativos (default: $TEST_DURATION)
    -v, --verbose             Modo verboso
    -q, --quiet               Modo silencioso (sem interação)
    --no-log                  Não salvar logs
    -h, --help                Mostrar esta ajuda

EXAMPLES:
    $0                        # Teste servo básico
    $0 joystick               # Teste controle joystick
    $0 system --verbose       # Teste sistema completo verboso
    $0 all --duration 30      # Todos testes com 30s cada

EOF
}

# Processar argumentos
while [[ $# -gt 0 ]]; do
    case $1 in
        servo|joystick|system|hardware|all)
            TEST_TYPE="$1"
            shift
            ;;
        -d|--duration)
            TEST_DURATION="$2"
            shift 2
            ;;
        -v|--verbose)
            VERBOSE_MODE=true
            shift
            ;;
        -q|--quiet)
            INTERACTIVE_MODE=false
            shift
            ;;
        --no-log)
            LOG_TESTS=false
            shift
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
# CONFIGURAÇÃO E VALIDAÇÃO INICIAL
# =============================================================================

setup_test_environment() {
    f1tenth_log_info "Configurando ambiente de testes..."
    
    # Validar workspace
    if [[ ! -f "$WORKSPACE_ROOT/src/f1tenth_control/package.xml" ]]; then
        f1tenth_log_error "Workspace F1TENTH inválido!"
        exit 1
    fi
    
    # Source ROS2 e workspace
    f1tenth_source_ros2
    if [[ -f "$WORKSPACE_ROOT/install/setup.bash" ]]; then
        source "$WORKSPACE_ROOT/install/setup.bash"
    else
        f1tenth_log_error "Workspace não compilado. Execute: ./scripts/build/master_build.sh"
        exit 1
    fi
    
    # Configurar logs
    if [[ "$LOG_TESTS" == "true" ]]; then
        mkdir -p "$LOG_DIR"
        f1tenth_log_info "Logs salvos em: $LOG_FILE"
        # Redirecionar stdout e stderr para arquivo
        exec > >(tee -a "$LOG_FILE") 2>&1
    fi
    
    # Entrar no workspace
    cd "$WORKSPACE_ROOT"
    
    f1tenth_log_success "Ambiente configurado"
}

validate_hardware_requirements() {
    f1tenth_log_info "Validando requisitos de hardware..."
    
    # Verificar se estamos no Raspberry Pi
    if [[ ! -d "/sys/class/gpio" ]]; then
        f1tenth_log_warn "GPIO não detectado - pode não estar no Raspberry Pi"
    fi
    
    # Verificar pigpiod
    if ! systemctl is-active --quiet pigpiod; then
        f1tenth_log_warn "pigpiod inativo. Tentando iniciar..."
        if ! sudo systemctl start pigpiod; then
            f1tenth_log_error "Falha ao iniciar pigpiod"
            return 1
        fi
    fi
    
    # Verificar grupos do usuário
    local missing_groups=()
    
    if ! groups "$USER" | grep -q "gpio"; then
        missing_groups+=("gpio")
    fi
    
    if ! groups "$USER" | grep -q "dialout"; then
        missing_groups+=("dialout")
    fi
    
    if [[ ${#missing_groups[@]} -gt 0 ]]; then
        f1tenth_log_warn "Grupos faltando: ${missing_groups[*]}"
        f1tenth_log_info "Execute: sudo usermod -a -G ${missing_groups[*]} $USER && logout"
    fi
    
    f1tenth_log_success "Validação de hardware concluída"
}

# =============================================================================
# FUNÇÕES DE TESTE ESPECÍFICAS
# =============================================================================

test_servo_basic() {
    f1tenth_log_info "TESTE SERVO BÁSICO"
    f1tenth_separator
    
    # Testar node servo_control_node
    if ! command -v ros2 >/dev/null; then
        f1tenth_log_error "ROS2 não encontrado"
        return 1
    fi
    
    f1tenth_log_info "Iniciando servo_control_node..."
    
    # Iniciar node em background
    ros2 run f1tenth_control servo_control_node > /dev/null 2>&1 &
    local servo_pid=$!
    sleep 3
    
    # Verificar se node está rodando
    if ! kill -0 $servo_pid 2>/dev/null; then
        f1tenth_log_error "Falha ao iniciar servo_control_node"
        return 1
    fi
    
    f1tenth_log_success "servo_control_node iniciado (PID: $servo_pid)"
    
    # Verificar tópicos
    if timeout 5s ros2 topic list | grep -q "/servo"; then
        f1tenth_log_success "Tópicos servo detectados"
    else
        f1tenth_log_warn "Tópicos servo não encontrados"
    fi
    
    # Teste interativo se habilitado
    if [[ "$INTERACTIVE_MODE" == "true" ]]; then
        f1tenth_log_info "Publicando comandos de teste..."
        
        # Teste sequência de movimentos
        local angles=(-0.3 0.0 0.3 0.0)
        for angle in "${angles[@]}"; do
            f1tenth_log_info "Testando ângulo: $angle"
            ros2 topic pub -1 /servo_angle std_msgs/msg/Float64 "{data: $angle}" > /dev/null 2>&1
            sleep 2
        done
        
        f1tenth_log_success "Sequência de teste concluída"
    fi
    
    # Cleanup
    kill $servo_pid 2>/dev/null || true
    sleep 1
    
    f1tenth_log_success "Teste servo básico finalizado"
}

test_joystick_control() {
    f1tenth_log_info "TESTE CONTROLE JOYSTICK"
    f1tenth_separator
    
    # Detectar joystick
    local joystick_dev
    joystick_dev=$(detect_joystick_device)
    
    if [[ -z "$joystick_dev" ]]; then
        f1tenth_log_error "Nenhum joystick detectado"
        f1tenth_log_info "Conecte um controle USB e tente novamente"
        return 1
    fi
    
    f1tenth_log_success "Joystick detectado: $joystick_dev"
    
    # Verificar permissões
    if [[ ! -r "$joystick_dev" ]]; then
        f1tenth_log_warn "Permissões insuficientes para joystick"
        f1tenth_log_info "Execute: sudo chmod 666 $joystick_dev"
    fi
    
    # Teste joy_node
    f1tenth_log_info "Testando joy_node..."
    ros2 run joy joy_node --ros-args -p device_id:=0 > /dev/null 2>&1 &
    local joy_pid=$!
    sleep 3
    
    # Verificar publicação /joy
    if timeout 5s ros2 topic echo /joy --once > /dev/null 2>&1; then
        f1tenth_log_success "Tópico /joy publicando dados"
    else
        f1tenth_log_error "Tópico /joy não está funcionando"
        kill $joy_pid 2>/dev/null || true
        return 1
    fi
    
    # Sistema completo de controle manual
    f1tenth_log_info "Iniciando sistema de controle manual..."
    kill $joy_pid 2>/dev/null || true
    sleep 1
    
    # Usar launch file do joy_converter
    ros2 launch joy_converter launch_joy_ackerman_fixed.py > /dev/null 2>&1 &
    local launch_pid=$!
    sleep 5
    
    # Verificar nós ativos
    local active_nodes
    active_nodes=$(ros2 node list 2>/dev/null | wc -l)
    
    if [[ $active_nodes -ge 2 ]]; then
        f1tenth_log_success "Sistema manual ativo ($active_nodes nós)"
        ros2 node list | while read -r node; do
            f1tenth_log_info "   - $node"
        done
    else
        f1tenth_log_error "Sistema manual falhou ao iniciar"
        kill $launch_pid 2>/dev/null || true
        return 1
    fi
    
    # Teste interativo
    if [[ "$INTERACTIVE_MODE" == "true" ]]; then
        f1tenth_log_info "TESTE INTERATIVO ($TEST_DURATION segundos)"
        echo ""
        echo "INSTRUÇÕES:"
        echo "1. Mova stick ESQUERDO: velocidade (cima/baixo)"
        echo "2. Mova stick DIREITO: direção (esquerda/direita)" 
        echo "3. Pressione L1/LB: ativar controle"
        echo "4. Observe movimento físico do servo/motor"
        echo ""
        
        # Monitorar comandos
        timeout ${TEST_DURATION}s ros2 topic echo /drive --once > /dev/null 2>&1 &
        local echo_pid=$!
        
        # Aguardar teste
        local count=0
        while [[ $count -lt $TEST_DURATION ]]; do
            if [[ $((count % 5)) -eq 0 ]]; then
                f1tenth_log_info "Teste em andamento... ${count}s/${TEST_DURATION}s"
            fi
            sleep 1
            ((count++))
        done
        
        kill $echo_pid 2>/dev/null || true
        f1tenth_log_success "Teste interativo finalizado"
    fi
    
    # Cleanup
    kill $launch_pid 2>/dev/null || true
    sleep 2
    pkill -f "joy_node" 2>/dev/null || true
    pkill -f "joy_ackerman" 2>/dev/null || true
    
    f1tenth_log_success "Teste joystick finalizado"
}

test_system_complete() {
    f1tenth_log_info "TESTE SISTEMA COMPLETO"
    f1tenth_separator
    
    # Iniciar sistema completo
    f1tenth_log_info "Iniciando f1tenth_complete_system..."
    
    ros2 launch f1tenth_control f1tenth_complete_system.launch.py > /dev/null 2>&1 &
    local system_pid=$!
    sleep 10
    
    # Verificar se sistema iniciou
    local active_nodes
    active_nodes=$(ros2 node list 2>/dev/null | wc -l)
    
    if [[ $active_nodes -ge 3 ]]; then
        f1tenth_log_success "Sistema completo ativo ($active_nodes nós)"
    else
        f1tenth_log_error "Sistema completo falhou ($active_nodes nós)"
        kill $system_pid 2>/dev/null || true
        return 1
    fi
    
    # Verificar tópicos principais
    local expected_topics=("/drive" "/servo_angle")
    local missing_topics=()
    
    for topic in "${expected_topics[@]}"; do
        if timeout 3s ros2 topic list | grep -q "$topic"; then
            f1tenth_log_success "Tópico: $topic"
        else
            f1tenth_log_warn "Tópico: $topic [AUSENTE]"
            missing_topics+=("$topic")
        fi
    done
    
    # Teste de comunicação
    f1tenth_log_info "Testando comunicação inter-nós..."
    
    # Publicar comando de teste
    ros2 topic pub -1 /drive ackermann_msgs/msg/AckermannDriveStamped \
        "{drive: {speed: 0.5, steering_angle: 0.1}}" > /dev/null 2>&1
    
    sleep 2
    
    # Verificar resposta do servo
    if timeout 3s ros2 topic echo /servo_angle --once > /dev/null 2>&1; then
        f1tenth_log_success "Comunicação drive → servo funcionando"
    else
        f1tenth_log_warn "Comunicação drive → servo com problemas"
    fi
    
    # Cleanup
    kill $system_pid 2>/dev/null || true
    sleep 3
    pkill -f "f1tenth" 2>/dev/null || true
    
    f1tenth_log_success "Teste sistema completo finalizado"
}

test_hardware_dependencies() {
    f1tenth_log_info "TESTE DEPENDÊNCIAS HARDWARE"
    f1tenth_separator
    
    local tests_passed=0
    local tests_total=0
    
    # Teste 1: pigpiod
    ((tests_total++))
    if systemctl is-active --quiet pigpiod; then
        f1tenth_log_success "pigpiod: ATIVO"
        ((tests_passed++))
    else
        f1tenth_log_error "pigpiod: INATIVO"
    fi
    
    # Teste 2: YDLiDAR SDK
    ((tests_total++))
    if [[ -f "/usr/local/lib/libydlidar_sdk.so" ]]; then
        f1tenth_log_success "YDLiDAR SDK: INSTALADO"
        ((tests_passed++))
    else
        f1tenth_log_error "YDLiDAR SDK: NÃO ENCONTRADO"
    fi
    
    # Teste 3: GPIO access
    ((tests_total++))
    if [[ -d "/sys/class/gpio" ]] && [[ -r "/sys/class/gpio" ]]; then
        f1tenth_log_success "GPIO: ACESSÍVEL"
        ((tests_passed++))
    else
        f1tenth_log_error "GPIO: INACESSÍVEL"
    fi
    
    # Teste 4: Grupos do usuário
    ((tests_total++))
    if groups "$USER" | grep -q "gpio"; then
        f1tenth_log_success "Grupo gpio: OK"
        ((tests_passed++))
    else
        f1tenth_log_error "Grupo gpio: FALTANDO"
    fi
    
    ((tests_total++))
    if groups "$USER" | grep -q "dialout"; then
        f1tenth_log_success "Grupo dialout: OK"
        ((tests_passed++))
    else
        f1tenth_log_error "Grupo dialout: FALTANDO"
    fi
    
    # Resumo
    f1tenth_log_info "Hardware: $tests_passed/$tests_total testes passaram"
    
    if [[ $tests_passed -eq $tests_total ]]; then
        f1tenth_log_success "Todos os testes de hardware passaram"
        return 0
    else
        f1tenth_log_warn "Alguns testes de hardware falharam"
        return 1
    fi
}

# =============================================================================
# FUNÇÕES AUXILIARES
# =============================================================================

detect_joystick_device() {
    # Procurar dispositivos joystick comuns
    local joystick_paths=(
        "/dev/input/js0"
        "/dev/input/js1"
        "/dev/input/event*"
    )
    
    for path in "${joystick_paths[@]}"; do
        if [[ -e $path ]]; then
            echo "$path"
            return 0
        fi
    done
    
    return 1
}

run_all_tests() {
    f1tenth_log_info "EXECUTANDO TODOS OS TESTES"
    f1tenth_separator
    
    local tests_results=()
    
    # Teste 1: Hardware
    f1tenth_log_info "Executando teste: Hardware Dependencies"
    if test_hardware_dependencies; then
        tests_results+=("Hardware: ✅")
    else
        tests_results+=("Hardware: ❌")
    fi
    
    echo ""
    
    # Teste 2: Servo
    f1tenth_log_info "Executando teste: Servo Basic"
    if test_servo_basic; then
        tests_results+=("Servo: ✅")
    else
        tests_results+=("Servo: ❌")
    fi
    
    echo ""
    
    # Teste 3: Sistema
    f1tenth_log_info "Executando teste: System Complete"
    if test_system_complete; then
        tests_results+=("Sistema: ✅")
    else
        tests_results+=("Sistema: ❌")
    fi
    
    echo ""
    
    # Teste 4: Joystick (apenas se detectado)
    if detect_joystick_device >/dev/null 2>&1; then
        f1tenth_log_info "Executando teste: Joystick Control"
        if test_joystick_control; then
            tests_results+=("Joystick: ✅")
        else
            tests_results+=("Joystick: ❌")
        fi
    else
        f1tenth_log_warn "Joystick não detectado, pulando teste"
        tests_results+=("Joystick: ⏭️")
    fi
    
    # Resumo final
    f1tenth_separator
    f1tenth_log_info "RESUMO DOS TESTES"
    f1tenth_separator
    
    for result in "${tests_results[@]}"; do
        echo "  $result"
    done
    
    echo ""
    
    # Contar sucessos
    local passed
    passed=$(printf '%s\n' "${tests_results[@]}" | grep -c "✅" || echo "0")
    local total=${#tests_results[@]}
    
    if [[ $passed -eq $total ]]; then
        f1tenth_log_success "TODOS OS TESTES PASSARAM! ($passed/$total)"
        return 0
    else
        f1tenth_log_warn "ALGUNS TESTES FALHARAM ($passed/$total)"
        return 1
    fi
}

show_test_results() {
    if [[ "$LOG_TESTS" == "true" ]]; then
        echo ""
        f1tenth_log_info "Log completo salvo em: $LOG_FILE"
        echo ""
        echo "Para revisar:"
        echo "  cat $LOG_FILE"
        echo "  tail -f $LOG_FILE"
    fi
}

# =============================================================================
# FUNÇÃO PRINCIPAL
# =============================================================================

main() {
    f1tenth_print_header "F1TENTH Master Test" "2.0.0"
    
    # Configuração inicial
    setup_test_environment
    validate_hardware_requirements
    
    f1tenth_log_info "Tipo de teste: $TEST_TYPE"
    f1tenth_log_info "Duração: ${TEST_DURATION}s"
    f1tenth_log_info "Interativo: $INTERACTIVE_MODE"
    
    f1tenth_separator
    
    # Executar teste selecionado
    local test_result=0
    
    case "$TEST_TYPE" in
        "$TEST_TYPE_SERVO")
            test_servo_basic || test_result=$?
            ;;
        "$TEST_TYPE_JOYSTICK")
            test_joystick_control || test_result=$?
            ;;
        "$TEST_TYPE_SYSTEM")
            test_system_complete || test_result=$?
            ;;
        "$TEST_TYPE_HARDWARE")
            test_hardware_dependencies || test_result=$?
            ;;
        "$TEST_TYPE_ALL")
            run_all_tests || test_result=$?
            ;;
        *)
            f1tenth_log_error "Tipo de teste inválido: $TEST_TYPE"
            usage
            exit 1
            ;;
    esac
    
    # Mostrar resultados finais
    show_test_results
    
    if [[ $test_result -eq 0 ]]; then
        f1tenth_log_success "Testes finalizados com sucesso!"
    else
        f1tenth_log_warn "Alguns testes apresentaram problemas"
    fi
    
    exit $test_result
}

# =============================================================================
# EXECUÇÃO
# =============================================================================

# Executar apenas se script for chamado diretamente
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi 