#!/bin/bash
# =============================================================================
# F1TENTH Legacy Cleanup Script - Remoção de Scripts Obsoletos
# Versão: 2.0.0
# Descrição: Remove scripts redundantes e obsoletos após refatoração
# =============================================================================

# Proteções e configuração inicial
set -euo pipefail
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Carregar funções comuns
source "$WORKSPACE_ROOT/scripts/utils/common_functions.sh"

# =============================================================================
# CONFIGURAÇÕES DE LIMPEZA
# =============================================================================

# Scripts legacy que serão removidos (redundantes/obsoletos)
readonly LEGACY_SCRIPTS=(
    "scripts/build_f1tenth.sh"                    # Substituído por build/master_build.sh
    "scripts/build_and_test_f1tenth.sh"           # Substituído por build/master_build.sh
    "scripts/post_build_setup.sh"                 # Integrado ao master_build.sh
    "scripts/test_f1tenth.sh"                     # Substituído por test/master_test.sh
    "scripts/test_f1tenth_manual_control.sh"      # Substituído por test/master_test.sh
    "scripts/f1tenth_startup.sh"                  # Substituído por runtime/startup.sh
    "scripts/detect_8bitdo_controller.sh"         # Funcionalidade integrada ao master_test.sh
)

# Arquivos de backup que podem ser removidos
readonly BACKUP_FILES=(
    "*.bak"
    "*.backup"
    "*.old"
    "*~"
    ".*.swp"
    ".*.swo"
)

# Parâmetros de limpeza
DRY_RUN=false
FORCE_CLEANUP=false
BACKUP_BEFORE_DELETE=true
VERBOSE_MODE=false

# =============================================================================
# PROCESSAMENTO DE ARGUMENTOS  
# =============================================================================

usage() {
    cat << EOF
🧹 F1TENTH Legacy Cleanup Script v2.0.0

USAGE:
    $0 [OPTIONS]

OPTIONS:
    --dry-run                 Simular limpeza sem remover arquivos
    --force                   Forçar remoção sem confirmação
    --no-backup               Não criar backup antes de remover
    -v, --verbose             Modo verboso
    -h, --help                Mostrar esta ajuda

DESCRIPTION:
    Remove scripts obsoletos e redundantes após refatoração:
    - Scripts legacy consolidados em versões master
    - Arquivos de backup temporários
    - Links simbólicos quebrados
    
EXAMPLES:
    $0 --dry-run              # Simular limpeza
    $0 --force                # Limpeza automática
    $0 --verbose              # Limpeza com detalhes

EOF
}

# Processar argumentos
while [[ $# -gt 0 ]]; do
    case $1 in
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        --force)
            FORCE_CLEANUP=true
            shift
            ;;
        --no-backup)
            BACKUP_BEFORE_DELETE=false
            shift
            ;;
        -v|--verbose)
            VERBOSE_MODE=true
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
# FUNÇÕES DE LIMPEZA
# =============================================================================

show_cleanup_summary() {
    f1tenth_log_info "RESUMO DA LIMPEZA PLANEJADA"
    f1tenth_separator
    
    echo ""
    echo "📁 Scripts Legacy (serão removidos):"
    for script in "${LEGACY_SCRIPTS[@]}"; do
        if [[ -f "$WORKSPACE_ROOT/$script" ]]; then
            echo "  ❌ $script"
        else
            echo "  ✅ $script (já removido)"
        fi
    done
    
    echo ""
    echo "🗂️ Arquivos de Backup (serão removidos):"
    local backup_count=0
    for pattern in "${BACKUP_FILES[@]}"; do
        if ls "$WORKSPACE_ROOT/"$pattern 2>/dev/null; then
            ((backup_count++))
        fi
    done
    
    if [[ $backup_count -eq 0 ]]; then
        echo "  ✅ Nenhum arquivo de backup encontrado"
    else
        echo "  ❌ $backup_count arquivo(s) de backup encontrado(s)"
    fi
    
    echo ""
    echo "🔗 Links Simbólicos (verificação):"
    check_broken_symlinks_summary
    
    echo ""
}

check_broken_symlinks_summary() {
    local broken_links=()
    
    while IFS= read -r -d '' link; do
        if [[ -L "$link" ]] && [[ ! -e "$link" ]]; then
            broken_links+=("$(basename "$link")")
        fi
    done < <(find "$WORKSPACE_ROOT" -type l -print0 2>/dev/null)
    
    if [[ ${#broken_links[@]} -eq 0 ]]; then
        echo "  ✅ Nenhum link simbólico quebrado"
    else
        echo "  ❌ ${#broken_links[@]} link(s) quebrado(s):"
        for link in "${broken_links[@]}"; do
            echo "    - $link"
        done
    fi
}

confirm_cleanup() {
    if [[ "$FORCE_CLEANUP" == "true" ]]; then
        return 0
    fi
    
    echo ""
    f1tenth_log_warn "ATENÇÃO: Esta operação irá remover arquivos permanentemente!"
    
    if [[ "$BACKUP_BEFORE_DELETE" == "true" ]]; then
        f1tenth_log_info "Backup será criado antes da remoção"
    else
        f1tenth_log_warn "Nenhum backup será criado"
    fi
    
    echo ""
    read -p "Continuar com a limpeza? [y/N]: " -r
    
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        f1tenth_log_info "Limpeza cancelada pelo usuário"
        exit 0
    fi
}

create_backup() {
    if [[ "$BACKUP_BEFORE_DELETE" != "true" ]]; then
        return 0
    fi
    
    local backup_dir="$WORKSPACE_ROOT/scripts/.cleanup_backup_$(date +%Y%m%d_%H%M%S)"
    
    f1tenth_log_info "Criando backup em: $backup_dir"
    mkdir -p "$backup_dir"
    
    # Backup dos scripts legacy
    for script in "${LEGACY_SCRIPTS[@]}"; do
        if [[ -f "$WORKSPACE_ROOT/$script" ]]; then
            if [[ "$DRY_RUN" == "false" ]]; then
                cp "$WORKSPACE_ROOT/$script" "$backup_dir/"
                [[ "$VERBOSE_MODE" == "true" ]] && f1tenth_log_info "Backup: $script"
            else
                [[ "$VERBOSE_MODE" == "true" ]] && f1tenth_log_info "Backup: $script [DRY-RUN]"
            fi
        fi
    done
    
    # Backup dos arquivos temporários
    for pattern in "${BACKUP_FILES[@]}"; do
        for file in "$WORKSPACE_ROOT/"$pattern; do
            if [[ -f "$file" ]]; then
                if [[ "$DRY_RUN" == "false" ]]; then
                    cp "$file" "$backup_dir/"
                    [[ "$VERBOSE_MODE" == "true" ]] && f1tenth_log_info "Backup: $(basename "$file")"
                else
                    [[ "$VERBOSE_MODE" == "true" ]] && f1tenth_log_info "Backup: $(basename "$file") [DRY-RUN]"
                fi
            fi
        done
    done
    
    if [[ "$DRY_RUN" == "false" ]]; then
        f1tenth_log_success "Backup criado: $backup_dir"
    else
        f1tenth_log_info "Backup seria criado: $backup_dir [DRY-RUN]"
    fi
}

remove_legacy_scripts() {
    f1tenth_log_info "Removendo scripts legacy..."
    
    local removed_count=0
    
    for script in "${LEGACY_SCRIPTS[@]}"; do
        local script_path="$WORKSPACE_ROOT/$script"
        
        if [[ -f "$script_path" ]]; then
            if [[ "$DRY_RUN" == "false" ]]; then
                rm "$script_path"
                f1tenth_log_success "Removido: $script"
            else
                f1tenth_log_info "Removeria: $script [DRY-RUN]"
            fi
            ((removed_count++))
        else
            [[ "$VERBOSE_MODE" == "true" ]] && f1tenth_log_info "Não encontrado: $script"
        fi
    done
    
    f1tenth_log_info "Scripts legacy: $removed_count removido(s)"
}

remove_backup_files() {
    f1tenth_log_info "Removendo arquivos de backup..."
    
    local removed_count=0
    
    for pattern in "${BACKUP_FILES[@]}"; do
        for file in "$WORKSPACE_ROOT/"$pattern; do
            if [[ -f "$file" ]]; then
                if [[ "$DRY_RUN" == "false" ]]; then
                    rm "$file"
                    f1tenth_log_success "Removido: $(basename "$file")"
                else
                    f1tenth_log_info "Removeria: $(basename "$file") [DRY-RUN]"
                fi
                ((removed_count++))
            fi
        done
    done
    
    f1tenth_log_info "Arquivos backup: $removed_count removido(s)"
}

fix_broken_symlinks() {
    f1tenth_log_info "Corrigindo links simbólicos quebrados..."
    
    local fixed_count=0
    
    while IFS= read -r -d '' link; do
        if [[ -L "$link" ]] && [[ ! -e "$link" ]]; then
            if [[ "$DRY_RUN" == "false" ]]; then
                rm "$link"
                f1tenth_log_success "Removido link quebrado: $(basename "$link")"
            else
                f1tenth_log_info "Removeria link quebrado: $(basename "$link") [DRY-RUN]"
            fi
            ((fixed_count++))
        fi
    done < <(find "$WORKSPACE_ROOT" -type l -print0 2>/dev/null)
    
    f1tenth_log_info "Links quebrados: $fixed_count removido(s)"
}

update_permissions() {
    f1tenth_log_info "Atualizando permissões dos scripts..."
    
    # Tornar scripts executáveis
    local script_dirs=(
        "$WORKSPACE_ROOT/setup"
        "$WORKSPACE_ROOT/build" 
        "$WORKSPACE_ROOT/test"
        "$WORKSPACE_ROOT/runtime"
        "$WORKSPACE_ROOT/utils"
    )
    
    for dir in "${script_dirs[@]}"; do
        if [[ -d "$dir" ]]; then
            if [[ "$DRY_RUN" == "false" ]]; then
                chmod +x "$dir"/*.sh 2>/dev/null || true
                [[ "$VERBOSE_MODE" == "true" ]] && f1tenth_log_info "Permissões: $dir/*.sh"
            else
                [[ "$VERBOSE_MODE" == "true" ]] && f1tenth_log_info "Permissões: $dir/*.sh [DRY-RUN]"
            fi
        fi
    done
    
    f1tenth_log_success "Permissões atualizadas"
}

show_final_structure() {
    f1tenth_log_info "ESTRUTURA FINAL DOS SCRIPTS"
    f1tenth_separator
    
    echo ""
    echo "📁 scripts/"
    echo "├── 📚 README.md"
    echo "├── 🔧 setup/"
    echo "│   ├── install_dependencies.sh"
    echo "│   └── install_service.sh"
    echo "├── 🏗️ build/"
    echo "│   ├── master_build.sh         # ✨ NOVO: Build consolidado"
    echo "│   └── quick_build.sh"
    echo "├── 🧪 test/"
    echo "│   ├── master_test.sh          # ✨ NOVO: Testes consolidados"
    echo "│   ├── test_servo_basic.sh"
    echo "│   └── detect_controller.sh"
    echo "├── 🚀 runtime/"
    echo "│   ├── startup.sh              # ✨ NOVO: Startup otimizado"
    echo "│   └── f1tenth.service"
    echo "└── 🛠️ utils/"
    echo "    ├── common_functions.sh     # ✨ NOVO: Funções compartilhadas"
    echo "    └── config_manager.sh       # ✨ NOVO: Configurações centralizadas"
    echo ""
    
    echo "✅ BENEFÍCIOS DA REFATORAÇÃO:"
    echo "• Eliminadas redundâncias: 5 scripts → 3 scripts master"
    echo "• Funções compartilhadas: reutilização de código"
    echo "• Configuração centralizada: parâmetros unificados"
    echo "• Documentação completa: README.md atualizado"
    echo "• Estrutura organizada: diretórios por função"
    echo ""
}

# =============================================================================
# FUNÇÃO PRINCIPAL
# =============================================================================

main() {
    f1tenth_print_header "F1TENTH Legacy Cleanup" "2.0.0"
    
    # Verificar se estamos no diretório correto
    if [[ ! -f "$WORKSPACE_ROOT/src/f1tenth_control/package.xml" ]]; then
        f1tenth_log_error "Execute no diretório raiz do workspace F1TENTH"
        exit 1
    fi
    
    # Mostrar resumo da limpeza
    show_cleanup_summary
    
    # Confirmação do usuário
    if [[ "$DRY_RUN" == "false" ]]; then
        confirm_cleanup
    else
        f1tenth_log_info "Modo DRY-RUN ativado - nenhum arquivo será removido"
    fi
    
    # Executar limpeza
    f1tenth_separator
    f1tenth_log_info "Iniciando limpeza..."
    
    # Criar backup se necessário
    create_backup
    
    # Remover arquivos
    remove_legacy_scripts
    remove_backup_files
    fix_broken_symlinks
    
    # Atualizar configurações
    update_permissions
    
    # Mostrar estrutura final
    f1tenth_separator
    show_final_structure
    
    # Resumo final
    if [[ "$DRY_RUN" == "false" ]]; then
        f1tenth_log_success "Limpeza concluída com sucesso!"
        
        if [[ "$BACKUP_BEFORE_DELETE" == "true" ]]; then
            echo ""
            f1tenth_log_info "💾 Backup dos arquivos removidos disponível em:"
            find "$WORKSPACE_ROOT/scripts" -name ".cleanup_backup_*" -type d 2>/dev/null | head -1
        fi
    else
        f1tenth_log_info "DRY-RUN finalizado - execute sem --dry-run para aplicar"
    fi
    
    echo ""
    f1tenth_log_info "🚀 PRÓXIMOS PASSOS:"
    echo "1. Execute os novos scripts master:"
    echo "   ./scripts/build/master_build.sh"
    echo "   ./scripts/test/master_test.sh servo"
    echo ""
    echo "2. Valide o sistema:"
    echo "   ./scripts/test/master_test.sh all"
    echo ""
    
    f1tenth_separator
}

# =============================================================================
# EXECUÇÃO
# =============================================================================

# Executar apenas se script for chamado diretamente
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi 