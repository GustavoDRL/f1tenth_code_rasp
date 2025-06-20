#!/usr/bin/env python3
"""
Configuração global do pytest para testes F1TENTH.

Este arquivo configura:
- Paths para importação dos módulos
- Fixtures globais compartilhadas
- Marcadores de teste personalizados
- Configuração do ambiente de teste

Autor: Professor PhD em Engenharia Robótica
"""

import os
import sys
import pytest
from pathlib import Path

# Adicionar src ao PYTHONPATH para importações
project_root = Path(__file__).parent.parent
src_path = project_root / "src"
sys.path.insert(0, str(src_path))

# Definir variáveis de ambiente para testes
os.environ["F1TENTH_TEST_MODE"] = "simulation"
os.environ["ROS_DOMAIN_ID"] = "0"

# Importar fixtures globais
pytest_plugins = [
    "tests.mock.test_fixtures",
]

def pytest_configure(config):
    """Configuração global do pytest."""
    # Definir marcadores personalizados
    config.addinivalue_line(
        "markers", "hardware: marca testes que requerem hardware"
    )
    config.addinivalue_line(
        "markers", "performance: marca testes de performance"
    )
    config.addinivalue_line(
        "markers", "integration: marca testes de integração"
    )
    config.addinivalue_line(
        "markers", "slow: marca testes que demoram para executar"
    )
    # Adicionar marcadores específicos usados nos testes
    config.addinivalue_line(
        "markers", "unit: marca testes unitários"
    )
    config.addinivalue_line(
        "markers", "joy_converter: marca testes do conversor de joystick"
    )
    config.addinivalue_line(
        "markers", "servo: marca testes do controle de servo"
    )
    config.addinivalue_line(
        "markers", "ros_communication: marca testes de comunicação ROS2"
    )
    config.addinivalue_line(
        "markers", "latency: marca testes de latência"
    )
    config.addinivalue_line(
        "markers", "realtime: marca testes de tempo real"
    )

def pytest_collection_modifyitems(config, items):
    """Modifica a coleta de testes para adicionar marcadores automáticos."""
    for item in items:
        # Marcar testes baseado no path do arquivo
        if "hardware" in str(item.fspath):
            item.add_marker(pytest.mark.hardware)
        elif "performance" in str(item.fspath):
            item.add_marker(pytest.mark.performance)
        elif "integration" in str(item.fspath):
            item.add_marker(pytest.mark.integration)

@pytest.fixture(scope="session", autouse=True)
def setup_test_session():
    """Setup inicial da sessão de testes."""
    print("\n🔧 Configurando sessão de testes F1TENTH...")
    
    # Verificar se estamos em modo de simulação
    test_mode = os.environ.get("F1TENTH_TEST_MODE", "simulation")
    print(f"   Modo de teste: {test_mode}")
    
    # Configurar paths
    print(f"   Path do projeto: {project_root}")
    print(f"   Path do src: {src_path}")
    
    yield
    
    print("\n✅ Sessão de testes finalizada")

@pytest.fixture(autouse=True)
def cleanup_after_test():
    """Limpeza automática após cada teste."""
    yield
    # Limpeza pode ser adicionada aqui se necessário 