#!/usr/bin/env python3
"""
Script principal para execução de todos os testes do sistema F1tenth.

Este script executa sequencialmente:
1. Testes unitários
2. Testes de integração
3. Testes de performance (opcional)
4. Testes de hardware (apenas se disponível)

Autor: Professor PhD em Engenharia Robótica
Data: 2025
"""

import os
import sys
import subprocess
import time
import json
from pathlib import Path
from datetime import datetime


class F1TenthTestRunner:
    """Executor de testes para o sistema F1tenth com relatórios detalhados."""

    def __init__(self):
        self.test_dir = Path(__file__).parent
        self.start_time = datetime.now()
        self.results = {
            'unit': {'passed': 0, 'failed': 0, 'errors': []},
            'integration': {'passed': 0, 'failed': 0, 'errors': []},
            'hybrid_system': {'passed': 0, 'failed': 0, 'errors': []},
            'performance': {'passed': 0, 'failed': 0, 'errors': []},
            'hardware': {'passed': 0, 'failed': 0, 'errors': []}
        }

        # Configuração do ambiente
        self.setup_environment()

    def setup_environment(self):
        """Configura ambiente para execução dos testes."""
        os.environ['PYTHONPATH'] = str(self.test_dir.parent / 'src')
        test_mode = os.getenv('F1TENTH_TEST_MODE', 'simulation')
        os.environ['F1TENTH_TEST_MODE'] = test_mode
        os.environ['ROS_DOMAIN_ID'] = os.getenv('ROS_DOMAIN_ID', '0')

        print("🔧 Configuração do Ambiente:")
        print(f"   PYTHONPATH: {os.environ['PYTHONPATH']}")
        print(f"   F1TENTH_TEST_MODE: {os.environ['F1TENTH_TEST_MODE']}")
        print(f"   ROS_DOMAIN_ID: {os.environ['ROS_DOMAIN_ID']}")
        print()

    def check_dependencies(self) -> bool:
        """Verifica se todas as dependências estão instaladas."""
        print("📦 Verificando Dependências...")

        required_packages = [
            'pytest', 'pytest-asyncio', 'pytest-mock',
            'pytest-cov', 'rclpy', 'ackermann_msgs'
        ]

        missing = []
        for package in required_packages:
            try:
                __import__(package.replace('-', '_'))
                print(f"   ✅ {package}")
            except ImportError:
                missing.append(package)
                print(f"   ❌ {package}")

        if missing:
            print(f"\n⚠️  Dependências faltando: {', '.join(missing)}")
            print("   Instale com: pip install " + " ".join(missing))
            return False

        print("   ✅ Todas as dependências estão disponíveis\n")
        return True

    def run_test_category(self, category: str, optional: bool = False) -> bool:
        """Executa uma categoria específica de testes."""
        test_path = self.test_dir / category

        if not test_path.exists():
            if optional:
                print(f"⏭️  Categoria {category} não encontrada (opcional)")
                return True
            else:
                print(f"❌ Categoria {category} não encontrada (obrigatória)")
                return False

        print(f"🧪 Executando Testes: {category.upper()}")
        print(f"   Diretório: {test_path}")

        # Configurar comando pytest
        cmd = [
            'python', '-m', 'pytest',
            str(test_path),
            '-v',
            '--tb=short',
            '--no-header',
            f'--junitxml=results_{category}.xml'
        ]

        # Adicionar cobertura para testes unitários
        if category == 'unit':
            cmd.extend([
                '--cov=src',
                '--cov-report=term-missing',
                f'--cov-report=html:coverage_{category}'
            ])

        try:
            start_time = time.time()
            result = subprocess.run(
                cmd,
                cwd=self.test_dir,
                capture_output=True,
                text=True,
                timeout=300  # 5 minutos timeout
            )

            execution_time = time.time() - start_time

            # Processar resultados
            self.parse_test_results(category, result, execution_time)

            if result.returncode == 0:
                print(f"   ✅ {category.upper()} - Todos os testes passaram")
                return True
            else:
                print(f"   ❌ {category.upper()} - Alguns testes falharam")
                if result.stdout:
                    stdout_tail = result.stdout[-500:]  # Últimas 500 chars
                    print("   STDOUT:", stdout_tail)
                if result.stderr:
                    print("   STDERR:", result.stderr[-500:])
                return False

        except subprocess.TimeoutExpired:
            print(f"   ⏱️  {category.upper()} - Timeout (>5min)")
            self.results[category]['errors'].append("Timeout na execução")
            return False
        except Exception as e:
            print(f"   💥 {category.upper()} - Erro: {e}")
            self.results[category]['errors'].append(str(e))
            return False

    def parse_test_results(self, category: str, result, execution_time: float):
        """Extrai informações dos resultados dos testes."""
        output = result.stdout

        # Buscar por padrões pytest
        import re

        # Padrão: "X passed, Y failed in Zs"
        pattern = r'(\d+) passed(?:, (\d+) failed)?.*in ([\d.]+)s'
        match = re.search(pattern, output)

        if match:
            passed = int(match.group(1))
            failed = int(match.group(2)) if match.group(2) else 0

            self.results[category]['passed'] = passed
            self.results[category]['failed'] = failed
            self.results[category]['execution_time'] = execution_time

        # Extrair erros específicos
        if result.returncode != 0:
            lines = output.split('\n')
            error_lines = [line for line in lines 
                          if 'FAILED' in line or 'ERROR' in line]
            max_errors = 5  # Máximo 5 erros
            self.results[category]['errors'].extend(error_lines[:max_errors])

    def check_hardware_availability(self) -> bool:
        """Verifica se o hardware está disponível para testes."""
        if os.getenv('F1TENTH_TEST_MODE') != 'hardware':
            return False

        # Verificar se estamos no Raspberry Pi
        try:
            with open('/proc/cpuinfo', 'r') as f:
                content = f.read()
                if 'Raspberry Pi' not in content:
                    return False
        except (IOError, OSError):
            return False

        # Verificar GPIO e pigpio
        try:
            import pigpio
            pi = pigpio.pi()
            if pi.connected:
                pi.stop()
                return True
        except (ImportError, Exception):
            pass

        return False

    def generate_report(self):
        """Gera relatório final dos testes."""
        end_time = datetime.now()
        total_time = end_time - self.start_time

        print("\n" + "="*60)
        print("📊 RELATÓRIO FINAL DE TESTES - SISTEMA F1TENTH")
        print("="*60)

        total_passed = sum(cat['passed'] for cat in self.results.values())
        total_failed = sum(cat['failed'] for cat in self.results.values())
        total_tests = total_passed + total_failed

        print(f"⏱️  Tempo Total: {total_time}")
        print(f"🧪 Total de Testes: {total_tests}")
        print(f"✅ Passaram: {total_passed}")
        print(f"❌ Falharam: {total_failed}")
        if total_tests > 0:
            success_rate = (total_passed/total_tests*100)
            print(f"📈 Taxa de Sucesso: {success_rate:.1f}%")
        else:
            print("📈 Taxa de Sucesso: N/A")
        print()

        # Detalhes por categoria
        for category, results in self.results.items():
            if results['passed'] + results['failed'] > 0:
                print(f"📂 {category.upper()}:")
                print(f"   ✅ Passaram: {results['passed']}")
                print(f"   ❌ Falharam: {results['failed']}")

                if results['errors']:
                    print("   🔍 Erros principais:")
                    for error in results['errors'][:3]:
                        print(f"      • {error}")
                print()

        # Salvar relatório JSON
        timestamp_str = self.start_time.strftime('%Y%m%d_%H%M%S')
        report_file = self.test_dir / f"test_report_{timestamp_str}.json"
        report_data = {
            'timestamp': self.start_time.isoformat(),
            'duration': str(total_time),
            'total_tests': total_tests,
            'total_passed': total_passed,
            'total_failed': total_failed,
            'success_rate': ((total_passed/total_tests*100) 
                            if total_tests > 0 else 0),
            'categories': self.results
        }

        with open(report_file, 'w') as f:
            json.dump(report_data, f, indent=2)

        print(f"💾 Relatório salvo em: {report_file}")

        # Recomendações
        if total_failed > 0:
            print("\n🔧 RECOMENDAÇÕES:")
            print("   • Execute testes falhados individualmente para debug")
            print("   • Verifique logs detalhados com --log-cli-level=DEBUG")
            print("   • Considere executar no modo hardware se apropriado")

        return total_failed == 0

    def run_all(self) -> bool:
        """Executa todos os testes na sequência correta."""
        print("🚀 INICIANDO TESTES DO SISTEMA F1TENTH")
        print("="*50)
        print()

        # Verificar dependências
        if not self.check_dependencies():
            return False

        # Executar categorias de teste
        categories = [
            ('unit', False),           # Obrigatório
            ('integration', False),    # Obrigatório
            ('hybrid_system', False),  # Obrigatório - Sistema híbrido
            ('performance', True),     # Opcional
        ]

        # Adicionar hardware se disponível
        if self.check_hardware_availability():
            categories.append(('hardware', True))
            print("🔧 Hardware detectado - testes de hardware incluídos")
        else:
            print("💻 Modo simulação - testes de hardware ignorados")

        print()

        # Executar testes
        all_passed = True
        for category, optional in categories:
            success = self.run_test_category(category, optional)
            if not success and not optional:
                all_passed = False
            print()

        # Gerar relatório
        success = self.generate_report()

        return success and all_passed


def main():
    """Função principal."""
    runner = F1TenthTestRunner()

    try:
        success = runner.run_all()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n⏹️  Execução interrompida pelo usuário")
        sys.exit(1)
    except Exception as e:
        print(f"\n💥 Erro fatal: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
