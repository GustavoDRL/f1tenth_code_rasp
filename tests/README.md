# Estrutura de Testes - Sistema F1tenth

## Visão Geral

Esta pasta contém uma suíte completa de testes para validar o funcionamento de cada componente do sistema F1tenth individualmente e em conjunto.

## Estrutura de Diretórios

```
tests/
├── README.md                    # Este arquivo
├── unit/                        # Testes unitários por componente
│   ├── test_servo_control.py    # Testes do controle de servo
│   ├── test_joy_converter.py    # Testes dos conversores de joystick
│   ├── test_enhanced_control.py # Testes do controle avançado
│   └── test_config_validation.py # Validação de configurações
├── integration/                 # Testes de integração
│   ├── test_ros_communication.py # Comunicação entre nós
│   ├── test_full_pipeline.py    # Pipeline completo
│   └── test_failsafe.py         # Sistemas de segurança
├── mock/                        # Mocks e simuladores
│   ├── mock_pigpio.py           # Mock do pigpio para testes
│   ├── mock_vesc.py             # Mock do driver VESC
│   └── test_fixtures.py        # Fixtures de teste
├── performance/                 # Testes de performance
│   ├── test_latency.py          # Latência de comandos
│   └── test_throughput.py       # Taxa de processamento
├── hardware/                    # Testes específicos de hardware
│   ├── test_gpio_interface.py   # Interface GPIO
│   └── test_servo_calibration.py # Calibração do servo
└── run_all_tests.py            # Script para executar todos os testes
```

## Tipos de Teste

### 1. Testes Unitários
- **Objetivo**: Validar cada função/classe isoladamente
- **Cobertura**: Lógica de negócio, cálculos, validações
- **Ambiente**: Não requer hardware físico

### 2. Testes de Integração
- **Objetivo**: Validar comunicação entre componentes
- **Cobertura**: Tópicos ROS2, pipeline de dados, sincronização
- **Ambiente**: ROS2 em execução

### 3. Testes de Performance
- **Objetivo**: Validar requisitos de tempo real
- **Cobertura**: Latência, throughput, uso de CPU/memória
- **Ambiente**: Condições de carga variável

### 4. Testes de Hardware
- **Objetivo**: Validar interfaces físicas
- **Cobertura**: GPIO, PWM, comunicação serial
- **Ambiente**: Raspberry Pi com hardware conectado

## Execução dos Testes

### Todos os Testes
```bash
cd tests/
python run_all_tests.py
```

### Por Categoria
```bash
# Testes unitários
python -m pytest unit/ -v

# Testes de integração
python -m pytest integration/ -v

# Testes de performance
python -m pytest performance/ -v

# Testes de hardware (requer hardware)
python -m pytest hardware/ -v
```

### Teste Individual
```bash
python -m pytest unit/test_servo_control.py::TestServoControl::test_angle_conversion -v
```

## Configuração do Ambiente

### Dependências
```bash
pip install pytest pytest-asyncio pytest-mock pytest-cov
```

### Variáveis de Ambiente
```bash
export ROS_DOMAIN_ID=0
export F1TENTH_TEST_MODE=simulation  # ou 'hardware'
```

## Relatórios de Cobertura

```bash
# Gerar relatório de cobertura
python -m pytest --cov=src --cov-report=html --cov-report=term

# Visualizar relatório
firefox htmlcov/index.html
```

## Critérios de Aceitação

### Testes Unitários
- ✅ Cobertura mínima: 90%
- ✅ Todos os casos extremos testados
- ✅ Validação de parâmetros de entrada

### Testes de Integração
- ✅ Comunicação ROS2 funcional
- ✅ Pipeline completo sem erros
- ✅ Failsafe ativado corretamente

### Testes de Performance
- ✅ Latência < 10ms para controle de servo
- ✅ Throughput > 100Hz para comandos
- ✅ Uso de CPU < 50% em operação normal

### Testes de Hardware
- ✅ GPIO funcional em Raspberry Pi
- ✅ Servo responde corretamente
- ✅ VESC comunica sem erros

## Debugging

### Logs Detalhados
```bash
export ROS_LOG_LEVEL=DEBUG
python -m pytest unit/test_servo_control.py -v -s --log-cli-level=DEBUG
```

### Modo Interativo
```bash
python -m pytest --pdb unit/test_servo_control.py
```

## Continuous Integration

Os testes são executados automaticamente em:
- Push para branch main
- Pull requests
- Releases

### Pipeline CI/CD
1. Testes unitários (sempre)
2. Testes de integração (sempre)
3. Testes de performance (releases)
4. Testes de hardware (manual)
