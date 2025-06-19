# 📋 ÍNDICE COMPLETO - DOCUMENTAÇÃO F1TENTH RASPBERRY PI

**Projeto**: Sistema de Controle Integrado F1TENTH para Raspberry Pi
**Versão**: 1.0
**Última Atualização**: 2025-06-19
**Status**: Sistema Operacional - Calibração Servo Descoberta

---

## 🎯 VISÃO GERAL DO PROJETO

Este projeto implementa um sistema completo de controle para veículos autônomos F1TENTH em escala 1/10, executando em Raspberry Pi 4B com ROS2 Humble. O sistema integra controle de motor via VESC, direção via servo GPIO, interface joystick e preparação para sensores LiDAR.

---

## 🏁 **STATUS GERAL DO PROJETO**
- [`06_STATUS_PROJETO_F1TENTH.md`](./06_STATUS_PROJETO_F1TENTH.md) - **📊 Status completo e atualizado do projeto**

---

## 📁 ESTRUTURA DA DOCUMENTAÇÃO

### 🔧 **ANÁLISES TÉCNICAS**
- [`01_ANALISE_ARQUITETURA_SISTEMA.md`](./analises/01_ANALISE_ARQUITETURA_SISTEMA.md) - Arquitetura completa do sistema
- [`02_ANALISE_PACOTES_ROS2.md`](./analises/02_ANALISE_PACOTES_ROS2.md) - Detalhamento de todos os pacotes
- [`03_ANALISE_FLUXO_COMUNICACAO.md`](./analises/03_ANALISE_FLUXO_COMUNICACAO.md) - Mapeamento de tópicos e comunicação
- [`04_RELATORIO_REVIEW_TECNICO_CODIGO.md`](./analises/04_RELATORIO_REVIEW_TECNICO_CODIGO.md) - **Review técnico detalhado do código**
- [`05_CALIBRACAO_SERVO_DESCOBERTAS.md`](./analises/05_CALIBRACAO_SERVO_DESCOBERTAS.md) - **🔧 Calibração servo - descobertas validadas**

### ⚙️ **IMPLEMENTAÇÕES**
- [`05_IMPLEMENTACAO_CONTROLE_INTEGRADO.md`](./implementacoes/05_IMPLEMENTACAO_CONTROLE_INTEGRADO.md) - f1tenth_control detalhado
- [`06_IMPLEMENTACAO_CONVERSORES_JOY.md`](./implementacoes/06_IMPLEMENTACAO_CONVERSORES_JOY.md) - Joy_converter completo
- [`07_IMPLEMENTACAO_VESC_INTEGRATION.md`](./implementacoes/07_IMPLEMENTACAO_VESC_INTEGRATION.md) - Integração VESC
- [`08_IMPLEMENTACAO_CALIBRACAO_SISTEMA.md`](./implementacoes/08_IMPLEMENTACAO_CALIBRACAO_SISTEMA.md) - Ferramentas de calibração

### 🚀 **CONFIGURAÇÕES E DEPLOYMENT**
- [`09_CONFIGURACAO_PARAMETROS.md`](./configuracoes/09_CONFIGURACAO_PARAMETROS.md) - Todos os parâmetros do sistema
- [`10_CONFIGURACAO_LAUNCH_FILES.md`](./configuracoes/10_CONFIGURACAO_LAUNCH_FILES.md) - Launch files detalhados
- [`11_SETUP_COMPLETO_RASPBERRY.md`](./configuracoes/11_SETUP_COMPLETO_RASPBERRY.md) - Setup completo do Raspberry Pi
- [`12_GUIA_INSTALACAO_DEPENDENCIAS.md`](./configuracoes/12_GUIA_INSTALACAO_DEPENDENCIAS.md) - Instalação de todas as dependências
- [`22_WORKFLOW_COMANDOS_RASPBERRY.md`](./configuracoes/22_WORKFLOW_COMANDOS_RASPBERRY.md) - **Workflow comandos Raspberry Pi**

### 🔬 **DESENVOLVIMENTO E EXTENSÕES**
- [`13_ROADMAP_DESENVOLVIMENTO.md`](./desenvolvimento/13_ROADMAP_DESENVOLVIMENTO.md) - Plano de desenvolvimento futuro
- [`20_PLANO_CORRECAO_PROBLEMAS_TECNICOS.md`](./desenvolvimento/20_PLANO_CORRECAO_PROBLEMAS_TECNICOS.md) - **Plano detalhado de correções**
- [`21_REGRAS_WORKFLOW_SSH_RASPBERRY.md`](./21_REGRAS_WORKFLOW_SSH_RASPBERRY.md) - **REGRAS OBRIGATÓRIAS - Workflow SSH/Raspberry Pi**
- [`22_WORKFLOW_COMANDOS_RASPBERRY.md`](./configuracoes/22_WORKFLOW_COMANDOS_RASPBERRY.md) - **Workflow detalhado de comandos**
- [`14_INTEGRACAO_LIDAR_PLANEJADA.md`](./desenvolvimento/14_INTEGRACAO_LIDAR_PLANEJADA.md) - Integração LiDAR pendente
- [`15_OTIMIZACOES_PERFORMANCE.md`](./desenvolvimento/15_OTIMIZACOES_PERFORMANCE.md) - Otimizações recomendadas
- [`16_EXTENSOES_NAVEGACAO_AUTONOMA.md`](./desenvolvimento/16_EXTENSOES_NAVEGACAO_AUTONOMA.md) - Extensões para navegação

### 🔧 **TROUBLESHOOTING E MANUTENÇÃO**
- [`17_GUIA_TROUBLESHOOTING.md`](./manutencao/17_GUIA_TROUBLESHOOTING.md) - Solução de problemas comuns
- [`18_PROCEDIMENTOS_CALIBRACAO.md`](./manutencao/18_PROCEDIMENTOS_CALIBRACAO.md) - Procedimentos de calibração
- [`19_MONITORAMENTO_SISTEMA.md`](./manutencao/19_MONITORAMENTO_SISTEMA.md) - Monitoramento e diagnósticos
- [`20_BACKUP_CONFIGURACOES.md`](./manutencao/20_BACKUP_CONFIGURACOES.md) - Backup e restauração

---

## 🎛️ COMPONENTES PRINCIPAIS

### **Hardware Alvo**
- **Computador**: Raspberry Pi 4B (4GB+ recomendado)
- **SO**: Ubuntu Server 22.04 LTS
- **ROS**: ROS2 Humble Hawksbill
- **Controlador Motor**: VESC 6.0+ via USB
- **Servo Direção**: Servo RC padrão (GPIO 18)
- **Interface**: Gamepad/Joystick compatível
- **Sensor**: YDLiDAR (preparado para integração)

### **Pacotes ROS2**
1. **f1tenth_control** (principal)
   - `servo_control_node.py` - Controle básico
   - `enhanced_servo_control_node.py` - Controle avançado com PID
   - `servo_calibration.py` - Ferramenta de calibração

2. **Joy_converter** (interface usuário)
   - `joy_ackerman.py` - Conversão para Ackermann
   - `joy_twist.py` - Conversão para Twist

3. **vesc-humble** (motor)
   - `vesc_driver` - Driver principal VESC
   - `vesc_ackermann` - Conversão Ackermann↔VESC
   - `vesc_msgs` - Mensagens específicas

4. **vesc_config** (configuração)
   - Parâmetros específicos do hardware

---

## 🔄 FLUXO DE DADOS PRINCIPAL

```
Joystick → Joy_converter → /drive → f1tenth_control → GPIO Servo
                                ↓
                        vesc_ackermann → VESC Motor
                                ↓
                        vesc_to_odom → /odom → /ego_racecar/odom
```

---

## 📊 STATUS ATUAL DO PROJETO

| Componente | Status | Observações |
|------------|--------|-------------|
| Controle Motor VESC | ✅ Funcional | Testado e integrado |
| Controle Servo GPIO | ✅ Funcional | 2 implementações disponíveis |
| Interface Joystick | ✅ Funcional | Ackermann + Twist |
| Odometria | ✅ Funcional | Republicação para padrão F1TENTH |
| Calibração Servo | ✅ Funcional | Ferramenta interativa |
| Integração LiDAR | ⏳ Pendente | Código preparado, não testado |
| Navegação Autônoma | 🔄 Futuro | Dependente do LiDAR |

---

## 🚀 QUICK START

1. **Setup Inicial**: Siga [`11_SETUP_COMPLETO_RASPBERRY.md`](./configuracoes/11_SETUP_COMPLETO_RASPBERRY.md)
2. **Instalação**: Execute [`12_GUIA_INSTALACAO_DEPENDENCIAS.md`](./configuracoes/12_GUIA_INSTALACAO_DEPENDENCIAS.md)
3. **Calibração**: Use [`18_PROCEDIMENTOS_CALIBRACAO.md`](./manutencao/18_PROCEDIMENTOS_CALIBRACAO.md)
4. **Execução**: Lance com `ros2 launch f1tenth_control f1tenth_full.launch.py`

---

## 👥 INFORMAÇÕES PARA NOVOS COLABORADORES

### **Para Entender o Sistema**
1. Comece com [`01_ANALISE_ARQUITETURA_SISTEMA.md`](./analises/01_ANALISE_ARQUITETURA_SISTEMA.md)
2. Estude [`02_ANALISE_PACOTES_ROS2.md`](./analises/02_ANALISE_PACOTES_ROS2.md)
3. Revise [`03_ANALISE_FLUXO_COMUNICACAO.md`](./analises/03_ANALISE_FLUXO_COMUNICACAO.md)

### **Para Desenvolvimento**
1. Configure ambiente com [`11_SETUP_COMPLETO_RASPBERRY.md`](./configuracoes/11_SETUP_COMPLETO_RASPBERRY.md)
2. Consulte [`13_ROADMAP_DESENVOLVIMENTO.md`](./desenvolvimento/13_ROADMAP_DESENVOLVIMENTO.md)
3. Implemente seguindo padrões em [`15_OTIMIZACOES_PERFORMANCE.md`](./desenvolvimento/15_OTIMIZACOES_PERFORMANCE.md)

### **Para Troubleshooting**
1. Use [`17_GUIA_TROUBLESHOOTING.md`](./manutencao/17_GUIA_TROUBLESHOOTING.md)
2. Monitore com [`19_MONITORAMENTO_SISTEMA.md`](./manutencao/19_MONITORAMENTO_SISTEMA.md)

---

## 📞 CONTATO E CONTRIBUIÇÕES

**Maintainer**: Projeto F1TENTH UFABC
**Email**: gustavo.rio@aluno.ufabc.edu.br
**Repositório**: [f1tenth_code_rasp]
**Licença**: Em definição

---

*Documentação gerada automaticamente via análise de código - 2025-01-20*
