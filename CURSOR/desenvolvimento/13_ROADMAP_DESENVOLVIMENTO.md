# 🗺️ ROADMAP DE DESENVOLVIMENTO F1TENTH

**Documento**: Plano de Desenvolvimento Futuro
**Versão**: 1.0
**Data**: 2025-01-20
**Timeframe**: 2025-2026
**Responsável**: Equipe F1TENTH UFABC

---

## 🎯 VISÃO DE LONGO PRAZO

### **Objetivos Estratégicos**
1. **Navegação Autônoma Completa**: Implementar sistema SLAM + Path Planning
2. **Multi-Robot Systems**: Suporte para múltiplos veículos F1TENTH
3. **AI Integration**: Integração com frameworks de aprendizado
4. **Research Platform**: Base para pesquisa em robótica móvel
5. **Educational Tool**: Ferramenta para ensino de robótica

### **Principais Marcos**
- **Q1 2025**: Integração LiDAR + SLAM básico
- **Q2 2025**: Navegação autônoma em ambientes estruturados
- **Q3 2025**: Sistemas multi-agente básicos
- **Q4 2025**: Integração AI/ML para tomada de decisão
- **Q1 2026**: Plataforma educacional completa

---

## 📅 FASE 1: INTEGRAÇÃO SENSORIAL (Q1 2025)

### **1.1 Integração LiDAR YDLiDAR**
**Prioridade**: 🔴 Alta
**Estimativa**: 3-4 semanas
**Responsável**: Hardware Team

#### **Tarefas Principais**
- [ ] **Teste Hardware YDLiDAR**
  - Verificar comunicação USB/Serial
  - Calibrar parâmetros físicos (range, resolução)
  - Integrar alimentação e montagem física

- [ ] **Configuração Driver ROS2**
  - Descomentar seção LiDAR em launch files
  - Configurar parâmetros YDLiDAR específicos
  - Implementar static TF publisher (base_link → laser_frame)

- [ ] **Validação Dados**
  - Verificar qualidade dados `/scan`
  - Implementar filtros de ruído
  - Calibrar transformação espacial

#### **Entregáveis**
```yaml
- Hardware: LiDAR montado e funcional
- Software: Driver ydlidar_ros2_driver configurado
- Config: Arquivo vesc_lidar_config.yaml
- Test: Scripts validação dados /scan
```

#### **Critérios de Aceitação**
- [ ] Dados `/scan` publicados consistentemente a 10Hz
- [ ] Range detection funcional (0.1m - 12m)
- [ ] TF tree completa (map → odom → base_link → laser_frame)
- [ ] Visualização RViz funcionando

### **1.2 Melhoria Sistema Odometria**
**Prioridade**: 🟡 Média
**Estimativa**: 2 semanas

#### **Melhorias Planejadas**
- **Fusão Sensorial**: Combinar odometria VESC + IMU (futuro)
- **Calibração Avançada**: Wheelbase dinâmico e slip compensation
- **Filtering**: Implementar Kalman Filter básico
- **Accuracy**: Melhorar precisão estimativa velocidade angular

#### **Implementação**
```python
# Extended Kalman Filter para odometria
class EKFOdometry:
    def __init__(self):
        self.state = [x, y, theta, v, omega]  # Estado estimado
        self.P = np.eye(5)  # Covariance matrix

    def predict(self, dt, u_linear, u_angular):
        # Predição modelo cinemático

    def update(self, vesc_measurement):
        # Atualização com dados VESC
```

---

## 🗺️ FASE 2: SLAM E MAPEAMENTO (Q2 2025)

### **2.1 Implementação SLAM 2D**
**Prioridade**: 🔴 Alta
**Estimativa**: 4-6 semanas
**Dependências**: Fase 1 completa

#### **Approach Técnico**
- **Framework**: `slam_toolbox` (ROS2 padrão)
- **Algorithm**: Online SLAM com loop closure
- **Map Format**: OccupancyGrid (nav_msgs/OccupancyGrid)

#### **Implementação Planejada**
```yaml
# slam_config.yaml
slam_toolbox:
  ros__parameters:
    mode: "mapping"
    resolution: 0.05
    minimum_travel_distance: 0.1
    minimum_travel_heading: 0.1
    loop_closure_search_space: 3.0
    scan_buffer_size: 10
```

#### **Integration Points**
- **Input**: `/scan` (LiDAR) + `/ego_racecar/odom` (odometria)
- **Output**: `/map` (OccupancyGrid) + TF (map → odom)
- **Services**: Save/load maps, relocalization

### **2.2 Sistema de Localização**
**Prioridade**: 🟡 Média
**Estimativa**: 2-3 semanas

#### **Componentes**
- **AMCL**: Adaptive Monte Carlo Localization
- **Map Loading**: Sistema carregamento mapas salvos
- **Relocalization**: Recovery em caso de kidnapped robot

#### **Arquivos de Configuração**
```python
# amcl_config.yaml
amcl:
  ros__parameters:
    laser_max_beams: 60
    laser_max_range: 12.0
    min_particles: 200
    max_particles: 2000
    recovery_alpha_slow: 0.001
    recovery_alpha_fast: 0.1
```

---

## 🧠 FASE 3: NAVEGAÇÃO AUTÔNOMA (Q2-Q3 2025)

### **3.1 Path Planning Global**
**Prioridade**: 🔴 Alta
**Estimativa**: 4 semanas

#### **Algoritmos Planejados**
- **A\* / Dijkstra**: Para planejamento global
- **RRT\***: Para ambientes mais complexos
- **Costmap Integration**: Integração com occupancy grids

#### **Implementation Stack**
```yaml
Framework: Nav2 (ROS2 Navigation Stack)
Planners:
  - nav2_navfn_planner (A*)
  - nav2_smac_planner (Hybrid A*)
Controllers:
  - nav2_regulated_pure_pursuit_controller
  - nav2_dwb_controller
```

### **3.2 Obstacle Avoidance Local**
**Prioridade**: 🔴 Alta
**Estimativa**: 3 semanas

#### **Strategies**
- **DWA**: Dynamic Window Approach para controle local
- **Pure Pursuit**: Para seguimento de trajetórias suaves
- **Emergency Stop**: Sistema parada de emergência

#### **Safety Features**
- **Collision Detection**: Sistema tempo real
- **Speed Modulation**: Redução velocidade próximo obstáculos
- **Recovery Behaviors**: Comportamentos recuperação

### **3.3 Behavior Planning**
**Prioridade**: 🟡 Média
**Estimativa**: 3-4 semanas

#### **State Machine**
```python
class NavigationBehavior(Enum):
    IDLE = "idle"
    MAPPING = "mapping"
    NAVIGATING = "navigating"
    AVOIDING_OBSTACLE = "avoiding"
    RECOVERY = "recovery"
    EMERGENCY_STOP = "emergency"
```

#### **Mission Planning**
- **Waypoint Navigation**: Navegação por pontos de interesse
- **Area Coverage**: Cobertura sistemática de áreas
- **Return to Base**: Retorno automático posição inicial

---

## 🤖 FASE 4: SISTEMAS MULTI-AGENTE (Q3-Q4 2025)

### **4.1 Communication Framework**
**Prioridade**: 🟡 Média
**Estimativa**: 4-5 semanas

#### **Architecture**
- **Namespace Strategy**: Separação por agente (`/agent_N/`)
- **Central Coordinator**: Nó central coordenação
- **Distributed Planning**: Planejamento distribuído

#### **Communication Topics**
```yaml
# Per-agent topics
/agent_1/scan
/agent_1/odom
/agent_1/drive
/agent_1/goal

# Global coordination
/global/agent_states
/global/coordination_commands
/global/shared_map
```

### **4.2 Collision Avoidance Multi-Agent**
**Prioridade**: 🔴 Alta (para multi-agent)
**Estimativa**: 3-4 semanas

#### **Algorithms**
- **ORCA**: Optimal Reciprocal Collision Avoidance
- **Priority-based**: Sistema prioridades dinâmicas
- **Negotiation**: Protocolo negociação trajetórias

### **4.3 Cooperative Behaviors**
**Prioridade**: 🟢 Baixa
**Estimativa**: 6+ semanas

#### **Scenarios**
- **Formation Control**: Manutenção formações
- **Area Coverage**: Cobertura cooperativa
- **Leader-Follower**: Seguimento de líder

---

## 🧠 FASE 5: INTEGRAÇÃO AI/ML (Q4 2025 - Q1 2026)

### **5.1 Reinforcement Learning**
**Prioridade**: 🟢 Baixa
**Estimativa**: 8-10 semanas

#### **Applications**
- **Autonomous Racing**: RL para racing otimizado
- **Dynamic Obstacle Avoidance**: Aprendizado comportamentos
- **Energy Optimization**: Otimização consumo energético

#### **Framework Integration**
```python
# Example: ROS2 + PyTorch integration
class RLNavigationNode(Node):
    def __init__(self):
        # ROS2 setup
        self.scan_sub = self.create_subscription(...)
        self.cmd_pub = self.create_publisher(...)

        # RL setup
        self.model = PolicyNetwork()
        self.env = F1TENTHEnvironment()
```

### **5.2 Computer Vision**
**Prioridade**: 🟡 Média
**Estimativa**: 6 semanas

#### **Capabilities**
- **Camera Integration**: Adição câmera RGB
- **Object Detection**: Detecção objetos específicos
- **Lane Detection**: Detecção faixas (indoor racing)
- **Visual SLAM**: SLAM visual complementar

### **5.3 Simulation Integration**
**Prioridade**: 🟡 Média
**Estimativa**: 4 semanas

#### **Simulators**
- **F1TENTH Gym**: Integração simulador oficial
- **Gazebo**: Simulação física detalhada
- **Unity/Unreal**: Simulação visual realística

---

## 📚 FASE 6: PLATAFORMA EDUCACIONAL (Q1 2026)

### **6.1 Educational Framework**
**Prioridade**: 🟡 Média
**Estimativa**: 6-8 semanas

#### **Components**
- **Lab Exercises**: Roteiros práticos estruturados
- **Simulation Labs**: Exercícios simulação
- **Competition Tools**: Ferramentas para competições
- **Documentation**: Material didático completo

#### **Course Integration**
```yaml
Courses:
  - "Introdução à Robótica Móvel"
  - "Sistemas de Controle Autônomo"
  - "SLAM e Navegação"
  - "Sistemas Multi-Robô"
  - "Machine Learning em Robótica"
```

### **6.2 Research Platform**
**Prioridade**: 🟢 Baixa
**Estimativa**: Ongoing

#### **Research Areas**
- **Motion Planning Algorithms**
- **Multi-Agent Coordination**
- **Human-Robot Interaction**
- **Edge Computing in Robotics**

---

## 🛠️ INFRAESTRUTURA E FERRAMENTAS

### **Development Tools**
- **CI/CD**: GitHub Actions para build automático
- **Testing**: Framework testes automatizados
- **Documentation**: Auto-geração documentação
- **Monitoring**: Sistema monitoramento performance

### **Hardware Evolution**
- **Raspberry Pi 5**: Upgrade para melhor performance
- **Additional Sensors**: IMU, câmeras, ultrasônicos
- **Better VESC**: Upgrade controlador motor
- **Enhanced Chassis**: Melhorias mecânicas

---

## 📊 MÉTRICAS DE SUCESSO

### **Technical KPIs**
- **Navigation Accuracy**: <10cm erro posicionamento
- **Real-time Performance**: <100ms latência comando
- **System Reliability**: >99% uptime em operação
- **Multi-Agent Coordination**: 4+ robôs simultâneos

### **Educational KPIs**
- **Student Engagement**: Feedback qualitativo
- **Learning Outcomes**: Avaliações específicas
- **Research Publications**: Papers gerados
- **Community Adoption**: Uso por outras instituições

---

## 💡 CONSIDERAÇÕES TÉCNICAS

### **Performance Optimization**
- **Real-time Optimization**: SCHED_FIFO, CPU isolation
- **Memory Management**: Otimização uso RAM
- **Network Optimization**: DDS tuning
- **Power Management**: Otimização consumo

### **Scalability Considerations**
- **Modular Architecture**: Componentes intercambiáveis
- **Configuration Management**: Sistema configuração unificado
- **Hardware Abstraction**: Suporte múltiplas plataformas
- **Cloud Integration**: Integração serviços cloud

---

## 🎯 PRÓXIMOS PASSOS IMEDIATOS

### **Janeiro 2025**
1. **LiDAR Hardware Setup** (Semana 1-2)
2. **Driver Configuration** (Semana 2-3)
3. **Integration Testing** (Semana 3-4)

### **Fevereiro 2025**
1. **SLAM Implementation** (Semana 1-3)
2. **Map Building Tests** (Semana 3-4)
3. **Documentation Update** (Semana 4)

### **Março 2025**
1. **Navigation Stack** (Semana 1-3)
2. **Autonomous Testing** (Semana 3-4)
3. **System Integration** (Semana 4)

---

## 🚀 RECURSOS NECESSÁRIOS

### **Hardware**
- **YDLiDAR X4**: ~$99 USD
- **IMU Module**: ~$30 USD
- **RGB Camera**: ~$50 USD
- **Additional Raspberry Pi**: ~$100 USD (multi-agent)

### **Software/Tools**
- **Development Licenses**: Simulação tools
- **Cloud Services**: Para CI/CD e storage
- **Testing Infrastructure**: Hardware de teste

### **Human Resources**
- **Lead Developer**: Coordenação técnica
- **Hardware Specialist**: Integração sensores
- **Algorithm Developer**: SLAM/Navigation
- **Documentation Specialist**: Material educacional

---

**ROADMAP APROVADO E EM EXECUÇÃO** 🚀

Este roadmap será revisado trimestralmente e atualizado conforme progresso e novas necessidades identificadas. Para contribuições ou sugestões, consulte a equipe de desenvolvimento.

---

*Roadmap atualizado em 2025-01-20 - Versão 1.0*
