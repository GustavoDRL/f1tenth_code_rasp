# F1TENTH Raspberry Pi ROS2 Project

## Visão Geral

Este repositório contém o código ROS2 para um veículo autônomo da categoria F1TENTH, projetado para operar em um Raspberry Pi 4B. O objetivo é implementar a navegação autônoma, incluindo percepção, planejamento e controle.

## Plataforma Alvo

- **Hardware:** Raspberry Pi 4B
- **Sistema Operacional:** Ubuntu Server 22.04 (ou compatível)
- **ROS Distro:** ROS 2 Humble Hawksbill (recomendado)

## Status Atual

- **Controle do Motor:** Integração com o VESC via pacote `f1tenth/vesc` funcional.
- **Lidar:** Integração pendente.
- **Servo de Direção:** Controle pendente.

## Estrutura de Tópicos ROS2 (Esperada - Padrão F1TENTH Sim)

A comunicação entre os nós seguirá o padrão estabelecido pelo simulador F1TENTH para garantir compatibilidade e facilitar a transição entre simulação e realidade.

**Tópicos Publicados pelo Robô:**

- `/scan` ([sensor_msgs/msg/LaserScan](http://docs.ros.org/en/humble/api/sensor_msgs/msg/LaserScan.html)): Leitura do Lidar do agente ego.
- `/ego_racecar/odom` ([nav_msgs/msg/Odometry](http://docs.ros.org/en/humble/api/nav_msgs/msg/Odometry.html)): Odometria do agente ego (pode ser estimada pelo VESC ou fusão de sensores).
- `/map` ([nav_msgs/msg/OccupancyGrid](http://docs.ros.org/en/humble/api/nav_msgs/msg/OccupancyGrid.html)): Mapa do ambiente (geralmente publicado por um nó SLAM).
- Árvore TF (`tf` e `tf_static`): Transformações entre os frames do robô (`base_link`, `laser_frame`, etc.).

_(Tópicos adicionais para múltiplos agentes são omitidos por enquanto, mas podem ser encontrados na documentação F1TENTH)_

**Tópicos Assinados pelo Robô:**

- `/drive` ([ackermann_msgs/msg/AckermannDriveStamped](https://github.com/ros-drivers/ackermann_msgs/blob/ros2/msg/AckermannDriveStamped.msg)): Comando de controle de velocidade e ângulo de direção para o agente ego.

_(Tópicos de controle adicionais como `/initialpose` são geralmente usados com RViz e não diretamente publicados por nós de controle)_

## Instalação de Drivers e Dependências

**Pré-requisitos:**

- ROS 2 Humble instalado.
- `colcon`, `git`, `rosdep` instalados.
- Permissões de usuário para acesso a portas seriais (`sudo usermod -a -G dialout $USER`).

**1. Driver VESC (f1tenth/vesc)**

Este pacote permite a comunicação com o controlador de motor VESC.

```bash
# Navegue até o diretório src do seu workspace ROS2
cd ~/ros2_ws/src

# Clone o repositório (branch ros2)
git clone https://github.com/f1tenth/vesc.git -b ros2

# Volte para a raiz do workspace
cd ..

# Instale dependências
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Construa o pacote
colcon build --packages-select vesc_driver vesc_msgs vesc_ackermann

# Configure a porta serial e outros parâmetros (IMPORTANTE!)
# Edite o arquivo: ~/ros2_ws/src/vesc/vesc_driver/params/vesc_config.yaml
# Certifique-se que 'port' corresponde à porta onde o VESC está conectado (e.g., /dev/ttyACM0)

# Dê permissão à porta serial (se necessário, substitua ttyACM0 pela porta correta)
sudo chmod 777 /dev/ttyACM0

# Para testar, rode o driver (após source install/setup.bash)
# ros2 launch vesc_driver vesc_driver_node.launch.py
```

_Referência: [https://github.com/f1tenth/vesc/tree/ros2](https://github.com/f1tenth/vesc/tree/ros2)_

**2. Driver Lidar (YDLIDAR)**

Este pacote fornece o nó para comunicação com Lidars da YDLIDAR.

```bash
# Navegue até o diretório src do seu workspace ROS2
cd ~/ros2_ws/src

# Clone o repositório
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git

# Volte para a raiz do workspace
cd ..

# Instale dependências
rosdep update # Já deve ter sido feito, mas não custa
rosdep install --from-paths src --ignore-src -r -y

# Construa o pacote
colcon build --packages-select ydlidar_ros2_driver

# Configure a porta serial, modelo do Lidar e outros parâmetros (IMPORTANTE!)
# Edite o arquivo de parâmetros (e.g., ~/ros2_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml)
# Ajuste 'port', 'baudrate', 'lidar_type', 'frame_id' (e.g., 'laser_frame'), 'angle_min', 'angle_max', etc.
# Consulte a documentação do YDLIDAR para os valores corretos do seu modelo.

# Dê permissão à porta serial (se necessário, substitua ttyUSB0 pela porta correta)
sudo chmod 777 /dev/ttyUSB0

# Para testar, rode o driver (após source install/setup.bash e ajuste do launch file se necessário)
# ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

_Referência: [https://github.com/YDLIDAR/ydlidar_ros2_driver](https://github.com/YDLIDAR/ydlidar_ros2_driver)_

## Configuração Atual do Projeto

_(Esta seção deve ser atualizada conforme o projeto evolui)_

- O workspace contém os pacotes `vesc_driver`, `vesc_msgs`, `vesc_ackermann` e `ydlidar_ros2_driver` no diretório `src/`.
- _(Adicione aqui outros pacotes customizados do seu projeto quando existirem)_
- Os arquivos de configuração (`vesc_config.yaml`, `ydlidar.yaml`) devem ser ajustados conforme o hardware específico.
- Um launch file principal (`.launch.py`) deve ser criado para iniciar todos os nós necessários (VESC, Lidar, Servo, controle, etc.).

## Trabalhos Futuros

**1. Controle do Servo Motor de Direção**

- **Desafio:** O tópico `/drive` (`AckermannDriveStamped`) contém `steering_angle`, mas o VESC (geralmente) controla apenas a velocidade (`speed`). Um mecanismo separado é necessário para comandar o servo de direção conectado aos pinos GPIO do Raspberry Pi.
- **Solução Proposta:**
  - Criar um nó ROS2 simples (Python é recomendado pela facilidade de uso com GPIO).
  - Este nó assinará o tópico `/drive`.
  - Ao receber uma mensagem, extrairá o `steering_angle`.
  - Converterá o ângulo (radianos) para um valor de PWM (Pulse Width Modulation) adequado para o servo motor. Isso requer calibração para mapear ângulos para larguras de pulso (e.g., 1000µs a 2000µs).
  - Utilizará uma biblioteca Python para controle de GPIO/PWM no Raspberry Pi:
    - **`RPi.GPIO`:** Padrão, mas pode ter jitter no PWM por software.
    - **`pigpio`:** Oferece PWM por hardware (mais estável), requer um daemon (`sudo systemctl start pigpiod`). É geralmente a **opção preferida** para controle preciso de servos.
    - **`gpiozero`:** Abstração de alto nível sobre `RPi.GPIO` ou `pigpio`.
  - O nó publicará o sinal PWM para o pino GPIO conectado ao servo.
- **Considerações:**
  - Alimentação do servo (geralmente requer uma fonte externa de 5V, não diretamente dos pinos 5V do RPi que podem não fornecer corrente suficiente).
  - Calibração cuidadosa do mapeamento ângulo-PWM.
  - Definição de limites de ângulo para evitar danos mecânicos.

**2. Integração e Interpretação do Lidar**

- **Configuração:** Garantir que o driver YDLIDAR esteja corretamente configurado (`ydlidar.yaml`) e publicando no tópico `/scan`.
- **Frame ID e TF:** Verificar se o `frame_id` no arquivo de configuração do Lidar (e.g., `laser_frame`) está correto e se existe uma transformação estática (publicada via `tf_static` publisher, geralmente em um launch file) entre o frame base do robô (`base_link` ou `ego_racecar/base_link`) e o `laser_frame`. Isso é crucial para que outros nós entendam a posição do Lidar em relação ao robô.
- **Utilização dos Dados:** Os dados do `/scan` são a base para:
  - **SLAM (Simultaneous Localization and Mapping):** Usar pacotes como `slam_toolbox` ou `cartographer` para construir mapas (`/map`) e localizar o robô dentro deles.
  - **Detecção de Obstáculos:** Implementar algoritmos simples ou complexos para identificar obstáculos próximos.
  - **Planejamento de Caminho e Desvio de Obstáculos:** Usar os dados de scan e/ou o mapa para planejar trajetórias seguras (e.g., usando `nav2`).

## Construindo e Executando o Projeto

```bash
# Navegue até a raiz do workspace
cd ~/ros2_ws

# Faça o source do setup do ROS 2 (adicione ao seu .bashrc para não precisar fazer sempre)
source /opt/ros/humble/setup.bash

# Construa todos os pacotes no workspace
colcon build

# Faça o source do setup do seu workspace (necessário após cada build ou em novo terminal)
source install/setup.bash

# Execute o launch file principal (que você precisará criar)
# Exemplo: ros2 launch seu_pacote_launch seu_launch_file.launch.py
```

---

_Este README é um documento vivo. Atualize-o conforme o projeto avança._
