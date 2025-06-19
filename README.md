# F1TENTH Raspberry Pi ROS2 Project

## Visão Geral

Este repositório contém o código ROS2 para um veículo autônomo da categoria F1TENTH, projetado para operar em um Raspberry Pi 4B. O objetivo é implementar a navegação autônoma, incluindo percepção, planejamento e controle.

## Plataforma Alvo

- **Hardware:** Raspberry Pi 4B
- **Sistema Operacional:** Ubuntu Server 22.04 (ou compatível)
- **ROS Distro:** ROS 2 Humble Hawksbill (recomendado)

## Status Atual

- **Controle do Motor:** Integração com o VESC via pacote `f1tenth/vesc` funcional, utilizando os nós `ackermann_to_vesc` e `vesc_to_odom`.
- **Controle do Servo de Direção:** Implementado no pacote `f1tenth_control` (`servo_control_node.py`). O nó assina `/drive` e controla o servo via GPIO utilizando a biblioteca `pigpio`.
- **Odometria:** Publicada no tópico `/ego_racecar/odom` pelo `servo_control_node` (republicando dados do `vesc_to_odom`) com a transformação TF (`odom` -> `base_link`) correspondente.
- **Lidar:** Integração pendente. O driver YDLIDAR está no workspace, mas sua inicialização está comentada no launch file principal. A transformação TF estática (`base_link` -> `laser_frame`) ainda precisa ser configurada.
- **Controle Remoto:** Funcional através do pacote `Joy_converter` e nós relacionados.

## Estrutura de Tópicos ROS2 (Esperada - Padrão F1TENTH Sim)

A comunicação entre os nós seguirá o padrão estabelecido pelo simulador F1TENTH para garantir compatibilidade e facilitar a transição entre simulação e realidade.

**Tópicos Publicados pelo Robô:**

- `/scan` ([sensor_msgs/msg/LaserScan](http://docs.ros.org/en/humble/api/sensor_msgs/msg/LaserScan.html)): Leitura do Lidar do agente ego.
- `/ego_racecar/odom` ([nav_msgs/msg/Odometry](http://docs.ros.org/en/humble/api/nav_msgs/msg/Odometry.html)): Odometria do agente ego (pode ser estimada pelo VESC ou fusão de sensores).
- `/map` ([nav_msgs/msg/OccupancyGrid](http://docs.ros.org/en/humble/api/nav_msgs/msg/OccupancyGrid.html)): Mapa do ambiente (geralmente publicado por um nó SLAM).
- Árvore TF (`tf` e `tf_static`): Transformações entre os frames do robô (`base_link`, `laser_frame`, etc.).

_(Tópicos adicionais para múltiplos agentes são omitidos por enquanto, mas podem ser encontrados na documentação F1TENTH)_

**Tópicos Assinados pelo Robô:**

- `/drive` ([ackermann_msgs/msg/AckermannDriveStamped](https://github.com/ros-drivers/ackermann_msgs/blob/ros2/msg/AckermannDriveStamped.msg)): Comando de controle de velocidade e ângulo de direção para o agente ego. Assinado pelo `ackermann_to_vesc_node` (para velocidade) e `servo_control_node` (para direção).
- `/joy` ([sensor_msgs/msg/Joy](http://docs.ros.org/en/humble/api/sensor_msgs/msg/Joy.html)): Mensagens do controle joystick (geralmente para o `joy_node`).
- `/odom` ([nav_msgs/msg/Odometry](http://docs.ros.org/en/humble/api/nav_msgs/msg/Odometry.html)): Odometria publicada pelo `vesc_to_odom_node`, assinada pelo `servo_control_node` para republicação.

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

- O workspace contém os pacotes `vesc` (driver original), `ydlidar_ros2_driver`, e pacotes customizados:
  - `vesc_config`: Contém configurações e launch file para o driver VESC.
  - `f1tenth_control`: Contém o nó principal `servo_control_node.py` (para controle de servo e republicação de odometria/TF) e os launch files do projeto.
  - `Joy_converter`: Para converter comandos do joystick em mensagens `/drive`.
- Os arquivos de configuração (`vesc_config.yaml`, `ydlidar.yaml`, e parâmetros dentro dos nós/launch files) devem ser ajustados conforme o hardware específico.
- O launch file principal (`src/f1tenth_control/launch/f1tenth_full.launch.py`) inicia os drivers (VESC), nós de controle (ackermann, servo) e a interface do joystick. A inicialização do Lidar está presente mas comentada.
- Um arquivo `.gitignore` está configurado para ignorar arquivos de build, logs e outros arquivos temporários do Python e ROS 2, mantendo o repositório limpo.

## Trabalhos Futuros

**1. Integração e Interpretação do Lidar**

- **Configuração:**
  - Descomentar a seção do Lidar no launch file principal (`f1tenth_full.launch.py`).
  - Garantir que o driver YDLIDAR esteja corretamente configurado (`ydlidar.yaml`) e publicando no tópico `/scan`.
  - **Frame ID e TF:** Verificar se o `frame_id` no arquivo de configuração do Lidar (e.g., `laser_frame`) está correto.
  - **Criar e adicionar um publicador de transformação estática** (Static TF Publisher) ao launch file principal para publicar a transformação entre o frame base do robô (`base_link` ou `ego_racecar/base_link`) e o `laser_frame`. Isso é crucial para que outros nós entendam a posição do Lidar em relação ao robô. Exemplo de nó a adicionar no launch file:
    ```python
    Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_link_to_laser_frame',
        arguments=['0.1', '0.0', '0.2', '0', '0', '0', 'base_link', 'laser_frame'] # Exemplo: X, Y, Z, Roll, Pitch, Yaw, parent_frame, child_frame
        # Ajuste os valores de translação (X, Y, Z) e rotação (Roll, Pitch, Yaw)
        # para corresponder à posição física do Lidar no robô.
    ),
    ```
- **Utilização dos Dados:** Após a integração, os dados do `/scan` serão a base para:
  - **SLAM (Simultaneous Localization and Mapping):** Usar pacotes como `slam_toolbox` ou `cartographer` para construir mapas (`/map`) e localizar o robô dentro deles.
  - **Detecção de Obstáculos:** Implementar algoritmos para identificar obstáculos.
  - **Planejamento de Caminho e Desvio de Obstáculos:** Usar os dados de scan e/ou o mapa para planejar trajetórias seguras (e.g., usando `nav2`).

**2. Calibração e Ajustes Finos**

- Calibrar precisamente o mapeamento ângulo-PWM do servo no `servo_control_node.py` (parâmetros `servo_min_pulse_width`, `servo_max_pulse_width`, `min_steering_angle`, `max_steering_angle`).
- Verificar e ajustar os parâmetros do VESC (correntes, limites, etc.) em `vesc_config.yaml`.
- Ajustar os parâmetros de odometria no `vesc_to_odom_node` (e.g., `wheelbase`, `publish_tf`) se necessário.

## Construindo e Executando o Projeto

```bash
# Navegue até a raiz do workspace
cd ~/ros2_ws

# Faça o source do setup do ROS 2 (adicione ao seu .bashrc para não precisar fazer sempre)
source /opt/ros/humble/setup.bash

# Construa todos os pacotes no workspace
colcon build --symlink-install # Usar --symlink-install é útil durante o desenvolvimento

# Faça o source do setup do seu workspace (necessário após cada build ou em novo terminal)
source install/setup.bash

# Execute o launch file principal
ros2 launch f1tenth_control f1tenth_full.launch.py
```

## Controle de Versão

Este projeto utiliza Git para controle de versão.

- O arquivo `.gitignore` na raiz do repositório está configurado para evitar o rastreamento de arquivos gerados automaticamente (builds, logs, caches, etc.). É importante mantê-lo atualizado caso novas ferramentas ou arquivos temporários sejam introduzidos.
- Recomenda-se fazer commits frequentes com mensagens descritivas para rastrear o progresso e facilitar a colaboração ou reversão de alterações.

---

_Este README é um documento vivo. Atualize-o conforme o projeto avança._
