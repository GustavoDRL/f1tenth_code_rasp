# Plano de Implementação: Controle Integrado F1TENTH (VESC + Servo) em Raspberry Pi

## Contexto Atual

O projeto atual já possui:

1. Driver VESC (`vesc_driver`, pacote ROS2 oficial F1TENTH) configurado para controlar o motor
2. Pacote de conversão de comandos Ackermann para VESC (`vesc_ackermann`)
3. Conversão de Joystick para comandos Ackermann (`Joy_converter`)
4. Driver do Lidar YDLidar pendente para integração

## Objetivo

Desenvolver um nó ROS2 Python que:

1. Subscreva ao tópico `/drive` (AckermannDriveStamped)
2. Controle a velocidade através da VESC (já funciona via `vesc_ackermann`)
3. **Controle o ângulo de direção com um servo via GPIO do Raspberry Pi**
4. Publique odometria no formato `/ego_racecar/odom` (padrão da F1TENTH)

## Tarefas de Implementação

### 1. Criar Pacote ROS2 para Controle Integrado

```bash
# Navegue até o diretório src
cd ~/workspace/src

# Crie o pacote ROS2 Python
ros2 pkg create --build-type ament_python f1tenth_control --dependencies rclpy ackermann_msgs sensor_msgs std_msgs geometry_msgs nav_msgs
```

### 2. Implementar Nó de Controle do Servo Motor

#### 2.1 Instalar Dependências para GPIO

```bash
# Instalar a biblioteca pigpio (preferida para controle preciso de PWM)
sudo apt-get update
sudo apt-get install -y pigpio python3-pigpio

# Garantir que o daemon do pigpio inicie na inicialização
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

#### 2.2 Criar Arquivo de Configuração dos Parâmetros

Criar `~/workspace/src/f1tenth_control/config/control_params.yaml`:

```yaml
f1tenth_control:
  ros__parameters:
    # GPIO para controle do servo de direção
    servo_gpio_pin: 18 # Pino GPIO BCM para o sinal PWM do servo
    servo_pwm_frequency: 50 # Frequência do PWM (Hz)
    servo_min_pulse_width: 1000 # Largura de pulso mínima (µs)
    servo_max_pulse_width: 2000 # Largura de pulso máxima (µs)

    # Parâmetros de conversão
    max_steering_angle: 0.4 # Ângulo máximo de direção (rad)
    min_steering_angle: -0.4 # Ângulo mínimo de direção (rad)

    # Nome dos frames (devem corresponder aos frames usados no nó vesc_to_odom)
    odom_frame: "odom"
    base_frame: "base_link"

    # Tópicos ROS
    drive_topic: "/drive" # Tópico para comandos Ackermann
    odom_topic: "/ego_racecar/odom" # Tópico para publicação de odometria (padrão F1TENTH)

    # Reutilizar cálculos de odometria do vesc_to_odom ou calcular independentemente
    use_vesc_odom: true # Se true, irá subscrever aos tópicos de odometria do vesc_to_odom
    vesc_odom_topic: "/odom" # Tópico de odometria calculado pelo vesc_to_odom
```

#### 2.3 Implementar o Nó de Controle em Python

Criar `~/workspace/src/f1tenth_control/f1tenth_control/servo_control_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped
import pigpio
import math
import time
from tf2_ros import TransformBroadcaster

class ServoControlNode(Node):
    """
    Nó ROS2 para controle integrado do VESC e servo em um Raspberry Pi
    para veículos F1TENTH.
    """

    def __init__(self):
        super().__init__('servo_control_node')

        # Declarar e obter parâmetros
        self.declare_parameters(
            namespace='',
            parameters=[
                ('servo_gpio_pin', 18),
                ('servo_pwm_frequency', 50),
                ('servo_min_pulse_width', 1000),
                ('servo_max_pulse_width', 2000),
                ('max_steering_angle', 0.4),
                ('min_steering_angle', -0.4),
                ('odom_frame', 'odom'),
                ('base_frame', 'base_link'),
                ('drive_topic', '/drive'),
                ('odom_topic', '/ego_racecar/odom'),
                ('use_vesc_odom', True),
                ('vesc_odom_topic', '/odom')
            ]
        )

        # Obter parâmetros
        self.servo_gpio_pin = self.get_parameter('servo_gpio_pin').value
        self.servo_pwm_frequency = self.get_parameter('servo_pwm_frequency').value
        self.servo_min_pulse_width = self.get_parameter('servo_min_pulse_width').value
        self.servo_max_pulse_width = self.get_parameter('servo_max_pulse_width').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.min_steering_angle = self.get_parameter('min_steering_angle').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.use_vesc_odom = self.get_parameter('use_vesc_odom').value
        self.vesc_odom_topic = self.get_parameter('vesc_odom_topic').value

        # Conectar ao daemon pigpio
        try:
            self.pi = pigpio.pi()
            if not self.pi.connected:
                self.get_logger().error('Não foi possível conectar ao daemon pigpio. Está instalado e executando?')
                self.get_logger().error('Execute: sudo systemctl start pigpiod')
                return

            # Configurar o pino GPIO para o servo
            self.pi.set_mode(self.servo_gpio_pin, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(self.servo_gpio_pin, self.servo_pwm_frequency)

            # Centralizar o servo na inicialização
            self.set_servo_angle(0.0)

            self.get_logger().info(f'Servo inicializado no pino GPIO {self.servo_gpio_pin}')
        except Exception as e:
            self.get_logger().error(f'Erro ao inicializar GPIO: {e}')
            return

        # Criar subscription para o tópico de drive (comandos Ackermann)
        self.drive_subscription = self.create_subscription(
            AckermannDriveStamped,
            self.drive_topic,
            self.drive_callback,
            10)

        # Se estamos usando a odometria do VESC, subscrever-se ao tópico
        if self.use_vesc_odom:
            self.odom_subscription = self.create_subscription(
                Odometry,
                self.vesc_odom_topic,
                self.odom_callback,
                10)

        # Publisher para odometria no formato F1TENTH
        self.odom_publisher = self.create_publisher(
            Odometry,
            self.odom_topic,
            10)

        # Broadcaster TF para transformações
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('Nó de controle do servo inicializado')

    def set_servo_angle(self, angle):
        """
        Converte um ângulo de direção (em radianos) para um valor de PWM e envia para o servo.

        Args:
            angle (float): Ângulo de direção em radianos
        """
        # Limitar o ângulo aos valores mínimo e máximo
        angle = min(max(angle, self.min_steering_angle), self.max_steering_angle)

        # Mapear o ângulo para a faixa de largura de pulso
        # Supõe que o ângulo 0 está no centro do servo
        angle_range = self.max_steering_angle - self.min_steering_angle
        pulse_range = self.servo_max_pulse_width - self.servo_min_pulse_width

        # Normalizar o ângulo para um valor entre 0 e 1
        normalized_angle = (angle - self.min_steering_angle) / angle_range

        # Converter para largura de pulso em microssegundos
        pulse_width = self.servo_min_pulse_width + normalized_angle * pulse_range

        # Converter para duty_cycle que o pigpio espera (0-255)
        # A fórmula é duty_cycle = pulse_width / (1000000 / freq) * range
        # Para range = 255 (8 bits) e freq em Hz
        duty_cycle_8bit = pulse_width / (1000000.0 / self.servo_pwm_frequency) * 255

        # Aplicar ao pino GPIO
        self.pi.set_PWM_dutycycle(self.servo_gpio_pin, int(duty_cycle_8bit))

    def drive_callback(self, msg):
        """
        Processa mensagens AckermannDriveStamped do tópico /drive

        Args:
            msg (AckermannDriveStamped): Mensagem com comandos de velocidade e ângulo de direção
        """
        # O controle de velocidade já é tratado pelo vesc_ackermann
        # Aqui apenas controlamos o servo diretamente
        steering_angle = msg.drive.steering_angle

        # Ajustar o ângulo do servo via PWM
        self.set_servo_angle(steering_angle)

        self.get_logger().debug(f'Recebido comando de direção: {steering_angle} rad')

    def odom_callback(self, msg):
        """
        Recebe a odometria do vesc_to_odom e a republica no formato F1TENTH

        Args:
            msg (Odometry): Mensagem de odometria do vesc_to_odom
        """
        # Criar uma nova mensagem de odometria no formato esperado pela F1TENTH
        f1tenth_odom = Odometry()

        # Copiar os dados da mensagem original
        f1tenth_odom.header.stamp = self.get_clock().now().to_msg()
        f1tenth_odom.header.frame_id = self.odom_frame
        f1tenth_odom.child_frame_id = self.base_frame

        # Copiar pose e twist
        f1tenth_odom.pose = msg.pose
        f1tenth_odom.twist = msg.twist

        # Publicar a odometria no tópico F1TENTH
        self.odom_publisher.publish(f1tenth_odom)

        # Publicar transformação tf
        transform = TransformStamped()
        transform.header.stamp = f1tenth_odom.header.stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame

        # Posição
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z

        # Orientação
        transform.transform.rotation = msg.pose.pose.orientation

        # Publicar transformação
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    try:
        servo_control_node = ServoControlNode()
        rclpy.spin(servo_control_node)
    except Exception as e:
        print(f'Erro no nó de controle do servo: {e}')
    finally:
        # Desligar corretamente
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 2.4 Criar Launch File

Criar `~/workspace/src/f1tenth_control/launch/f1tenth_control.launch.py`:

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Obter diretório do pacote
    pkg_dir = get_package_share_directory('f1tenth_control')

    # Arquivo de configuração
    config_file = os.path.join(pkg_dir, 'config', 'control_params.yaml')

    # Nó de controle do servo
    servo_control_node = Node(
        package='f1tenth_control',
        executable='servo_control_node',
        name='servo_control_node',
        parameters=[config_file],
        output='screen'
    )

    # Launch file completo
    return LaunchDescription([
        servo_control_node,
    ])
```

#### 2.5 Atualizar `setup.py` para Incluir os Arquivos Necessários

Editar `~/workspace/src/f1tenth_control/setup.py`:

```python
from setuptools import setup
from glob import glob
import os

package_name = 'f1tenth_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seu_nome',
    maintainer_email='seu_email@example.com',
    description='Controle integrado F1TENTH para Raspberry Pi',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_control_node = f1tenth_control.servo_control_node:main',
        ],
    },
)
```

### 3. Integrar com Nós Existentes

#### 3.1 Criar um Launch File Integrado

Criar `~/workspace/src/f1tenth_control/launch/f1tenth_full.launch.py`:

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Obter diretórios dos pacotes
    f1tenth_control_dir = get_package_share_directory('f1tenth_control')
    vesc_config_dir = get_package_share_directory('vesc_config')
    joy_converter_dir = get_package_share_directory('joy_converter')
    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')

    # Launch files para os componentes individuais
    vesc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vesc_config_dir, 'launch', 'vesc_driver.launch.py')
        )
    )

    ackermann_to_vesc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('vesc_ackermann'), 'launch', 'ackermann_to_vesc_node.launch.xml')
        )
    )

    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(joy_converter_dir, 'launch', 'launch_joy_ackerman.py')
        )
    )

    # Uncomment quando o lidar estiver integrado
    # lidar_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(ydlidar_dir, 'launch', 'ydlidar_launch.py')
    #     )
    # )

    servo_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_control_dir, 'launch', 'f1tenth_control.launch.py')
        )
    )

    # Launch file completo
    return LaunchDescription([
        vesc_launch,
        ackermann_to_vesc_launch,
        joy_launch,
        # lidar_launch,  # Uncomment quando o lidar estiver integrado
        servo_control_launch,
    ])
```

### 4. Build e Instalação

```bash
# Navegue até a raiz do workspace
cd ~/workspace

# Construir o pacote
colcon build --packages-select f1tenth_control

# Source o setup
source install/setup.bash

# Executar apenas o controle de servo
ros2 launch f1tenth_control f1tenth_control.launch.py

# OU executar o sistema completo
ros2 launch f1tenth_control f1tenth_full.launch.py
```

### 5. Teste e Calibração do Servo Motor

#### 5.1 Script de Calibração para o Servo

Criar `~/workspace/src/f1tenth_control/f1tenth_control/servo_calibration.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pigpio
import time
import sys

class ServoCalibrationNode(Node):
    def __init__(self):
        super().__init__('servo_calibration_node')

        # Parâmetros padrão
        self.declare_parameter('gpio_pin', 18)
        self.declare_parameter('frequency', 50)

        self.gpio_pin = self.get_parameter('gpio_pin').value
        self.frequency = self.get_parameter('frequency').value

        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error('Não foi possível conectar ao daemon pigpio')
            return

        # Configurar o pino GPIO
        self.pi.set_mode(self.gpio_pin, pigpio.OUTPUT)
        self.pi.set_PWM_frequency(self.gpio_pin, self.frequency)

        self.get_logger().info("""
        Iniciando calibração do servo.
        Use as seguintes teclas:
            1: Definir posição mínima (ângulo negativo máximo)
            2: Definir posição central (ângulo zero)
            3: Definir posição máxima (ângulo positivo máximo)
            q: Sair

        Anote os valores para configurar o arquivo control_params.yaml
        """)

        # Posição central inicial
        self.current_pulse_width = 1500  # Valor típico para posição central
        self.set_pulse_width(self.current_pulse_width)

    def set_pulse_width(self, pulse_width):
        """Define a largura de pulso em microssegundos"""
        self.current_pulse_width = pulse_width
        # Converter para duty cycle que o pigpio espera
        duty_cycle = self.current_pulse_width / (1000000.0 / self.frequency) * 255
        self.pi.set_PWM_dutycycle(self.gpio_pin, int(duty_cycle))
        self.get_logger().info(f'Pulse width: {pulse_width} µs')

    def run_interactive(self):
        """Modo interativo para calibração"""
        increment = 10  # Incremento/decremento em microssegundos

        while rclpy.ok():
            cmd = input("Pressione +/- para ajustar (increment: 10µs), wasd para ajustes finos, 1/2/3 para definir posições, q para sair: ")

            if cmd == 'q':
                break
            elif cmd == '+':
                self.set_pulse_width(self.current_pulse_width + increment)
            elif cmd == '-':
                self.set_pulse_width(self.current_pulse_width - increment)
            elif cmd == 'w':  # Ajuste fino para cima
                self.set_pulse_width(self.current_pulse_width + 1)
            elif cmd == 's':  # Ajuste fino para baixo
                self.set_pulse_width(self.current_pulse_width - 1)
            elif cmd == 'a':  # Ajuste fino mais rápido para baixo
                self.set_pulse_width(self.current_pulse_width - 5)
            elif cmd == 'd':  # Ajuste fino mais rápido para cima
                self.set_pulse_width(self.current_pulse_width + 5)
            elif cmd == '1':
                self.get_logger().info(f'Posição MÍNIMA definida: {self.current_pulse_width} µs')
                self.min_pulse_width = self.current_pulse_width
            elif cmd == '2':
                self.get_logger().info(f'Posição CENTRAL definida: {self.current_pulse_width} µs')
                self.center_pulse_width = self.current_pulse_width
            elif cmd == '3':
                self.get_logger().info(f'Posição MÁXIMA definida: {self.current_pulse_width} µs')
                self.max_pulse_width = self.current_pulse_width

        # No final, mostrar os valores para configuração
        try:
            self.get_logger().info("\nValores para configuração:")
            self.get_logger().info(f"servo_min_pulse_width: {self.min_pulse_width}")
            self.get_logger().info(f"servo_center_pulse_width: {self.center_pulse_width}")
            self.get_logger().info(f"servo_max_pulse_width: {self.max_pulse_width}\n")
        except AttributeError:
            self.get_logger().info("Calibração incompleta, algumas posições não foram definidas.")

        # Centralizar o servo antes de sair
        self.set_pulse_width(1500)
        time.sleep(0.5)
        # Liberar o GPIO
        self.pi.set_PWM_dutycycle(self.gpio_pin, 0)
        self.pi.stop()

def main(args=None):
    rclpy.init(args=args)
    node = ServoCalibrationNode()

    try:
        node.run_interactive()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Adicionar à lista de scripts no `setup.py`:

```python
'servo_calibration = f1tenth_control.servo_calibration:main',
```

#### 5.2 Procedimento de Calibração

Executar o script de calibração:

```bash
ros2 run f1tenth_control servo_calibration
```

Seguir as instruções na tela para:

1. Encontrar os valores mínimo, central e máximo de PWM para o servo
2. Anotar esses valores
3. Atualizar o arquivo `control_params.yaml` com os valores corretos

### 6. Notas Importantes para Trabalhos Futuros

#### 6.1 Integração do Lidar

- Quando o Lidar for integrado, descomente a parte correspondente no launch file.
- Certifique-se de configurar corretamente o frame_id no arquivo de parâmetros do YDLidar.
- Adicione uma transformação estática TF entre o frame base do robô e o do Lidar.

#### 6.2 Ajuste de Parâmetros

- Velocidade - Verifique os valores de `speed_to_erpm_gain` e `speed_to_erpm_offset` com o VESC real.
- Direção - Ajuste os valores de `min_steering_angle` e `max_steering_angle` para combinarem com as limitações físicas do veículo.
- Calibre cuidadosamente os limites do servo para prevenir danos mecânicos.

#### 6.3 Considerações de Alimentação

- O servo motor provavelmente requer uma fonte externa de 5V.
- É altamente recomendável NÃO alimentar o servo diretamente pelo Raspberry Pi!
- Use um regulador de tensão ou fonte separada.

## Resumo das Tarefas

1. [x] Analisar a configuração atual dos drivers VESC e Lidar
2. [x] Criar pacote ROS2 Python 'f1tenth_control'
3. [x] Implementar nó de controle do servo via GPIO do Raspberry Pi
4. [x] Criar script de calibração para o servo motor
5. [x] Integrar com o fluxo existente da VESC e republir odometria no formato F1TENTH
6. [ ] Testar componentes individualmente
7. [x] Criar launch file integrado para execução completa
8. [ ] Calibrar parâmetros com o hardware real
