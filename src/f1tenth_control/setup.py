from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'f1tenth_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Incluir arquivos de launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Incluir arquivos de configuração
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seu_nome', # TODO: Atualizar com seu nome
    maintainer_email='seu_email@example.com', # TODO: Atualizar com seu email
    description='Controle integrado F1TENTH para Raspberry Pi (VESC + Servo)',
    license='TODO: License declaration', # TODO: Escolher e declarar uma licença (e.g., Apache 2.0, BSD)
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Definir o executável para o nó de controle
            'servo_control_node = f1tenth_control.servo_control_node:main',
            # Adicionar o executável para o script de calibração
            'servo_calibration = f1tenth_control.servo_calibration:main',
            # Adicionar aliases explícitos (tentativa de corrigir)
            'f1tenth_servo_control = f1tenth_control.servo_control_node:main',
            'f1tenth_servo_calibration = f1tenth_control.servo_calibration:main',
        ],
    },
) 