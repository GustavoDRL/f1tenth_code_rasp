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
        # Incluir arquivos de launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Incluir arquivos de configuração
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='F1TENTH Team',
    maintainer_email='dev@f1tenth.org',
    description='Controle integrado F1TENTH para Raspberry Pi com VESC e servo GPIO',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Definir o executável para o nó de controle
            'servo_control_node = f1tenth_control.servo_control_node:main',
            # Adicionar o executável para o script de calibração
            'enhanced_servo_control_node = f1tenth_control.enhanced_servo_control_node:main',
            'servo_calibration = f1tenth_control.servo_calibration:main',
        ],
    },
)