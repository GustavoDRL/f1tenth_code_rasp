from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'f1tenth_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Incluir arquivos de launch
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        # Incluir arquivos de configuração
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='F1TENTH Team',
    maintainer_email='dev@f1tenth.org',
    description=('Controle integrado F1TENTH para Raspberry Pi '
                 'com VESC e servo GPIO'),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_control_node = f1tenth_control.servo_control_node:main',
            ('enhanced_servo_control_node = '
             'f1tenth_control.enhanced_servo_control_node:main'),
            'servo_calibration = f1tenth_control.servo_calibration:main',
        ],
    },
)
