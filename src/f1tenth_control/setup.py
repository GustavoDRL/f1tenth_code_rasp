from setuptools import setup, find_packages
from setuptools.command.develop import develop
from setuptools.command.install import install
from glob import glob
import os
import atexit


def create_ros2_executable_links():
    """Criar links simbólicos para compatibilidade ROS2 Humble"""
    try:
        # Tentar diferentes caminhos possíveis de instalação
        possible_install_dirs = [
            os.path.join(os.getcwd(), 'install', 'f1tenth_control'),
            os.path.join(os.path.dirname(__file__), '..', '..', 'install', 'f1tenth_control'),
            '/opt/ros/humble/share/f1tenth_control',
        ]
        
        install_dir = None
        for path in possible_install_dirs:
            if os.path.exists(os.path.join(path, 'bin')):
                install_dir = path
                break
        
        if not install_dir:
            print("Warning: Could not find f1tenth_control install directory")
            return
            
        lib_dir = os.path.join(install_dir, 'lib', 'f1tenth_control')
        bin_dir = os.path.join(install_dir, 'bin')
        
        # Criar diretório lib se não existir
        os.makedirs(lib_dir, exist_ok=True)
        
        # Lista de executáveis para linkar
        executables = [
            'servo_control_node',
            'enhanced_servo_control_node', 
            'servo_calibration'
        ]
        
        # Criar links simbólicos
        for exe in executables:
            bin_path = os.path.join(bin_dir, exe)
            lib_path = os.path.join(lib_dir, exe)
            
            # Remover link existente se houver
            if os.path.exists(lib_path) or os.path.islink(lib_path):
                os.remove(lib_path)
            
            # Criar link simbólico relativo
            if os.path.exists(bin_path):
                rel_path = os.path.relpath(bin_path, lib_dir)
                os.symlink(rel_path, lib_path)
                print(f"✅ Created ROS2 symlink: {exe}")
                
    except Exception as e:
        print(f"Warning: Could not create ROS2 executable links: {e}")


class PostDevelopCommand(develop):
    """Comando personalizado para executar após develop (symlink install)"""
    def run(self):
        develop.run(self)
        # Agendar criação de links para depois que tudo estiver instalado
        atexit.register(create_ros2_executable_links)


class PostInstallCommand(install):
    """Comando personalizado para executar após install"""
    def run(self):
        install.run(self)
        # Agendar criação de links para depois que tudo estiver instalado
        atexit.register(create_ros2_executable_links)


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
    cmdclass={
        'develop': PostDevelopCommand,
        'install': PostInstallCommand,
    },
)
