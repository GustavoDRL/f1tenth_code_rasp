#!/usr/bin/env python3
from setuptools import setup, find_packages
from glob import glob
import os

package_name = "f1tenth_control"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test", "tests*"]),
    # Metadados do Pacote
    author="F1TENTH Team",
    author_email="dev@f1tenth.org",
    description="Controle F1TENTH para Raspberry Pi com VESC e servo",
    long_description=(
        "Sistema de controle F1TENTH otimizado para performance em tempo real "
        "em Raspberry Pi 4B com integração VESC e controle de servo."
    ),
    license="MIT",
    url="https://github.com/GustavoDRL/f1tenth_code_rasp",
    # Estrutura de arquivos de dados
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    # Dependências Python essenciais para F1TENTH (não-ROS)
    # Estas serão instaladas via pip se não estiverem presentes
    install_requires=[
        "setuptools>=58.2.0",
        "numpy",
        "pyyaml",
        "psutil>=5.8.0",
        "pyserial>=3.4",
    ],
    # Dependências de Desenvolvimento
    extras_require={
        "dev": [
            "pytest>=6.0",
            "flake8>=3.8",
            "black>=21.0",
        ]
    },
    # Pontos de entrada para os executáveis
    entry_points={
        "console_scripts": [
            "servo_control_node = f1tenth_control.servo_control_node:main",
            "enhanced_servo_control_node = f1tenth_control.enhanced_servo_control_node:main",
            "servo_calibration = f1tenth_control.servo_calibration:main",
            "test_calibrated_servo = f1tenth_control.test_calibrated_servo:main",
        ],
    },
    # Configurações de build para performance
    python_requires=">=3.8",
    zip_safe=False,
    # Classificadores para PyPI
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Topic :: Scientific/Engineering :: Robotics",
    ],
)
