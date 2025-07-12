from setuptools import setup
import os
from glob import glob

package_name = "joy_converter"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Adiciona a instalação de todos os arquivos .launch.py da pasta launch
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="disney",
    maintainer_email="gustavo.rio@aluno.ufabc.edu.br",
    description="Conversor de comandos joystick para F1TENTH",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joy_ackerman = joy_converter.joy_ackerman:main",
            "joy_keyboard = joy_converter.joy_keyboard_converter:main",
        ],
    },
)
