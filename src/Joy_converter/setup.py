from setuptools import setup
from glob import glob
import os

package_name = "joy_converter"

setup(
    name="joy-converter",
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/launch_joy_ackerman.py"]),
        ("share/" + package_name + "/launch", ["launch/launch_joy_ackerman_fixed.py"]),
        ("share/" + package_name + "/launch", ["launch/launch_joy_twist.py"]),
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
            "joy_twist = joy_converter.joy_twist:main",
            "joy_keyboard = joy_converter.joy_keyboard_converter:main",
        ],
    },
)
