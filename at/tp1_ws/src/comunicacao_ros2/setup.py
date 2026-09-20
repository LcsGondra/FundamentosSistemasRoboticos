import os
from glob import glob
from setuptools import find_packages, setup

package_name = "comunicacao_ros2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lucas Dias de Gondra",
    maintainer_email="lucas.gondra@al.infnet.edu.br",
    description="Pacote de comunicacao ROS 2 para AT",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "monitor_transporte = comunicacao_ros2.monitor_transporte:main",
            "confirmar_entrega_client = comunicacao_ros2.confirmar_entrega_client:main",
            "transporte_action_server = comunicacao_ros2.transporte_action_server:main",
            "transporte_action_client = comunicacao_ros2.transporte_action_client:main",
            "joint_state_publisher = comunicacao_ros2.joint_state_publisher:main",
        ],
    },
)
