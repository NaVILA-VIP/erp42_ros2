from setuptools import setup

setup(
    name="erp42_control",
    version="0.1.0",
    packages=["erp42_control"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/erp42_control"]),
        ("share/erp42_control", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    maintainer="nabih",
    maintainer_email="nabih@todo.todo",
    description="Torque, steering and noisy odometry nodes for ERP42",
    license="TODO",
    entry_points={"console_scripts": [
        "main.py = erp42_control.commands:torque",
        "steer.py = erp42_control.commands:steer",
        "motionERP42.py = erp42_control.commands:motion",
        "noisy_odom.py = erp42_control.noisy_odom:main",
    ]},
)
