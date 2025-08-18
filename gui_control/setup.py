import os
from glob import glob

from setuptools import setup

package_name = "gui_control"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "resource"),
            glob("resource/*.png"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cyberbanana777",
    maintainer_email="sashagrachev2005@gmail.com",
    description="GUI for controlling Unitree H1 robot joints via ROS2.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gui_control_node = gui_control.gui_control_node:main",
            "gui_check_motors_node = gui_control.gui_check_motors_node:main"
        ],
    },
)
