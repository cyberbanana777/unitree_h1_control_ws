from setuptools import setup

package_name = "low_level_control"
path_to_bin = "resource/communication_pack/build/inspire_hand"

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
        ("share/" + package_name, [path_to_bin]),
    ],
    install_requires=["setuptools", "black", "isort", "flake8"],
    zip_safe=True,
    maintainer="cyberbanana777",
    maintainer_email="sashagrachev2005@gmail.com",
    description="Implements low-level control for Unitree H1 robot via ROS 2,\
                managing arm joints, fingers and wrists.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "low_level_control_with_hands_node = low_level_control.low_level_node_with_hands:main",
            "low_level_control_without_hands_node = low_level_control.low_level_node_without_hands:main",
            "wrist_control_node = low_level_control.wrist_control_node:main",
            "hands_init_node = low_level_control.hands_init:main",
            'hands_node_with_lib = low_level_control.hands_node_with_lib:main',
        ],
    },
)
