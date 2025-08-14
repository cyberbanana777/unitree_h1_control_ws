from setuptools import setup

package_name = "cmd_to_high_level_control_package"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cyberbanana777",
    maintainer_email="sashagrachev2005@gmail.com",
    description="Programm Converts velocity commands (Twist) to control signals for Unitree \
        H1 robot via ROS 2.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"cmd_to_high_level_control_node = {package_name}.cmd_to_high_level_control_node:main"
        ],
    },
)
