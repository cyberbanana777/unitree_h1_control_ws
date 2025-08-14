from setuptools import setup

package_name = "teleop_twist_keyboard_custom"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cyberbanana777",
    maintainer_email="sashagrachev2005@gmail.com",
    description="Implements teleoperation control for robots via keyboard,\
                publishing geometry_msgs/msg/Twist messages in ROS2 topic cmd_vel.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"teleop_node = {package_name}.teleop_twist_keyboard_custom_node:main"
        ],
    },
)
