from setuptools import setup

package_name = 'wrist_control_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cyberbanana777',
    maintainer_email='sashagrachev2005@gmail.com',
    description='The ROS2 node wrist_control_node controls the DM4310 motors\
        via the CAN interface, receiving commands (position/speed/torque) via\
        the wrist/cmds topic and publishing states (wrist/states, 1000 Hz)\
        with a limited operating range in MIT mode. The node uses a UART-CAN\
        adapter (/dev/ttyACM0), logs errors, and correctly terminates by\
        disabling the motors.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wrist_control_node = wrist_control_package.wrist_control_node:main'
        ],
    },
)
