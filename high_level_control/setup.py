from setuptools import setup

package_name = 'high_level_control'

setup(
    name=package_name,
    version='1.0.0',
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
    description='This script launches a high-level client that allows you to control the Unitree H1 robot using\
                igh-level commands (such as damping, readiness, and balancing, etc.) as if it were a remote control.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'high_level_control = high_level_control.high_level_control_node:main',
        ],
    },
)
