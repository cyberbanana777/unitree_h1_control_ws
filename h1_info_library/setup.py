from setuptools import setup, find_packages

package_name = 'h1_info_library'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cyberbanana',
    maintainer_email='sasha_grachev2005@mail.ru',
    description='This is a python3 library for comfortable programming Unitree H1. \
        It lib include reference info and optimized data structure for programming robot.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
