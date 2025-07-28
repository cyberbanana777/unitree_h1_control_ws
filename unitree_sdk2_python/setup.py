from setuptools import find_packages, setup

setup(
    name="unitree_sdk2py",
    version="1.0.1",
    author="UnitreeRobotics",
    author_email="unitree@unitree.com",
    long_description="",
    long_description_content_type="",
    license="BSD-3-Clause",
    packages=find_packages(include=["unitree_sdk2py", "unitree_sdk2py.*"]),
    description="Unitree robot sdk version 2 for python",
    project_urls={
        "Source Code": "https://github.com/unitreerobotics/unitree_sdk2_python",
    },
    python_requires=">=3.8",
    install_requires=[
        "cyclonedds==0.10.2",
        "numpy",
        "opencv-python",
    ],
)
