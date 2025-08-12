"""
Setup script for the inspire_hand package.
"""

from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="inspire_hand",
    version="0.1.0",
    author="Inspire Hand Team",
    author_email="your.email@example.com",
    description="Python library for controlling the Inspire Hand RH56dfq robotic hand",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/yourusername/inspire_hand",
    packages=find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
    install_requires=[
        "pyserial>=3.5",
    ],
) 