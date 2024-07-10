from setuptools import find_packages
from setuptools import setup

setup(
    name='custom_robot_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('custom_robot_msgs', 'custom_robot_msgs.*')),
)
