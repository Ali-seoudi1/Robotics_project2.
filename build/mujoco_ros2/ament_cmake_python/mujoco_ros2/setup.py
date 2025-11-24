from setuptools import find_packages
from setuptools import setup

setup(
    name='mujoco_ros2',
    version='0.0.0',
    packages=find_packages(
        include=('mujoco_ros2', 'mujoco_ros2.*')),
)
