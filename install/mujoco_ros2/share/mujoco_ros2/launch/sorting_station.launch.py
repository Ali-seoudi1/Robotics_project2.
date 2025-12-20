#!/usr/bin/env python3
"""ROS 2 launch file that runs the automated sorting station script."""
import sys

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description() -> LaunchDescription:
    simulation = ExecuteProcess(
        cmd=[sys.executable, '-m', 'mujoco_ros2.sorting_station_gui'],
        output='screen',
    )

    return LaunchDescription([simulation])
