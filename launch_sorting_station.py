#!/usr/bin/env python3
"""
Quick launcher for the Sorting Station GUI
Run this directly: python3 launch_sorting_station.py
"""

import os
import sys

# Add the src directory to Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.join(script_dir, 'src', 'mujoco_ros2')
sys.path.insert(0, src_dir)

# Import and run the GUI
from mujoco_ros2.sorting_station_gui import main

if __name__ == "__main__":
    print("="*60)
    print("SORTING STATION - Quick Launch")
    print("="*60)
    main()
