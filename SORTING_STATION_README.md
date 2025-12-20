# Sorting Station GUI - Interactive Simulation Environment

## Overview
This is a complete sorting station simulation environment built with MuJoCo that includes:
- **Staging area** with four boxes directly in front of the UR5e
- **4 boxes**: 2 white boxes and 2 black boxes
- **2 sorting bins**: White bin and black bin mounted side-by-side near the conveyor
- **UR5e robotic arm** for automated sorting
- **Work environment**: Table, safety fence, tool rack, safety guard rails
- **Automated pick & place routine** that satisfies the Milestone 5 requirement by sorting the white/black boxes into their dedicated bins using IK + trajectory planning.

## Components

### Scene Elements
- **Staging Area**: Four boxes spawn on the tabletop directly in front of the arm
- **White Boxes** (2x): Positioned on the positive Y side
- **Black Boxes** (2x): Positioned on the negative Y side
- **White & Black Sorting Bins**: Positioned together at x≈0.25 m, y=±0.38 m so the arm can drop off parts quickly
- **UR5e Robot**: Mounted at (-0.6m, 0, 0.42m) for optimal reach
- **Safety Equipment**: Back fence and table guard rails

### Technical Specifications
- **Degrees of Freedom**: 30 (6 robot joints + 24 DOFs for the four free boxes)
- **Sensors**: None (boxes are scripted via predefined coordinates)
- **Actuators**: 6 (robot joints)

## How to Run

The automated routine is implemented in `src/mujoco_ros2/mujoco_ros2/sorting_station_gui.py` and is exposed through the quick launcher in the repository root.

### Option 1: Direct Python Launch
```bash
# From project root
python3 launch_sorting_station.py
```

### Option 2: Run Module Directly
```bash
# From project root
cd src/mujoco_ros2
python3 -m mujoco_ros2.sorting_station_gui
```

### Option 3: After Building with colcon
```bash
# Build the package first
colcon build --packages-select mujoco_ros2

# Source the workspace
source install/setup.bash

# Launch via ROS2
ros2 launch mujoco_ros2 sorting_station.launch.py
```

## Controls

### Mouse Controls
- **Left Click + Drag**: Rotate camera view
- **Right Click + Drag**: Pan camera
- **Scroll Wheel**: Zoom in/out
- **Double Click**: Select and track objects

### Keyboard Shortcuts
- **Space**: Pause/Resume simulation
- **Ctrl+R**: Reset to initial configuration
- **Tab**: Toggle rendering options
- **F1**: Show/hide help overlay
- **ESC**: Exit application

## Automated Sorting Sequence
- **State machine** waits 3 seconds in the upright posture, then walks through four pick-and-place tasks alternating between white and black boxes so both bins are utilized.
- **IK solver** (`inverse_kinematics`) finds the joint configuration for every hover/pick/place pose with a downward tool orientation.
- **Trajectory planner** (`generate_joint_trajectory`) produces smooth cubic joint trajectories for every move, satisfying the milestone trajectory requirement.
- **Joint-level PD control** commands the MuJoCo actuators in real time to track each trajectory sample at 150 Hz while respecting the transparent guard that keeps the arm above the table.
- **Scripted pickup coordinates**: the arm follows predefined poses for each box (no runtime sensing needed) before placing items into the correct bins.
- **Virtual grasp** attaches a box to the gripper once it reaches the pick height, then releases it when the arm descends inside the correct bin.

## Scene Configuration

### Box Positions (Initial)
**White Boxes:**
1. Position: (-0.20, 0.18, 0.430m)
2. Position: (0.00, 0.18, 0.430m)

**Black Boxes:**
1. Position: (-0.20, -0.18, 0.430m)
2. Position: (0.00, -0.18, 0.430m)

### Sorting Bins (Initial)
- White Bin: (0.25, 0.38, 0.42m)
- Black Bin: (0.25, -0.38, 0.42m)

### Robot Home Position
- Shoulder pan: 0°
- Shoulder lift: -90°
- Elbow: 0°
- Wrist 1: -90°
- Wrist 2: 0°
- Wrist 3: 0°

## Development

### Adding New Objects
Edit `src/mujoco_ros2/model/sorting_scene.xml` and add bodies in the `<worldbody>` section.

### Modifying Materials
Materials are defined in the `<asset>` section. You can adjust colors, textures, and physical properties.

### Physics Parameters
- Friction coefficients can be adjusted per geom
- Box mass: 0.08 kg (realistic for small cardboard boxes)
- Bin walls: Semi-transparent for visibility

## Future Enhancements
- [ ] Animated conveyor belt motion synced with pick events
- [ ] Additional tools on rack (gripper, suction cup)
- [ ] Multiple robot arms for faster sorting
- [ ] Vision system simulation with cameras
- [ ] Quality inspection station
- [ ] Box dispensing mechanism
- [ ] Sorting statistics dashboard
- [ ] Automated sorting control logic

## Files
- `sorting_scene.xml` - Main MuJoCo scene definition
- `sorting_station_gui.py` - Python visualization script
- `sorting_station.launch.py` - ROS2 launch file
- `launch_sorting_station.py` - Quick launch script

## Troubleshooting

**Issue**: Scene won't load
- Ensure MuJoCo Python package is installed: `pip install mujoco`
- Check that mesh files exist in `model/assets/`

**Issue**: Poor performance
- Reduce visual quality in MuJoCo viewer settings
- Lower physics timestep in XML (currently 0.002s)

**Issue**: Robot appears at wrong location
- Verify robot base position in XML: `<body name="base" pos="-0.6 0 0.42">`

## Requirements
- Python 3.8+
- MuJoCo 3.0+ with Python bindings
- NumPy
- (Optional) ROS2 Humble for launch file support

---
**Last Updated**: November 12, 2025
