# Sorting Station GUI - Interactive Simulation Environment

## Overview
This is a complete sorting station simulation environment built with MuJoCo that includes:
- **Conveyor belt** with moving surface
- **6 boxes**: 3 white boxes and 3 black boxes
- **2 sorting bins**: White bin (left) and black bin (right)
- **UR5e robotic arm** for automated sorting
- **Work environment**: Table, safety fence, tool rack, emergency stop button

## Components

### Scene Elements
- **Conveyor Belt System**: Industrial-style conveyor (1.2m x 0.4m) at working height
- **White Boxes** (3x): Located at positions along the conveyor
- **Black Boxes** (3x): Interspersed with white boxes on conveyor
- **White Sorting Bin**: Left side (-0.9m, 0, 0.42m) with transparent walls
- **Black Sorting Bin**: Right side (0.9m, 0, 0.42m) with transparent walls
- **UR5e Robot**: Mounted at (-0.6m, 0, 0.42m) for optimal reach
- **Safety Equipment**: Back fence and emergency stop button

### Technical Specifications
- **Total Bodies**: 21
- **Geoms (shapes)**: 63
- **Degrees of Freedom**: 42 (6 for robot + 36 for free-moving boxes)
- **Sensors**: 6 (position sensors for all boxes)
- **Actuators**: 6 (robot joints)

## How to Run

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

## Scene Configuration

### Box Positions (Initial)
**White Boxes:**
1. Position: (-0.3, 0, 0.515m)
2. Position: (-0.05, 0.05, 0.515m)
3. Position: (0.25, -0.03, 0.515m)

**Black Boxes:**
1. Position: (-0.45, -0.04, 0.515m)
2. Position: (0.1, 0.02, 0.515m)
3. Position: (0.4, 0.06, 0.515m)

### Robot Home Position
- Shoulder pan: -90°
- Shoulder lift: -90°
- Elbow: 90°
- Wrist 1: -90°
- Wrist 2: -90°
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
- [ ] Animated conveyor belt motion
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
