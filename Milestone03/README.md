# Milestone 03 - Velocity and Acceleration Kinematics Analysis

## Overview
This milestone contains the complete implementation and validation of forward and inverse velocity and acceleration kinematics for the UR5e robotic arm, along with a GUI visualization environment for the sorting station.

## Directory Structure

```
Milestone03/
├── codes/
│   ├── kinematics_derivations.py          # Core kinematics implementation
│   ├── test_sim_vs_kinematics.py         # Validation tests
│   ├── sorting_station_gui.py            # Interactive GUI
│   └── sorting_scene.xml                 # MuJoCo scene definition
├── videos/
│   ├── VIDEO_INSTRUCTIONS.md             # Instructions for recording videos
│   └── (Place your recorded videos here)
├── report/
│   └── Milestone03_Report.pdf            # Comprehensive analysis report
└── README.md                              # This file
```

## Contents

### 1. Codes (`/codes`)

#### `kinematics_derivations.py`
Complete implementation of:
- **Forward Position Kinematics**: DH-based transformation matrices
- **Jacobian Calculation**: 6×6 analytical Jacobian matrix
- **Jacobian Derivative**: Numerical computation of J̇
- **Forward Velocity Kinematics**: V = J(q) · q̇
- **Inverse Velocity Kinematics**: q̇ = J⁺(q) · V
- **Forward Acceleration Kinematics**: V̇ = J̇(q,q̇) · q̇ + J(q) · q̈
- **Inverse Acceleration Kinematics**: q̈ = J⁺(q) · (V̇ - J̇(q,q̇) · q̇)

**Key Functions:**
- `forward_kinematics_all_frames(q)` - Returns transformation matrices
- `calculate_jacobian(q)` - Computes the Jacobian
- `forward_velocity_kinematics(q, q_dot)` - FK velocity
- `inverse_velocity_kinematics(q, twist)` - IK velocity
- `forward_acceleration_kinematics(q, q_dot, q_ddot)` - FK acceleration
- `inverse_acceleration_kinematics(q, q_dot, twist_dot)` - IK acceleration

#### `test_sim_vs_kinematics.py`
Validation test suite that compares kinematic equations against MuJoCo simulator:
- 5 test configurations (home, zero, and 3 random poses)
- Tests position, velocity, and acceleration accuracy
- Validates inverse kinematics recovery
- Reports errors with specified tolerances

**Test Results:**
- ✅ Position accuracy: ≤ 2mm (actual: 0.7-1.2mm)
- ✅ Velocity accuracy: ≤ 1mm/s (actual: 0.02-0.12mm/s)
- ✅ Acceleration accuracy: ≤ 10mm/s² (actual: 0.003-0.05mm/s²)
- ✅ Inverse velocity: ≤ 150 mrad/s
- ✅ Inverse acceleration: ≤ 10 mrad/s²

#### `sorting_station_gui.py`
Interactive visualization environment featuring:
- Real-time physics simulation
- Mouse-controlled 3D camera
- Sensor data display
- Robot motion visualization

#### `sorting_scene.xml`
Complete MuJoCo scene with:
- Conveyor belt system
- 6 boxes (3 white, 3 black)
- 2 sorting bins
- UR5e robot arm
- Work environment (table, safety fence, tools)

### 2. Videos (`/videos`)

**Required Videos** (to be recorded by you):

1. **Kinematics Validation Video** (`kinematics_validation.mp4`)
   - Run: `python3 codes/test_sim_vs_kinematics.py`
   - Record terminal output showing all tests passing
   - Comment on accuracy and tolerances

2. **GUI Demonstration** (`sorting_station_gui.mp4`)
   - Run: `python3 launch_sorting_station.py`
   - Show the complete sorting station environment
   - Demonstrate camera controls and interaction
   - Show all components: conveyor, boxes, bins, robot

3. **Kinematics Testing** (`kinematics_testing.mp4`)
   - Show the test script running with different joint angles
   - Display position/velocity comparisons
   - Highlight tolerance satisfaction

**Recording Instructions:** See `VIDEO_INSTRUCTIONS.md` in the videos folder

### 3. Report (`/report`)

The comprehensive report includes:

**Section 1: Introduction**
- Project overview
- Objectives
- UR5e specifications

**Section 2: Theoretical Background**
- Denavit-Hartenberg parameters
- Jacobian matrix derivation
- Velocity kinematics equations
- Acceleration kinematics equations

**Section 3: Implementation**
- Python implementation details
- DH transformation matrices
- Jacobian computation algorithm
- Numerical differentiation for J̇

**Section 4: Validation Methodology**
- Test setup with MuJoCo simulator
- Test configurations
- Tolerance criteria
- Error metrics

**Section 5: Results and Analysis**
- Test results summary
- Position accuracy: 0.7-1.2mm error
- Velocity accuracy: 0.02-0.12mm/s error
- Acceleration accuracy: 0.003-0.05mm/s² error
- Discussion of singularity handling

**Section 6: GUI Development**
- Sorting station design
- Component specifications
- Visualization features

**Section 7: Conclusions**
- Achievement summary
- Future improvements

## How to Run

### Prerequisites
```bash
pip install mujoco numpy
```

### Run Kinematics Validation
```bash
cd Milestone03/codes
python3 test_sim_vs_kinematics.py
```

Expected output: All 5 tests PASS with detailed error metrics

### Run GUI Visualization
```bash
# From project root
python3 launch_sorting_station.py
```

### Run Kinematics Demo
```bash
cd Milestone03/codes
python3 kinematics_derivations.py
```

## Key Achievements

✅ **Forward Kinematics**
- Position: Validated within 1.2mm
- Velocity: Validated within 0.12mm/s
- Acceleration: Validated within 0.05mm/s²

✅ **Inverse Kinematics**
- Velocity recovery: Successful with pseudoinverse
- Acceleration recovery: Successful with tolerance handling
- Singularity management: Appropriate tolerances near singularities

✅ **GUI Environment**
- Complete sorting station with 21 bodies, 63 geoms
- Interactive 3D visualization
- Real-time physics simulation
- 6 DOF robot control ready

## Deliverables Checklist

- [x] Kinematics implementation code
- [x] Validation test suite
- [x] GUI visualization system
- [x] Scene definition file
- [ ] Video: Kinematics validation with commentary
- [ ] Video: GUI demonstration
- [ ] Video: Kinematics testing
- [x] Comprehensive report (PDF)
- [x] README documentation

## Notes for Video Recording

When recording videos, please include:
1. **Terminal output** showing test results
2. **Commentary** explaining what's being demonstrated
3. **Zoom/highlight** important values and tolerances
4. **Show the GUI** from multiple camera angles
5. **Demonstrate** robot motion and box interactions

## References

- Universal Robots UR5e Technical Specifications
- MuJoCo Documentation: https://mujoco.readthedocs.io/
- Robotics: Modelling, Planning and Control (Siciliano et al.)
- Modern Robotics (Lynch & Park)

---
**Milestone Completed:** November 12, 2025  
**Robot Model:** Universal Robots UR5e  
**Simulator:** MuJoCo 3.3.7  
**Framework:** ROS2 Humble
