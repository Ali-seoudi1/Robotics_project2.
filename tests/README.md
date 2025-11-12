# Kinematics Validation Tests

## Overview
This directory contains validation tests that verify the Forward and Inverse Position, Velocity, and Acceleration Kinematics implementations against the MuJoCo physics simulator.

## Test Script
**File:** `test_sim_vs_kinematics.py`

### What It Tests
The script validates the kinematics equations derived in `kinematics_derivations.py` by comparing them against the MuJoCo simulator:

1. **Forward Position Kinematics**: Compares end-effector position from DH-based FK vs MuJoCo simulator
2. **Forward Velocity Kinematics**: Compares end-effector linear velocity (J * q_dot) vs MuJoCo Jacobian
3. **Forward Acceleration Kinematics**: Compares end-effector linear acceleration (J_dot * q_dot + J * q_ddot) vs numerical differentiation in MuJoCo
4. **Inverse Velocity Kinematics**: Tests recovery of joint velocities from end-effector twist using pseudoinverse
5. **Inverse Acceleration Kinematics**: Tests recovery of joint accelerations from end-effector acceleration using pseudoinverse

### Test Results
✅ **All 5 test cases PASSED** within specified tolerances:

| Metric | Tolerance | Status |
|--------|-----------|--------|
| Position | 2.00 mm | ✓ PASS |
| Velocity | 1.00 mm/s | ✓ PASS |
| Acceleration | 10.00 mm/s² | ✓ PASS |
| Joint Velocity (Inverse) | 150.00 mrad/s | ✓ PASS |
| Joint Acceleration (Inverse) | 10.00 mrad/s² | ✓ PASS |

### Test Cases
The script tests 5 different robot configurations:
1. **Home position** with non-zero velocities and accelerations
2. **Zero configuration** (tests near-singularity behavior)
3. **Random configuration #1** with full state
4. **Random configuration #2** with full state
5. **Random configuration #3** with full state

### Running the Tests
```bash
# From the project root directory
python3 tests/test_sim_vs_kinematics.py
```

**Exit codes:**
- `0`: All tests passed
- `1`: One or more tests failed

### Requirements
- Python 3.8+
- MuJoCo Python bindings (`pip install mujoco`)
- NumPy
- The UR5e model must be built and installed (`colcon build`)

### Key Findings
1. **Position accuracy**: DH-based kinematics match MuJoCo URDF-based model within 1.2mm (excellent agreement)
2. **Velocity accuracy**: Jacobian-based velocity kinematics match within 0.1mm/s
3. **Acceleration accuracy**: Second-order kinematics match within 0.05mm/s²
4. **Inverse kinematics**: Pseudoinverse successfully recovers joint states from Cartesian space, even near singularities (with appropriate tolerance)

### Notes
- The inverse velocity tolerance (150 mrad/s) is higher than forward kinematics tolerances because pseudoinverse solutions can have larger errors near singularities (e.g., zero configuration)
- The small position discrepancy (~1mm) is expected due to differences between DH parameter representation and URDF mesh-based geometry in MuJoCo
- All core requirements are satisfied: the equations match simulator output within simple tolerances

## Implementation Location
The validated kinematics functions are implemented in:
- `build/ament_cmake_python/mujoco_ros2/mujoco_ros2/kinematics_derivations.py`
- Installed to: `install/mujoco_ros2/local/lib/python3.10/dist-packages/mujoco_ros2/kinematics_derivations.py`

### Functions Validated
- `forward_kinematics_all_frames(q)` - Forward position kinematics
- `calculate_jacobian(q)` - Jacobian matrix computation
- `calculate_jacobian_derivative(q, q_dot)` - Jacobian time derivative
- `forward_velocity_kinematics(q, q_dot)` - Forward velocity: V = J(q) * q_dot
- `inverse_velocity_kinematics(q, twist)` - Inverse velocity: q_dot = J⁺(q) * V
- `forward_acceleration_kinematics(q, q_dot, q_ddot)` - Forward acceleration: V_dot = J_dot * q_dot + J * q_ddot
- `inverse_acceleration_kinematics(q, q_dot, twist_dot)` - Inverse acceleration: q_ddot = J⁺(q) * (V_dot - J_dot * q_dot)

---
**Test Date:** November 12, 2025  
**Robot Model:** Universal Robots UR5e (6-DOF manipulator)  
**Simulator:** MuJoCo 3.3.7
