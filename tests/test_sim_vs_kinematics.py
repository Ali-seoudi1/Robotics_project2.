#!/usr/bin/env python3
import os
import sys
import numpy as np
import mujoco
from importlib.machinery import SourceFileLoader

# Load kinematics module from build folder
kin_path = os.path.join(os.getcwd(), 'build', 'ament_cmake_python', 'mujoco_ros2', 'mujoco_ros2', 'kinematics_derivations.py')
kin = SourceFileLoader('kin', kin_path).load_module()

# Model path installed by the package
model_path = os.path.join(os.getcwd(), 'install', 'mujoco_ros2', 'share', 'mujoco_ros2', 'model', 'ur5e.xml')
if not os.path.exists(model_path):
    raise FileNotFoundError(f"Model XML not found: {model_path}")

model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Find site id for end-effector (attachment_site)
site_name = 'attachment_site'
site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name.encode('utf-8'))
if site_id == -1:
    raise RuntimeError(f"Site '{site_name}' not found in model")

# Tolerances (realistic for robotics applications)
pos_tol = 2e-3   # 2mm position tolerance (reasonable for DH vs URDF differences)
vel_tol = 1e-3   # 1mm/s velocity tolerance
qdot_tol = 150e-3  # 150 mrad/s joint velocity tolerance (inverse kinematics with pseudoinverse, can be large near singularities)
acc_tol = 1e-2   # 10mm/s^2 acceleration tolerance
qddot_tol = 1e-2 # 10 mrad/s^2 joint acceleration tolerance

# Test cases (q, q_dot, q_ddot)
tests = []
# home pose from XML keyframe
q_home = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
qdot_zero = np.zeros(6)
qddot_zero = np.zeros(6)

# basic tests
tests.append((q_home, np.array([0.1, -0.05, 0.0, 0.0, 0.02, 0.0]), np.array([0.01, 0.0, 0.0, 0.005, 0.0, 0.0])))
tests.append((np.zeros(6), np.array([0.0, 0.0, 0.1, -0.1, 0.0, 0.02]), np.array([0.0, 0.01, 0.0, -0.01, 0.0, 0.0])))
np.random.seed(123)
for _ in range(3):
    q_rand = (np.random.rand(6) - 0.5) * 1.0  # random within +/-0.5 rad
    qdot_rand = (np.random.rand(6) - 0.5) * 0.2
    qddot_rand = (np.random.rand(6) - 0.5) * 0.1
    tests.append((q_rand, qdot_rand, qddot_rand))

print(f"Running {len(tests)} tests comparing simulator vs kinematic functions...")
print(f"Tolerances: pos={pos_tol*1000:.2f}mm, vel={vel_tol*1000:.2f}mm/s, acc={acc_tol*1000:.2f}mm/s², qdot={qdot_tol*1000:.2f}mrad/s, qddot={qddot_tol*1000:.2f}mrad/s²\n")

all_pass = True
for i, (q, q_dot, q_ddot) in enumerate(tests, start=1):
    # set states
    data.qpos[:] = q
    data.qvel[:] = q_dot
    # Note: qacc is computed by MuJoCo, not set directly
    # For testing forward acceleration kinematics, we use q_ddot as the test input

    # forward kinematics in simulator
    mujoco.mj_forward(model, data)

    # simulator readings for end-effector
    sim_pos = data.site_xpos[site_id].copy()
    
    # Compute site velocity using Jacobian (standard MuJoCo approach)
    # Allocate jacobians (3xnv for translation, 3xnv for rotation)
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
    
    # Compute velocities: v = J * qvel
    sim_linvel = jacp @ data.qvel
    sim_angvel = jacr @ data.qvel
    
    # Compute accelerations using numerical differentiation of Jacobian
    # acc = J_dot * qvel + J * qacc
    delta_t = 1e-7
    data_temp = mujoco.MjData(model)
    data_temp.qpos[:] = q + q_dot * delta_t
    data_temp.qvel[:] = q_dot
    mujoco.mj_forward(model, data_temp)
    
    jacp_next = np.zeros((3, model.nv))
    jacr_next = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data_temp, jacp_next, jacr_next, site_id)
    
    jacp_dot = (jacp_next - jacp) / delta_t
    jacr_dot = (jacr_next - jacr) / delta_t
    
    # Simulator acceleration using our test q_ddot input
    sim_linacc = jacp_dot @ data.qvel + jacp @ q_ddot
    sim_angacc = jacr_dot @ data.qvel + jacr @ q_ddot

    # Kinematic predictions
    T_mats = kin.forward_kinematics_all_frames(q)
    kin_pos = T_mats[-1][:3, 3]

    twist = kin.forward_velocity_kinematics(q, q_dot)
    kin_linvel = twist[:3]
    kin_angvel = twist[3:]
    
    twist_dot = kin.forward_acceleration_kinematics(q, q_dot, q_ddot)
    kin_linacc = twist_dot[:3]
    kin_angacc = twist_dot[3:]

    # Inverse velocity: from simulator twist back to q_dot
    sim_twist = np.hstack((sim_linvel, sim_angvel))
    qdot_calc = kin.inverse_velocity_kinematics(q, sim_twist)
    
    # Inverse acceleration: from simulator twist_dot back to q_ddot
    sim_twist_dot = np.hstack((sim_linacc, sim_angacc))
    qddot_calc = kin.inverse_acceleration_kinematics(q, q_dot, sim_twist_dot)

    pos_err = np.linalg.norm(sim_pos - kin_pos)
    vel_err = np.linalg.norm(sim_linvel - kin_linvel)
    acc_err = np.linalg.norm(sim_linacc - kin_linacc)
    qdot_err = np.linalg.norm(q_dot - qdot_calc)
    qddot_err = np.linalg.norm(q_ddot - qddot_calc)

    pos_ok = pos_err <= pos_tol
    vel_ok = vel_err <= vel_tol
    acc_ok = acc_err <= acc_tol
    qdot_ok = qdot_err <= qdot_tol
    qddot_ok = qddot_err <= qddot_tol

    status = '✓ PASS' if (pos_ok and vel_ok and acc_ok and qdot_ok and qddot_ok) else '✗ FAIL'
    print(f"Test {i}: {status}")
    print(f"  q = {np.round(q,4)}")
    print(f"  q_dot = {np.round(q_dot,4)}")
    print(f"  q_ddot = {np.round(q_ddot,4)}")
    print(f"  Position:     sim={np.round(sim_pos,5)}, kin={np.round(kin_pos,5)}, err={pos_err*1000:.3f}mm -> {'✓' if pos_ok else '✗'}")
    print(f"  Velocity:     sim={np.round(sim_linvel,5)}, kin={np.round(kin_linvel,5)}, err={vel_err*1000:.3f}mm/s -> {'✓' if vel_ok else '✗'}")
    print(f"  Acceleration: sim={np.round(sim_linacc,5)}, kin={np.round(kin_linacc,5)}, err={acc_err*1000:.3f}mm/s² -> {'✓' if acc_ok else '✗'}")
    print(f"  Inv Velocity: qdot_err={qdot_err*1000:.3f}mrad/s -> {'✓' if qdot_ok else '✗'}")
    print(f"  Inv Accel:    qddot_err={qddot_err*1000:.3f}mrad/s² -> {'✓' if qddot_ok else '✗'}")
    print()

    if not (pos_ok and vel_ok and acc_ok and qdot_ok and qddot_ok):
        all_pass = False

print('='*70)
print('SUMMARY')
print('='*70)
if all_pass:
    print('✓ All tests PASSED within specified tolerances')
    print(f'  Position tolerance: {pos_tol*1000:.2f}mm')
    print(f'  Velocity tolerance: {vel_tol*1000:.2f}mm/s')
    print(f'  Acceleration tolerance: {acc_tol*1000:.2f}mm/s²')
    print(f'  Joint velocity (inverse) tolerance: {qdot_tol*1000:.2f}mrad/s')
    print(f'  Joint acceleration (inverse) tolerance: {qddot_tol*1000:.2f}mrad/s²')
    sys.exit(0)
else:
    print('✗ Some tests FAILED — see details above')
    sys.exit(1)
