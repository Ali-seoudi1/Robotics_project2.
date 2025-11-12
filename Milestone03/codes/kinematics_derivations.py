#!/usr/bin/env python3
import numpy as np

# ===============================
#   DH Parameters (UR5e)
# ===============================
# Copied from your fk_ur5e.py
dh_params = [
    [0, np.pi/2, 0.1625, 0],
    [-0.425, 0, 0, 0],
    [-0.3922, 0, 0, 0],
    [0, np.pi/2, 0.1333, 0],
    [0, -np.pi/2, 0.0997, 0],
    [0, 0, 0.0996, 0]
]

def dh_matrix(a, alpha, d, theta):
    """Calculates the DH transformation matrix."""
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def forward_kinematics_all_frames(joint_angles):
    """
    Calculates the forward kinematics, returning the transformation matrix
    for each joint frame relative to the base.
    """
    T_matrices = [np.eye(4)] # List to store T_0^0, T_0^1, ..., T_0^6
    T_current = np.eye(4)
    
    for i in range(6):
        a, alpha, d, _ = dh_params[i]
        # Note: joint_angles[i] is the theta for this joint
        T_local = dh_matrix(a, alpha, d, joint_angles[i])
        T_current = T_current @ T_local
        T_matrices.append(T_current)
        
    return T_matrices

def get_pose_from_transform(T):
    """Extracts position (x, y, z) and orientation (roll, pitch, yaw) from a 4x4 T-matrix."""
    pos = T[:3, 3]
    
    # RPY from rotation matrix
    sy = np.sqrt(T[0,0] * T[0,0] +  T[1,0] * T[1,0])
    singular = sy < 1e-6
    
    if not singular :
        roll = np.arctan2(T[2,1] , T[2,2])
        pitch = np.arctan2(-T[2,0], sy)
        yaw = np.arctan2(T[1,0], T[0,0])
    else :
        roll = np.arctan2(-T[1,2], T[1,1])
        pitch = np.arctan2(-T[2,0], sy)
        yaw = 0
        
    orientation = np.array([roll, pitch, yaw])
    return pos, orientation

def hat_operator(v):
    """Computes the skew-symmetric 'hat' matrix for a 3x1 vector."""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

def calculate_jacobian(q):
    """
    Calculates the 6x6 analytical Jacobian matrix.
    J = [J_p; J_o]
    """
    T_matrices = forward_kinematics_all_frames(q)
    p_end = T_matrices[-1][:3, 3] # End-effector position p_n
    
    J = np.zeros((6, 6))
    
    for i in range(6):
        T_0_i = T_matrices[i] # This is T_0^{i-1} in the formula (since list is 0-indexed)
        z_i = T_0_i[:3, 2]    # Z-axis of the i-th frame (z_{i-1})
        p_i = T_0_i[:3, 3]    # Position of the i-th frame (p_{i-1})
        
        # Linear velocity component (for revolute joint)
        J_p_i = np.cross(z_i, p_end - p_i)
        
        # Angular velocity component (for revolute joint)
        J_o_i = z_i
        
        J[:3, i] = J_p_i
        J[3:, i] = J_o_i
        
    return J

def calculate_jacobian_derivative(q, q_dot, delta_t=1e-6):
    """
    Calculates the time derivative of the Jacobian (J_dot) numerically.
    J_dot = (J(q + q_dot*dt) - J(q)) / dt
    """
    J = calculate_jacobian(q)
    J_next = calculate_jacobian(q + q_dot * delta_t)
    J_dot = (J_next - J) / delta_t
    return J_dot

# =======================================================
#   FORWARD/INVERSE VELOCITY/ACCELERATION KINEMATICS
# =======================================================

def forward_velocity_kinematics(q, q_dot):
    """
    Calculates end-effector twist (V) from joint angles (q) and velocities (q_dot).
    V = J(q) * q_dot
    """
    J = calculate_jacobian(q)
    twist = J @ q_dot
    return twist

def inverse_velocity_kinematics(q, twist):
    """
    Calculates joint velocities (q_dot) from joint angles (q) and end-effector twist (V).
    q_dot = J_inv(q) * V
    Uses the pseudoinverse for robustness against singularities.
    """
    J = calculate_jacobian(q)
    J_inv = np.linalg.pinv(J) # Pseudoinverse
    q_dot = J_inv @ twist
    return q_dot

def forward_acceleration_kinematics(q, q_dot, q_ddot):
    """
    Calculates end-effector acceleration (V_dot) from joint state.
    V_dot = J_dot(q, q_dot) * q_dot + J(q) * q_ddot
    """
    J = calculate_jacobian(q)
    J_dot = calculate_jacobian_derivative(q, q_dot)
    
    twist_dot = (J_dot @ q_dot) + (J @ q_ddot)
    return twist_dot

def inverse_acceleration_kinematics(q, q_dot, twist_dot):
    """
    Calculates joint accelerations (q_ddot) from joint state and desired EE acceleration (V_dot).
    q_ddot = J_inv(q) * (V_dot - J_dot(q, q_dot) * q_dot)
    """
    J = calculate_jacobian(q)
    J_inv = np.linalg.pinv(J) # Pseudoinverse
    J_dot = calculate_jacobian_derivative(q, q_dot)
    
    q_ddot = J_inv @ (twist_dot - (J_dot @ q_dot))
    return q_ddot

# ===============================
#   Example Usage
# ===============================
if __name__ == "__main__":
    # --- Example joint state ---
    q = np.array([0, -np.pi/4, np.pi/2, 0, np.pi/3, 0])
    q_dot = np.array([0.1, 0, 0.1, 0, 0, 0])
    q_ddot = np.array([0.01, 0, 0, 0.02, 0, 0])
    
    print("===== Kinematics Derivations Demo =====")
    print(f"Joint Angles (q): {np.round(q, 3)}")
    print(f"Joint Velocities (q_dot): {np.round(q_dot, 3)}")
    print(f"Joint Accelerations (q_ddot): {np.round(q_ddot, 3)}")

    # --- Jacobian ---
    J = calculate_jacobian(q)
    print(f"\nJacobian J(q) (6x6 matrix):\n{np.round(J, 3)}")

    # --- Fwd Velocity Kinematics ---
    twist = forward_velocity_kinematics(q, q_dot)
    print(f"\nForward Velocity (Twist V): \n{np.round(twist, 3)}")
    
    # --- Inv Velocity Kinematics ---
    q_dot_calc = inverse_velocity_kinematics(q, twist)
    print(f"\nInverse Velocity (q_dot_calc): \n{np.round(q_dot_calc, 3)}")
    print(f"  (Error: {np.linalg.norm(q_dot - q_dot_calc):.2e})")

    # --- Fwd Acceleration Kinematics ---
    twist_dot = forward_acceleration_kinematics(q, q_dot, q_ddot)
    print(f"\nForward Acceleration (Twist_dot): \n{np.round(twist_dot, 3)}")
    
    # --- Inv Acceleration Kinematics ---
    q_ddot_calc = inverse_acceleration_kinematics(q, q_dot, twist_dot)
    print(f"\nInverse Acceleration (q_ddot_calc): \n{np.round(q_ddot_calc, 3)}")
    print(f"  (Error: {np.linalg.norm(q_ddot - q_ddot_calc):.2e})")