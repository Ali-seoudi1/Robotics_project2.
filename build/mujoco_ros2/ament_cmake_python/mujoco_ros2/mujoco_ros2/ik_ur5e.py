import numpy as np
from scipy.optimize import minimize

# ===============================
#   DH Parameters (UR5e)
# ===============================
dh_params = [
    [0, np.pi/2, 0.1625, 0],
    [-0.425, 0, 0, 0],
    [-0.3922, 0, 0, 0],
    [0, np.pi/2, 0.1333, 0],
    [0, -np.pi/2, 0.0997, 0],
    [0, 0, 0.0996, 0]
]

def dh_matrix(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(joint_angles):
    T = np.eye(4)
    for i in range(6):
        a, alpha, d, _ = dh_params[i]
        T = T @ dh_matrix(a, alpha, d, joint_angles[i])
    return T

# ===============================
#   Inverse Kinematics (Numerical)
# ===============================

def pose_error(joint_angles, target_pose):
    """Compute the combined position and orientation error."""
    T = forward_kinematics(joint_angles)
    pos_error = np.linalg.norm(T[:3, 3] - target_pose[:3, 3])
    ori_error = np.linalg.norm(T[:3, :3] - target_pose[:3, :3])
    return pos_error + 0.1 * ori_error  # weighted to favor position

def inverse_kinematics(target_pose, initial_guess=None):
    """Find joint angles that reach the target pose."""
    if initial_guess is None:
        initial_guess = np.zeros(6)

    result = minimize(
        pose_error,
        initial_guess,
        args=(target_pose,),
        method='BFGS',
        options={'maxiter': 1000, 'disp': False}
    )
    return result.x, result.fun

# ===============================
#   Example Usage
# ===============================
if __name__ == "__main__":
    # Desired end-effector pose (same format as FK result)
    target_pose = np.array([
        [0, 0, 1, 0.4],
        [0, 1, 0, 0.2],
        [-1, 0, 0, 0.3],
        [0, 0, 0, 1]
    ])

    ik_solution, error = inverse_kinematics(target_pose)
    print("IK solution (radians):", np.round(ik_solution, 4))
    print("IK solution (degrees):", np.round(np.degrees(ik_solution), 2))
    print("Final error:", error)

    # Verify by forward kinematics
    T_check = forward_kinematics(ik_solution)
    print("\nReconstructed Pose:\n", np.round(T_check, 4))
