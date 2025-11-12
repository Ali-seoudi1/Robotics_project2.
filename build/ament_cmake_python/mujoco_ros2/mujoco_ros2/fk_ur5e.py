import numpy as np

# UR5e DH parameters
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

# Example: test with sample joint values
if __name__ == "__main__":
    theta = [0, -np.pi/4, np.pi/2, 0, np.pi/3, 0]
    T06 = forward_kinematics(theta)
    print("End-effector pose:\n", np.round(T06, 4))
