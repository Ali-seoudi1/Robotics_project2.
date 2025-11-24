#!/usr/bin/env python3
import numpy as np

# =======================================================
#   Cubic Polynomial Trajectory Derivation
# =======================================================
# The position function s(t) is defined by a cubic polynomial:
# s(t) = a0 + a1*t + a2*t^2 + a3*t^3
# 
# The boundary conditions (start time t0=0, end time tf) are:
# 1. s(0) = s_start    (Initial Position)
# 2. s(tf) = s_final   (Final Position)
# 3. s_dot(0) = 0      (Initial Velocity)
# 4. s_dot(tf) = 0     (Final Velocity)
#
# Solving for the coefficients (a0, a1, a2, a3) yields:
# a0 = s_start
# a1 = 0
# a2 = 3 * (s_final - s_start) / tf^2
# a3 = -2 * (s_final - s_start) / tf^3

def cubic_polynomial_trajectory(s_start, s_final, time_points):
    """
    Generates a smooth trajectory between s_start and s_final using a cubic polynomial.
    
    Args:
        s_start (np.ndarray): The starting state (position or joint angle vector).
        s_final (np.ndarray): The final state.
        time_points (np.ndarray): An array of time instances [t0, t1, ..., tf].
        
    Returns:
        tuple: (positions, velocities, accelerations) arrays over the time points.
    """
    if len(time_points) == 0:
        return np.array([]), np.array([]), np.array([])
        
    # Total time of the movement
    t0 = time_points[0]
    tf = time_points[-1]
    T = tf - t0
    
    # Handle the trivial case where T is zero or start == final
    if T < 1e-6:
        # Return the start state for all time points
        num_steps = len(time_points)
        s_vec = np.tile(s_start, (num_steps, 1))
        zero_vec = np.zeros_like(s_vec)
        return s_vec, zero_vec, zero_vec

    # Calculate coefficients
    delta_s = s_final - s_start
    a0 = s_start
    a1 = np.zeros_like(s_start)
    a2 = 3 * delta_s / (T**2)
    a3 = -2 * delta_s / (T**3)

    num_steps = len(time_points)
    dim = s_start.shape[0]
    
    positions = np.zeros((num_steps, dim))
    velocities = np.zeros((num_steps, dim))
    accelerations = np.zeros((num_steps, dim))

    # Generate points for each time instance t
    for i, t in enumerate(time_points):
        t_scaled = t - t0
        
        # Position s(t) = a0 + a1*t_scaled + a2*t_scaled^2 + a3*t_scaled^3
        positions[i] = a0 + a1 * t_scaled + a2 * (t_scaled**2) + a3 * (t_scaled**3)
        
        # Velocity s_dot(t) = a1 + 2*a2*t_scaled + 3*a3*t_scaled^2
        velocities[i] = a1 + 2 * a2 * t_scaled + 3 * a3 * (t_scaled**2)
        
        # Acceleration s_ddot(t) = 2*a2 + 6*a3*t_scaled
        accelerations[i] = 2 * a2 + 6 * a3 * t_scaled
        
    return positions, velocities, accelerations

# =======================================================
#   High-Level Trajectory Planning Functions
# =======================================================

def generate_joint_trajectory(q_start, q_final, total_time, rate_hz):
    """
    Derives the joint-space trajectory.
    
    Args:
        q_start (np.ndarray): Starting joint angles (6x1).
        q_final (np.ndarray): Final joint angles (6x1).
        total_time (float): Time duration for the trajectory in seconds.
        rate_hz (float): Sampling rate (Hz) for the trajectory steps.
        
    Returns:
        tuple: (q_traj, q_dot_traj, q_ddot_traj)
    """
    time_points = np.linspace(0, total_time, int(total_time * rate_hz), endpoint=True)
    
    q_traj, q_dot_traj, q_ddot_traj = cubic_polynomial_trajectory(q_start, q_final, time_points)
    
    return q_traj, q_dot_traj, q_ddot_traj


def generate_task_trajectory_position(p_start, p_final, total_time, rate_hz):
    """
    Derives the position component of the Task-Space trajectory.
    
    Args:
        p_start (np.ndarray): Starting XYZ position (3x1).
        p_final (np.ndarray): Final XYZ position (3x1).
        ... (time args)
        
    Returns:
        tuple: (p_traj, p_dot_traj, p_ddot_traj)
    """
    time_points = np.linspace(0, total_time, int(total_time * rate_hz), endpoint=True)
    
    p_traj, p_dot_traj, p_ddot_traj = cubic_polynomial_trajectory(p_start, p_final, time_points)
    
    return p_traj, p_dot_traj, p_ddot_traj
    
# NOTE: Orientation trajectory (rotation) planning is often done using spherical linear
# interpolation (slerp) on quaternions, which is much more complex. For simplicity,
# this example focuses on position and assumes a constant orientation.


if __name__ == '__main__':
    print("--- Trajectory Planner Demo ---")

    # 1. Joint-Space Example
    q_start = np.array([0.0, -0.7, 0.5, 0.0, 0.5, 0.0])
    q_final = np.array([0.5, -1.0, 1.2, 0.0, 0.8, 0.0])
    T_time = 5.0
    Rate = 100.0
    
    q_traj, q_dot_traj, q_ddot_traj = generate_joint_trajectory(q_start, q_final, T_time, Rate)
    
    print(f"\nJoint Trajectory (T={T_time}s, Steps={len(q_traj)}):")
    print(f"  Start q: {np.round(q_traj[0], 3)}")
    print(f"  Final q: {np.round(q_traj[-1], 3)}")
    print(f"  Start q_dot (should be 0): {np.round(q_dot_traj[0], 3)}")
    print(f"  Final q_dot (should be 0): {np.round(q_dot_traj[-1], 3)}")
    
    # 2. Task-Space (Position) Example
    p_start = np.array([0.5, 0.1, 0.2])
    p_final = np.array([0.2, 0.6, 0.4])
    
    p_traj, p_dot_traj, p_ddot_traj = generate_task_trajectory_position(p_start, p_final, T_time, Rate)
    
    print(f"\nTask-Space Trajectory (T={T_time}s, Steps={len(p_traj)}):")
    print(f"  Start p: {np.round(p_traj[0], 3)}")
    print(f"  Final p: {np.round(p_traj[-1], 3)}")