#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

# Import our new kinematics functions
from mujoco_ros2.kinematics_derivations import (
    forward_kinematics_all_frames,
    get_pose_from_transform,
    inverse_velocity_kinematics
)

# Import the FK function from your other file to get the T-matrix
from mujoco_ros2.fk_ur5e import forward_kinematics

class IKController(Node):
    def __init__(self):
        super().__init__('ik_controller')

        # --- Parameters ---
        self.control_rate = 100.0  # Hz
        self.kp = 2.5  # Proportional gain for position error
        self.ko = 1.0  # Proportional gain for orientation error

        # --- Target Pose (Position + Orientation as RPY) ---
        # Define a target 4x4 homogeneous transformation matrix
        # Example: 0.4m in x, 0.2m in y, 0.3m in z, rotated 90 deg about y
        R_target = np.array([
            [0, 0, 1],
            [0, 1, 0],
            [-1, 0, 0]
        ])
        p_target = np.array([0.4, 0.2, 0.3])
        self.T_target = np.eye(4)
        self.T_target[:3, :3] = R_target
        self.T_target[:3, 3] = p_target
        
        target_pos, target_ori = get_pose_from_transform(self.T_target)
        self.get_logger().info(f"Target Pose (Pos): {np.round(target_pos, 3)}")
        self.get_logger().info(f"Target Pose (Ori RPY): {np.round(target_ori, 3)}")

        # --- Internal State ---
        self.current_q = None # Current joint angles
        self.joint_names = [] # List of joint names

        # --- ROS 2 Publishers & Subscribers ---
        # Publishes velocity commands to the /joint_commands topic
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            'joint_commands', 
            10
        )
        
        # Subscribes to the /joint_states topic from the simulation
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        # --- Control Loop ---
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)
        
        self.get_logger().info('✅ Velocity IK Controller has started.')
        self.get_logger().info('Publishing joint velocities to /joint_commands')
        self.get_logger().info('Subscribing to /joint_states')


    def joint_state_callback(self, msg):
        """Update the current joint state."""
        # This assumes the joint_states message returns joints in the correct order
        if not self.joint_names:
            self.joint_names = msg.name
        self.current_q = np.array(msg.position)

    def control_loop(self):
        """Runs at self.control_rate Hz."""
        
        # Wait until we have received a joint state
        if self.current_q is None:
            self.get_logger().warn("Waiting for first /joint_states message...", throttle_duration_sec=5.0)
            return

        # 1. Get current end-effector pose (Forward Kinematics)
        T_current = forward_kinematics(self.current_q)
        
        # 2. Calculate the pose error
        # Position error
        pos_current = T_current[:3, 3]
        pos_target = self.T_target[:3, 3]
        pos_error = pos_target - pos_current
        
        # Orientation error
        R_current = T_current[:3, :3]
        R_target = self.T_target[:3, :3]
        # Skew-symmetric matrix of R_current * R_target.T
        R_error = R_current.T @ R_target
        # Convert rotation matrix error to axis-angle representation
        # (This is a simplified way to get an error vector)
        ori_error = 0.5 * np.array([
            R_error[2, 1] - R_error[1, 2],
            R_error[0, 2] - R_error[2, 0],
            R_error[1, 0] - R_error[0, 1]
        ])
        
        # 3. Generate desired twist (V) using a P-controller
        # This is a simple P-controller: V = Kp * error
        v_linear = self.kp * pos_error
        v_angular = self.ko * ori_error
        
        twist = np.concatenate([v_linear, v_angular])

        # 4. Calculate desired joint velocities (Inverse Velocity Kinematics)
        # q_dot = J_inv * V
        q_dot = inverse_velocity_kinematics(self.current_q, twist)

        # 5. Publish the joint velocity command
        msg = Float64MultiArray()
        msg.data = q_dot.tolist()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IKController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down IK Controller.")
        # Stop the robot by sending zero velocities
        msg = Float64MultiArray()
        msg.data = [0.0] * 6
        node.publisher_.publish(msg)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()