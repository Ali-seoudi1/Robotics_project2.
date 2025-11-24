#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

# Import kinematics and trajectory functions
# We need forward_kinematics (T-matrix from Q) to find the start position
from mujoco_ros2.fk_ur5e import forward_kinematics 
# We need inverse_velocity_kinematics (Q_dot from Twist V) for control
from mujoco_ros2.kinematics_derivations import inverse_velocity_kinematics 
# We import the Task-Space trajectory generation function
from mujoco_ros2.trajectory_planner import generate_task_trajectory_position

class TrajectoryController(Node):
    def __init__(self):
        super().__init__('trajectory_controller')

        # --- Trajectory Parameters ---
        self.TRAJECTORY_TIME = 5.0  # seconds to complete the path
        self.control_rate = 100.0   # Hz
        self.kp = 2.5               # Proportional gain for position error

        # --- Target Definition ---
        # The target orientation (e.g., tool pointing straight down)
        R_target = np.array([
            [0, 0, 1],
            [0, 1, 0],
            [-1, 0, 0]
        ])
        
        # --- Task Space Final Point (XYZ in meters) ---
        # NOTE: This is the target point the trajectory will aim for.
        self.p_final = np.array([0.4, 0.6, 0.3]) 
        self.T_target = np.eye(4)
        self.T_target[:3, :3] = R_target
        self.T_target[:3, 3] = self.p_final
        
        # --- Internal State ---
        self.current_q = None           # Current joint angles
        self.p_start = None             # Start position (discovered on first callback)
        # Stores (p_traj, p_dot_traj, p_ddot_traj) from the planner
        self.trajectory = None          
        self.trajectory_step = 0        # Current index in the trajectory
        self.total_trajectory_steps = 0 # Total length of the trajectory array

        # --- ROS 2 Publishers & Subscribers ---
        self.publisher_ = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        # --- Control Loop ---
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)
        
        self.get_logger().info('✅ Task-Space Trajectory Controller started. Waiting for joint state...')


    def joint_state_callback(self, msg):
        """Update the current joint state and initialize the trajectory on first call."""
        self.current_q = np.array(msg.position)
        
        # Initialize the trajectory on the very first received joint state
        if self.trajectory is None:
            # 1. Get the current position (p_start) using FK
            T_start = forward_kinematics(self.current_q)
            self.p_start = T_start[:3, 3]
            
            # 2. Generate the task-space trajectory
            self.get_logger().info(f"Trajectory Start (P): {np.round(self.p_start, 3)}")
            self.get_logger().info(f"Trajectory End (P): {np.round(self.p_final, 3)}")
            self.trajectory = generate_task_trajectory_position(
                self.p_start, self.p_final, self.TRAJECTORY_TIME, self.control_rate
            )
            self.total_trajectory_steps = len(self.trajectory[0])
            self.get_logger().info(f"Generated {self.total_trajectory_steps} trajectory steps over {self.TRAJECTORY_TIME}s.")
            

    def control_loop(self):
        """Runs at control_rate Hz to follow the trajectory."""
        
        if self.current_q is None or self.trajectory is None:
            return # Wait for initialization

        # 1. Check if trajectory is finished
        if self.trajectory_step >= self.total_trajectory_steps:
            self.get_logger().info("Trajectory complete. Holding position.")
            # Send zero velocity commands to stop the arm
            self.publisher_.publish(Float64MultiArray(data=np.zeros(6).tolist()))
            self.destroy_timer(self.timer) # Stop the control loop
            return

        # --- Get current and target states from trajectory ---
        p_traj, p_dot_traj, _ = self.trajectory
        
        # Target for this time step:
        p_target_step = p_traj[self.trajectory_step]
        v_feedforward = p_dot_traj[self.trajectory_step] # Desired velocity for feedforward control

        # --- Get current end-effector pose ---
        T_current = forward_kinematics(self.current_q)
        pos_current = T_current[:3, 3]
        
        # --- Calculate Error and Control Input (Combined Feedforward + Feedback) ---
        
        # 2. Position Error (P-control component)
        pos_error = p_target_step - pos_current
        v_feedback = self.kp * pos_error
        
        # 3. Desired Linear Velocity (Twist)
        # Total linear velocity = Feedforward (from trajectory) + Feedback (P-controller)
        v_linear_desired = v_feedforward + v_feedback
        
        # Angular Velocity (Maintain the constant target orientation)
        # Note: We are only controlling position actively here.
        v_angular_desired = np.zeros(3) 

        twist_desired = np.concatenate([v_linear_desired, v_angular_desired])

        # 4. Calculate desired joint velocities (Inverse Velocity Kinematics)
        # q_dot = J_inv * V
        q_dot = inverse_velocity_kinematics(self.current_q, twist_desired)

        # 5. Publish the joint velocity command
        msg = Float64MultiArray()
        msg.data = q_dot.tolist()
        self.publisher_.publish(msg)
        
        # Advance the trajectory step
        self.trajectory_step += 1


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.timer:
            node.get_logger().info("Shutting down Trajectory Controller.")
            # Send zero velocities to stop the arm
            msg = Float64MultiArray()
            msg.data = [0.0] * 6
            node.publisher_.publish(msg)
            
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()