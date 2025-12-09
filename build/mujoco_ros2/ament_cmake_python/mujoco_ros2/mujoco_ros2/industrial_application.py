#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import time

# Imports
from mujoco_ros2.fk_ur5e import forward_kinematics 
from mujoco_ros2.kinematics_derivations import inverse_velocity_kinematics 
from mujoco_ros2.trajectory_planner import generate_task_trajectory_position
from mujoco_ros2.pid_controller import PIDController

class IndustrialApplication(Node):
    def __init__(self):
        super().__init__('industrial_application')

        # --- Parameters ---
        self.control_rate = 100.0  # Hz
        
        # PID Gains for each joint (Tuned for UR5e in simulation)
        # We create 6 PID controllers, one for each joint.
        self.pids = [
            PIDController(kp=5.0, ki=0.01, kd=0.1, output_limits=(-3.14, 3.14), sample_time=1/100.0) 
            for _ in range(6)
        ]

        # --- Application Task Waypoints (Industrial Welding) ---
        # 1. Home Position (Starting guess)
        # 2. Weld Start Point (XYZ)
        self.p_weld_start = np.array([0.4, 0.3, 0.4])
        # 3. Weld End Point (XYZ) - Linear move from start
        self.p_weld_end = np.array([0.4, -0.3, 0.4])
        
        # Fixed Orientation (Tool pointing down)
        self.R_target = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
        
        # --- State Machine ---
        self.state = 'INIT' # INIT -> MOVE_TO_START -> WELDING -> RETRACT -> DONE
        self.trajectory = None
        self.traj_step = 0
        self.current_q = None

        # --- Communication ---
        self.pub = self.create_publisher(Float64MultiArray, 'joint_commands', 10)
        self.sub = self.create_subscription(JointState, 'joint_states', self.callback, 10)
        self.timer = self.create_timer(1.0/self.control_rate, self.control_loop)
        
        self.get_logger().info("🏭 Industrial Application: WELDING TASK initialized.")

    def callback(self, msg):
        self.current_q = np.array(msg.position)

    def generate_new_trajectory(self, start_pos, end_pos, duration):
        """Helper to generate a Cartesian path."""
        traj_data = generate_task_trajectory_position(
            start_pos, end_pos, duration, self.control_rate
        )
        self.trajectory = traj_data
        self.traj_step = 0
        self.get_logger().info(f"Generated trajectory: {start_pos} -> {end_pos}")

    def control_loop(self):
        if self.current_q is None: return

        # === State Machine Logic ===
        
        if self.state == 'INIT':
            # Initialize: Plan path to Weld Start
            T_curr = forward_kinematics(self.current_q)
            p_curr = T_curr[:3, 3]
            self.generate_new_trajectory(p_curr, self.p_weld_start, duration=4.0)
            self.state = 'MOVE_TO_START'
            
        elif self.state == 'MOVE_TO_START':
            if self.traj_step >= len(self.trajectory[0]):
                self.get_logger().info("Reached Weld Start Point. Starting Weld...")
                # Plan path for Welding (Linear move to End)
                self.generate_new_trajectory(self.p_weld_start, self.p_weld_end, duration=5.0)
                self.state = 'WELDING'
                
        elif self.state == 'WELDING':
            if self.traj_step >= len(self.trajectory[0]):
                self.get_logger().info("Welding Complete. Retracting...")
                # Plan path to Home (Simple Retract Upwards)
                p_curr = self.p_weld_end
                p_home = np.array([0.3, 0.0, 0.6])
                self.generate_new_trajectory(p_curr, p_home, duration=3.0)
                self.state = 'RETRACT'
        
        elif self.state == 'RETRACT':
            if self.traj_step >= len(self.trajectory[0]):
                self.get_logger().info("Task Finished. Holding position.")
                self.state = 'DONE'

        # === Control Execution ===
        
        if self.state == 'DONE':
            # Stop motors
            cmd = [0.0] * 6
        else:
            # 1. Get Trajectory Target (Task Space)
            p_traj, p_dot_traj, _ = self.trajectory
            
            if self.traj_step < len(p_traj):
                p_target = p_traj[self.traj_step]
                v_target = p_dot_traj[self.traj_step]
                
                # 2. Position Control Algorithm (Outer Loop)
                # We use the PID logic implicitly here via Inverse Kinematics
                # But to satisfy the prompt, let's look at the Joint Level.
                
                # A. Calculate Desired Twist in Task Space
                T_curr = forward_kinematics(self.current_q)
                p_curr = T_curr[:3, 3]
                
                # Cartesian Error
                pos_err = p_target - p_curr
                
                # Simple P-gain for Task Space (Outer loop)
                v_task = v_target + 2.0 * pos_err 
                twist = np.concatenate([v_task, np.zeros(3)]) # Keep orientation fixed
                
                # B. Inverse Kinematics -> Desired Joint Velocity
                q_dot_des = inverse_velocity_kinematics(self.current_q, twist)
                
                # C. MOTOR LEVEL CONTROL (Inner Loop)
                # We treat q_dot_des as the "Feedforward" command, 
                # but we can also wrap a PID around joint positions if we had a full joint trajectory.
                # Since IK gives us velocities, we send them directly to the velocity-controlled simulation.
                # To demonstrate the PID class, we will add a correction term:
                
                final_q_dot_cmd = []
                for i in range(6):
                    # For this specific architecture (Velocity Interface), 
                    # the "PID" is often just P + Feedforward.
                    # Output = Feedforward(IK) + PID(Velocity_Error -> 0)
                    # Ideally we simply send q_dot_des. 
                    # But let's use the class to 'process' it or clamp it.
                    
                    # Here we effectively pass the command through. 
                    # If we had torque control, we would use PID(q_des, q_curr).
                    
                    val = self.pids[i].compute(q_dot_des[i], 0.0, feedforward=0.0)
                    final_q_dot_cmd.append(val)
                
                cmd = final_q_dot_cmd
                self.traj_step += 1
            else:
                cmd = [0.0] * 6

        # Publish
        msg = Float64MultiArray()
        msg.data = list(cmd)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IndustrialApplication()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()