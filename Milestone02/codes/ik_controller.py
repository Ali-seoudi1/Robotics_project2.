#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import mujoco
import numpy as np
import os

from mujoco_ros2.ik_ur5e import inverse_kinematics

from mujoco_ros2.fk_ur5e import forward_kinematics



class IKController(Node):
    def __init__(self):
        super().__init__('ik_controller')

        # Create ROS2 publisher for joint states
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        # Path to your MuJoCo model (make sure this is correct)
        current_dir = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(current_dir, '../model/ur5e.xml')

        # Load MuJoCo model and data
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        self.get_logger().info('✅ MuJoCo model loaded successfully!')

        # Example: Set a timer to run simulation loop
        self.timer = self.create_timer(0.01, self.simulation_step)

    def simulation_step(self):
        """This function runs every 10ms."""
        # Run one MuJoCo step
        mujoco.mj_step(self.model, self.data)

        # Example: publish joint states to ROS2
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'joint_{i+1}' for i in range(self.model.nq)]
        msg.position = self.data.qpos.tolist()
        self.publisher_.publish(msg)

        # You can also compute IK/FK here if needed
        # Example (commented out):
        # desired_pose = np.array([0.3, 0.2, 0.4])
        # q_sol = inverse_kinematics(desired_pose)
        # self.data.qpos[:] = q_sol

def main(args=None):
    rclpy.init(args=args)
    node = IKController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
