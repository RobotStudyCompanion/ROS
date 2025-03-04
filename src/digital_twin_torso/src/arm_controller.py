!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # Publish at 10 Hz
        self.joint_state = JointState()
        
        # Initialize joint state names and default positions
        self.joint_state.name = ['flipper_r_to_body', 'flipper_l_to_body', 'base_to_body']
        self.joint_state.position = [0.0, 0.0, 0.0]  # Initial positions for all joints

        # Variable to control rotation angle
        self.angle = 0.0

    def publish_joint_states(self):
        # Update joint angles for an oscillating pattern
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # Example oscillation pattern for each joint
        self.joint_state.position[0] = 0.5 * math.sin(self.angle)  # arm1_to_pivot1 angle
        self.joint_state.position[1] = 0.5 * math.cos(self.angle)  # arm_1_to_pivot angle
        self.joint_state.position[2] = 0.5 * math.sin(self.angle)  # pivot_to_base angle

        # Publish the joint states
        self.publisher_.publish(self.joint_state)

        # Increment angle for smooth oscillation
        self.angle += 0.05  # Adjust this for speed of oscillation

def main(args=None):
    rclpy.init(args=args)
    arm_controller = ArmController()
    
    try:
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        pass

    arm_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
