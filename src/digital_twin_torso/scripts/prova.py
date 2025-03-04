#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class Anger(Node):
    def __init__(self):
        super().__init__('anger')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.control_motion)  # Publish at 10 Hz
        self.joint_state = JointState()

        # Initialize joint state names and default positions
        self.joint_state.name = ['arm1_to_pivot', 'arm2_to_pivot',  'base_to_body']
        self.joint_state.position = [0.0, 0.0, 0.0]  # Default positions for all joints

        # Define the top (maximum) positions for the joints
        self.max_positions = [3.0, 2.2, 0.5]  # Adjust these values based on your robot's limits
        self.min_positions = [2.2, 3.0, -0.5]  # Lower positions for "shaking"

        # Define the increment for smooth movement
        self.increment = -0.2

        # State variables
        self.moving_up = True  # True if moving towards the top position
        self.shake_up = False
        self.shake_count = 0   # Count of shake cycles
        self.max_shakes = 4    # Number of shakes to perform
        self.dir=1

    def control_motion(self):
        # Update the timestamp
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        self.shake_body()
    
        # Publish the updated joint states
        self.publisher_.publish(self.joint_state)

    def shake_body(self):
        if self.dir ==1:
            self.joint_state.position[2]-=self.increment
        else: 
            self.joint_state.position[2]+=self.increment
        if self.joint_state.position[2]>=self.max_positions[2] or self.joint_state.position[2]<=self.min_positions[2]:
            self.dir = self.dir*-1



def main(args=None):
    rclpy.init(args=args)
    joy_node = Anger()
    
    try:
        rclpy.spin(joy_node)
    except KeyboardInterrupt:
        pass

    joy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

