#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class Joy(Node):
    def __init__(self):
        super().__init__('joy')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.control_motion)  # Publish at 10 Hz
        self.joint_state = JointState()

        # Initialize joint state names and default positions
        self.joint_state.name = ['arm1_to_pivot', 'arm2_to_pivot']
        self.joint_state.position = [0.0, 0.0]  # Default positions for all joints

        # Define the top (maximum) positions for the joints
        self.max_positions = [1.8, 1.6]  # Adjust these values based on your robot's limits
        self.min_positions = [1.6, 1.8]  # Lower positions for "shaking"

        # Define the increment for smooth movement
        self.increment = -0.2

        # State variables
        self.moving_up = True  # True if moving towards the top position
        self.shake_up = False
        self.shake_count = 0   # Count of shake cycles
        self.max_shakes = 4    # Number of shakes to perform

    def control_motion(self):
        # Update the timestamp
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        if self.moving_up:
            # Perform shaking motion
            self.move_arms_to_top()
        elif self.shake_up and self.shake_count<self.max_shakes:
            # Move arms to the top
            self.shake_arms_top()
        elif self.shake_count<self.max_shakes:
            print("here")
            self.shake_arms_down()
        else:
            self.get_logger().info('Motion complete.')
            self.timer.cancel()  # Stop the timer

        # Publish the updated joint states
        self.publisher_.publish(self.joint_state)

    def move_arms_to_top(self):
        """Gradually move arms to the top position."""

        if self.joint_state.position[0] > -self.max_positions[0]:
            self.joint_state.position[0] += self.increment
            print(self.joint_state.position[0])
        if self.joint_state.position[1] > -self.max_positions[1]:
            self.joint_state.position[1] += self.increment
            print(self.joint_state.position[1])
                # Ensure the joint does not exceed the maximum position
                #self.joint_state.position[i] = max(self.joint_state.position[i], -self.max_positions[i])

        # Check if all joints have reached their maximum positions
        if all(self.joint_state.position[i] <= -self.max_positions[i] for i in range(len(self.joint_state.position))):
            print(self.joint_state.position[0])
            self.get_logger().info('Arms have reached the top.')
            self.moving_up = False  # Switch to shaking motion
    def shake_arms_top(self):
        print("going up")
        print(self.joint_state.position[0] , -self.max_positions[0])
        if self.joint_state.position[0] > -self.max_positions[0]:
            self.joint_state.position[0] += self.increment
            print(self.joint_state.position[0])
        if self.joint_state.position[1] < -self.max_positions[1]:
            self.joint_state.position[1] -= self.increment
            print(self.joint_state.position[1])
        if self.joint_state.position[0] <= -self.max_positions[0]:
            self.shake_up=False
            self.shake_count+=1
            print("->false")
     
    def shake_arms_down(self):
        print("going down")
        print(self.joint_state.position[0] , self.min_positions[0])
        if self.joint_state.position[0] < -self.min_positions[0]:
            self.joint_state.position[0] -= self.increment
            print(self.joint_state.position[0])
        if self.joint_state.position[1] > -self.min_positions[1]:
            self.joint_state.position[1] += self.increment
            print(self.joint_state.position[1])
        if self.joint_state.position[0] >= -self.min_positions[0]:
            self.shake_up=True

def main(args=None):
    rclpy.init(args=args)
    caring_node = Joy()
    
    try:
        rclpy.spin(caring_node)
    except KeyboardInterrupt:
        pass

    caring_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

