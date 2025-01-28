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
    pass


class Fun(Node):

    def __init__(self):
        super().__init__('fun')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.control_motion)  # Publish at 10 Hz
        self.joint_state = JointState()

        # Initialize joint state names and default positions
        self.joint_state.name = ['arm1_to_pivot', 'arm2_to_pivot']
        self.joint_state.position = [0.0, 0.0]  # Default positions for all joints

        # Define the top (maximum) positions for the joints
        self.max_positions = [3.0, 3.0]  # Adjust these values based on your robot's limits

        # Define the increment for smooth movement
        self.increment = -0.5

        # State variables
        self.shake_count = 0   # Count of shake cycles
        self.max_shakes = 50    # Number of shakes to perform

    def control_motion(self):
        # Update the timestamp
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        if self.shake_count<self.max_shakes:
            self.move_arms()
        else:
            self.get_logger().info('Motion complete.')
            self.timer.cancel()  # Stop the timer

        # Publish the updated joint states
        self.publisher_.publish(self.joint_state)

    def move_arms(self):
        """Gradually move arms to the top position."""
        self.joint_state.position[0] += self.increment
        self.joint_state.position[1] -= self.increment
        self.shake_count+=1
        print(self.shake_count)

    pass


class Caring(Node):

    def __init__(self):
        super().__init__('caring')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.control_motion)  # Publish at 10 Hz
        self.joint_state = JointState()

        # Initialize joint state names and default positions
        self.joint_state.name = ['arm1_to_pivot', 'arm2_to_pivot']
        self.joint_state.position = [0.0, 0.0]  # Default positions for all joints

        # Define the top (maximum) positions for the joints
        self.max_positions = [0.7, -0.5]  # Adjust these values based on your robot's limits
        self.min_positions = [-0.5, 0.7]  # Lower positions for "shaking"

        # Define the increment for smooth movement
        self.increment = -0.2

        # State variables
        self.shake_up = True
        self.shake_count = 0   # Count of shake cycles
        self.max_shakes = 4    # Number of shakes to perform

    def control_motion(self):
        # Update the timestamp
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        if self.shake_up and self.shake_count<self.max_shakes:
            # Move arms to the top
            self.shake_arms_top()
        elif self.shake_count<self.max_shakes:
            self.shake_arms_down()
        else:
            self.get_logger().info('Motion complete.')
            self.timer.cancel()  # Stop the timer

        # Publish the updated joint states
        self.publisher_.publish(self.joint_state)

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
    pass

class Surprise(Node):

    def __init__(self):
        super().__init__('joy')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.control_motion)  # Publish at 10 Hz
        self.joint_state = JointState()

        # Initialize joint state names and default positions
        self.joint_state.name = ['arm1_to_pivot', 'arm2_to_pivot']
        self.joint_state.position = [0.0, 0.0]  # Default positions for all joints

        # Define the top (maximum) positions for the joints
        self.max_positions = [3.0, 3.0]  # Adjust these values based on your robot's limits
        self.min_positions = [2.2, 2.2]  # Lower positions for "shaking"

        # Define the increment for smooth movement
        self.increment = -0.3

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
            self.shake_arms_down()
        else:
            self.get_logger().info('Motion complete.')
            self.timer.cancel()  # Stop the timer

        # Publish the updated joint states
        self.publisher_.publish(self.joint_state)

    def move_arms_to_top(self):
        """Gradually move arms to the top position."""
        for i in range(len(self.joint_state.position)):
            if self.joint_state.position[i] > -self.max_positions[i]:
                self.joint_state.position[i] += self.increment*2
                print(self.joint_state.position[0])



        if all(self.joint_state.position[i] <= -self.max_positions[i] for i in range(len(self.joint_state.position))):
            print(self.joint_state.position[0])
            self.get_logger().info('Arms have reached the top.')
            self.moving_up = False  # Switch to shaking motion

    def shake_arms_top(self):
        for i in range(len(self.joint_state.position)):
            # If moving up
            self.joint_state.position[i] += self.increment*2
        # Log shake count
        if not self.joint_state.position[i] > -self.max_positions[i]:
            self.shake_count += 1  # Count the shake cycle
            self.get_logger().info(f'Shake count: {self.shake_count}')
            self.shake_up=False
     
    def shake_arms_down(self):
        for i in range(len(self.joint_state.position)):
            # If moving down
            self.joint_state.position[i] -= self.increment*2
        if not self.joint_state.position[i] < -self.min_positions[i]:
            self.shake_up=True
    pass

class Anger(Node):
    def __init__(self):
        super().__init__('anger')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.control_motion)  # Publish at 10 Hz
        self.joint_state = JointState()

        # Initialize joint state names and default positions
        self.joint_state.name = ['arm1_to_pivot', 'arm2_to_pivot']
        self.joint_state.position = [0.0, 0.0]  # Default positions for all joints

        # Define the top (maximum) positions for the joints
        self.max_positions = [3.0, 2.2]  # Adjust these values based on your robot's limits
        self.min_positions = [2.2, 3.0]  # Lower positions for "shaking"

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
            self.joint_state.position[0] += self.increment*2
            print(self.joint_state.position[0])
        if self.joint_state.position[1] > -self.max_positions[1]:
            self.joint_state.position[1] += self.increment*2
            print(self.joint_state.position[1])

        if all(self.joint_state.position[i] <= -self.max_positions[i] for i in range(len(self.joint_state.position))):
            print(self.joint_state.position[0])
            self.get_logger().info('Arms have reached the top.')
            self.moving_up = False 

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
    pass


def main():
    nodes = {
        "1": Joy,
        "2": Fun,
        "3": Caring,
        "4": Joy,  
        "5": Anger,
        "6": Surprise,
    }

    print("Choose a node to run:")
    print("1: Joy (Node 1)")
    print("2: Fun (Node 2)")
    print("3: Caring (Node 3)")
    print("4: Joy (Node 4)")
    print("5: Anger (Node 5)")
    print("6: Surprise (Node 6)")

    choice = input("Enter your choice (1-6): ").strip()

    if choice in nodes:
        rclpy.init()
        try:
            node = nodes[choice]()
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        print("Invalid choice. Exiting.")


if __name__ == '__main__':

        main()
