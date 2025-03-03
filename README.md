# My Robot Sim

## Features

- **Robot Visualization:** View the virtual twin of the RSC in **RViz2**.
- **Customizable Appearance:** Change the robot’s colors and facial expressions.
- **Flipper Control:** Utilize ROS 2 nodes to move the robot’s flippers.
- **Modular Design:** Easily extend and integrate with other ROS 2-based projects.

## Installation

### Prerequisites

Ensure you have the following installed:

- **ROS 2 Humble**
- **RViz2** for visualization

### Steps

1. Clone the repository into your ROS 2 workspace:
   ```sh
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/my_robot_sim.git
   ```
2. Build the package:
   ```sh
   cd ~/ros2_ws
   colcon build --packages-select my_robot_sim
   ```
3. Source the workspace:
   ```sh
   source install/setup.bash
   ```
4. Launch the simulation:
   ```sh
   ros2 launch my_robot_sim display_robot.launch.py
   ```

## Usage

### Running the Simulation

To start the robot simulation in RViz2:

```sh
ros2 launch my_robot_sim display_robot.launch.py
```

### Changing Colors and Faces

Modify the robot’s appearance by adjusting parameters in the **config/** directory.

### Controlling Flippers

Run the following node to control the flippers:

```sh
ros2 run my_robot_sim emotion.py
```

## Contact

For questions or support, reach out via [**c**](mailto\:your.email@example.com)[**alafa@ut.ee**](mailto\:alafa@ut.ee) or open an issue in the repository.

---

Happy coding! 🚀


