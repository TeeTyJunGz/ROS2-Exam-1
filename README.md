# **ROS2 Turtlesim Promax Project**
This is a **ROS2-based project** that extends the functionality of the default **Turtlesim package**. 
The **Turtlesim_promax** project allows you to control a turtle using a **teleop keyboard interface**, **save the turtle's path**, **spawn and clear pizzas**
The project is configured using **ROS2 launch files**, where you can specify turtle names and manage multiple turtles.

## **Features**
-  **Teleop Keyboard Control**: Control the turtle's movement via the keyboard with terminal interface.
-  **Save Path**: Save the turtle's path as it moves around the simulation and clone it with another monitor.
-  **Pizza Handling**: Spawn pizzas in the environment and clear them when desired.
-  **Multiple Turtles**: Manage and control multiple **copy** turtles with custom names using launch files.

## **Prerequisites**

- [ROS2 Humble (or your preferred ROS2 distribution)](https://docs.ros.org/en/humble/Installation.html)
- Colcon build system

## **Installation**

1. Clone this repository:
   ```bash
   git clone https://github.com/TeeTyJunGz/ROS2-Exam-1.git
   ```
2. Navigate to the project directory:
   ```bash
   cd ROS2-Exam-1
   ```
3. Build the workspace:
   ```bash
   colcon build
   ```
4. Source the workspace:
   ```bash
   source install/setup.bash
   ```
