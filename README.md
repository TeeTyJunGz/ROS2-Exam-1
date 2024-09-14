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
3. Add **pizza_position** to the project directory:
   ```bash
   mkdir pizza_position
   ```
4. Build the workspace:
   ```bash
   colcon build
   ```
5. Source the workspace:
   ```bash
   source install/setup.bash
   ```
6. Add source to .bashrc
   ```bash
   echo "source ~/ROS2-Exam-1/install/setup.bash" >> ~/.bashrc
   ```
   
## **Usage**
You can launch the project using the provided ROS2 launch file. The launch file allows you to control up to four turtles, whose names can be customized using the **turtle_name_controller** argument.

### **Launching the Simulation**
To start the simulation with **default turtle name**, run the following command:

   ```bash
   ros2 launch turtlesim_promax teleop.launch.py
   ```
   - Default turtle name is **turtle1**

You can start the simulation with **custom turtle name** by passing the turtle_name_controller argument, run the following command:

   ```bash
   ros2 launch turtlesim_promax teleop.launch.py turtle_name_controller:={YOUR_CUSTOM_NAME}
   ```
   - Change  **{YOUR_CUSTOM_NAME}**  with your custom turtle name

#### **Example**

   ```bash
   ros2 launch turtlesim_promax teleop.launch.py turtle_name_controller:=IToon
   ```

### **Customizing Copy Turtle Names**
You can change the names of the 4 **copy** turtles in the **teleop.launch.py** file here at **line 51**.

   ```py
   copy_turtle_name = ['Foxy','Noetic','Humble','Iron']
   ```

After change.

   ```py
   copy_turtle_name = ['ITeeTy','IChokun','IBeam','IRon']
   ```

### **Launching the Teleop Keyboard Interface**
To start the Teleop Keyboard with terminal interface, run the following command:

   ```bash
   ros2 run turtlesim_promax teleop_interface_node.py
   ```

### **Additional Commands with Teleop Keyboard**
-    Spawn Pizza: Spawn pizza at your teleop turtle position.
-    Save Path: The project includes functionality to save the pizza path the turtle has create. You can use the teleop keyboard to trigger path saving.
-    Clear Pizza: Clear all unsaved pizzas from the environment using the designated service.

