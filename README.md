# **ROS2 Turtlesim Promax Project**
This is a **ROS2-based project** that extends the functionality of the default **Turtlesim package**. 
The **Turtlesim_promax** project allows you to control a turtle using a **teleop keyboard interface**, **save the turtle's path**, **spawn and clear pizzas**
The project is configured using **ROS2 launch files**, where you can specify turtle names and manage multiple turtles.

## **Features**
-  **Teleop Keyboard Control**: Control the turtle's movement via the keyboard with terminal interface.
-  **Save Path**: Save the turtle's path as it moves around the simulation and clone it with another monitor.
-  **Pizza Handling**: Spawn pizzas in the environment and clear them when desired.
-  **Multiple Turtles**: Manage and control multiple **copy** turtles with custom names using launch files.
-  **Param control RQT**: Control max pizza value and Kp linear velocity gain via **RQT**

## **Preview**
[![Video showcase](https://i9.ytimg.com/vi/gXcGxA444ao/mqdefault.jpg?v=66e5fe1d&sqp=CID8l7cG-oaymwEmCMACELQB8quKqQMa8AEB-AH-BYAC4AOKAgwIABABGEggUihyMA8=&rs=AOn4CLDRon4o5QMFTmuTxAqGtUpZY7iCgg)](https://youtu.be/gXcGxA444ao)

## **System Architechture**

![systemArch](https://cdn.discordapp.com/attachments/1278095176824393891/1284625703513362452/Exam_1.jpg?ex=66e7506a&is=66e5feea&hm=a19ba236a2db6e3ff4c5976fb8d0876db9b8c1760b30712917b62c319aa1ba2a&)

- **Package:** Turtlesim Promax.
- **Nodes:**
- **Keyboard Control Node:** Sends input values from the keyboard to the Scheduler Node and Teleop Controller Node.
- **Scheduler Node:** Receives input from the Keyboard Control Node to set the state and issues commands to the Teleop Controller Node to operate according to the state.
- **Teleop Controller Node:** Controls the Turtlesim Node to display results based on the commands from the Scheduler Node and has the ability to invoke commands from the  Turtlesim Node.
- **Turtlesim Node:** Displays results according to the commands from the Teleop Controller Node.
- **Copy Scheduler Node:** Receives input from the Copy Controller Node to check the state and returns values to the Copy Controller Node when the state changes.
- **Copy Controller Node:** Controls all 4 turtles to spawn pizzas at positions saved in the YAML file and sends state values to the Copy Scheduler Node.
- **YAML File:** Stores the pizza positions from the Teleop Controller Node.
- **Launch File:** teleop.launch.py Runs all nodes together except for the Keyboard Control Node (Teleop Interface Node) and spawns 4 turtles.


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
-    **Spawn Pizza**: Spawn pizza at your teleop turtle position.
-    **Save Path**: The project includes functionality to save the pizza path the turtle has create.
-    **Clear Pizza**: Clear all unsaved pizzas from the environment using the designated service.

### **Edit Param with RQT GUI**
1. To start the RQT with GUI, run the following command:
   ```bash
   rqt
   ```
2. Import Turtlesim Pro Max Perspectives
   ![Perc](https://cdn.discordapp.com/attachments/1278095176824393891/1284627878650052660/image.png?ex=66e75271&is=66e600f1&hm=71ddc8b2d4f140457f685c92ea9b9a41be59c089853cf0032b92814dee2bf5ca&)
3. Select **Turtlesim_promax.perspective**
   ![turtlesim_promax_perc](https://cdn.discordapp.com/attachments/1278095176824393891/1284628603836825610/image.png?ex=66e7531e&is=66e6019e&hm=1cee6a40e3d84ba9546b7fe88fc53ae9d9e06c959754d355936f2fb8719945af&)
4. Launch **teleop.launch.py** and find **teleop_controller** in RQT
    ```bash
   ros2 launch turtlesim_promax teleop.launch.py
   ```
   ![teleop_con](https://cdn.discordapp.com/attachments/1278095176824393891/1284629569663406150/image.png?ex=66e75404&is=66e60284&hm=8350aee16945fd30c2e3a64bd71ea81319aa23e36d6c701345f96645475bd3a4&)
5. Edit your param values and press **ENTER**
   ![param_edit](https://cdn.discordapp.com/attachments/1278095176824393891/1284629947016544266/image.png?ex=66e7545e&is=66e602de&hm=06d41c8b50530e3c4a482f5fe94387f2fef49c9f32d7163f7a53b214d70756d2&)
6. If Param edit successfully your launch terminal will notify
   ![notify](https://cdn.discordapp.com/attachments/1278095176824393891/1284630789824184421/image.png?ex=66e75527&is=66e603a7&hm=64b75e97474313bc322c564b134fefb7d20cdf24a4408b24c9e9959d95a162da&)

## **Contributor of Turtlesim Pro Max**
-   **Vasayos Tosiri**       65340500051
-   **Poppeth Pethchamli**   65340500041
  
## **About of Turtlesim Pro Max**
This project is one of **FRA501 Robotics Dev** Exam.
