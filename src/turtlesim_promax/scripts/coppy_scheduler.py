#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult

class Scheduler(Node):
    def __init__(self):
        super().__init__('scheduler_node')
        self.pizza_ready = False  # Initialize default state
        self.clear_ready = False  # Initialize default state
        self.cmd_vel_x = 0.0  # Initialize cmd_vel x
        self.cmd_vel_z = 0.0  # Initialize cmd_vel z
        self.declare_parameter('copy_name', ['Foxy','Noetic','Humble','Iron'])
        self.copy_name = self.get_parameter('copy_name').get_parameter_value().string_array_value

        self.create_subscription(Bool, self.copy_name[0] + '/finished_tasks', self.finished_taks_callback_1, 10)
        self.create_subscription(Bool, self.copy_name[1] + '/finished_tasks', self.finished_taks_callback_2, 10)
        self.create_subscription(Bool, self.copy_name[2] + '/finished_tasks', self.finished_taks_callback_3, 10)
        self.create_subscription(Bool, self.copy_name[3] + '/finished_tasks', self.finished_taks_callback_4, 10)

        self.state_publisher = self.create_publisher(String, 'state', 10)
        self.pizza_ready_publisher = self.create_publisher(String, 'PizzaReady', 10)
        self.save_ready_publisher = self.create_publisher(String, 'SavedReady', 10)
        self.go_right_publisher = self.create_publisher(Bool, 'go_right', 10)
        
        self.add_on_set_parameters_callback(self.set_param_callback)
        
        self.create_timer(0.01, self.timer_callback)  # Timer to call state method at 10Hz
        self.finish_1 = False
        self.finish_2 = False
        self.finish_3 = False
        self.finish_4 = False
        self.go_right1 = False
        self.go_right2 = False
        self.go_right3 = False
        self.go_right4 = False

    def set_param_callback(self, params):
        for param in params:
            if param.name == 'copy_name':
                self.get_logger().info(f'Updated copy name: {param.value}')
                self.copy_name = param.value
            else:
                self.get_logger().warn(f'Unknown parameter: {param.name}')
                return SetParametersResult(successful=False, reason=f'Unknown parameter: {param.name}')
        return SetParametersResult(successful=True)
    
    def finished_taks_callback_1(self, msg: Bool):
        self.finish_1 = msg.data
        if msg.data:
            self.go_right1 = True

    def finished_taks_callback_2(self, msg: Bool):
        self.finish_2 = msg.data
        if msg.data:
            self.go_right2 = True

    def finished_taks_callback_3(self, msg: Bool):
        self.finish_3 = msg.data
        if msg.data:
            self.go_right3 = True

    def finished_taks_callback_4(self, msg: Bool):
        self.finish_4 = msg.data  # Corrected finish_4
        if msg.data:
            self.go_right4 = True
    
    def check(self):
        if self.go_right1 and self.go_right2 and self.go_right3 and self.go_right4:
            msg = Bool()
            msg.data = True
            self.go_right_publisher.publish(msg)

    def timer_callback(self):
        self.check()
