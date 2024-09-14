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

        self.create_subscription(Bool, '/pizzaReady', self.pizzaReady_callback, 10)
        self.create_subscription(Bool, '/savedReady', self.saveReady_callback, 10)
        self.create_subscription(Bool, '/clearReady', self.clearReady_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Bool,self.copy_name[0] + '/finished_tasks', self.finished_taks_callback_1, 10)
        self.create_subscription(Bool,self.copy_name[1] + '/finished_tasks', self.finished_taks_callback_2, 10)
        self.create_subscription(Bool,self.copy_name[2] + '/finished_tasks', self.finished_taks_callback_3, 10)
        self.create_subscription(Bool,self.copy_name[3] + '/finished_tasks', self.finished_taks_callback_4, 10)


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
                # Return failure result for unknown parameters
                return SetParametersResult(successful=False, reason=f'Unknown parameter: {param.name}')
        # If all parameters are known, return success
        return SetParametersResult(successful=True)
    
    def finished_taks_callback_1(self, msg: Bool):
        self.finish_1 = msg.data
        if msg.data == True:
            self.go_right1 = True

    def finished_taks_callback_2(self, msg: Bool):
        self.finish_2 = msg.data
        if msg.data == True:
            self.go_right2 = True

    def finished_taks_callback_3(self, msg: Bool):
        self.finish_3 = msg.data
        if msg.data == True:
            self.go_right3 = True

    def finished_taks_callback_4(self, msg: Bool):
        self.finish_3 = msg.data
        if msg.data == True:
            self.go_right4 = True
    
    def check(self, msg : Bool):
        if self.go_right1 == True and self.go_right2 == True and self.go_right3 == True and self.go_right4 == True :
            msg = Bool
            msg.data = True
            self.go_right_publisher.publish(msg)
    
    def pizzaReady_callback(self, msg: Bool):
        self.pizza_ready = msg.data
        string_msg = String()
        string_msg.data = 'pizzaReady'
        self.pizza_ready_publisher.publish(string_msg)
        
    def clearReady_callback(self, msg: Bool):
        self.clear_ready = msg.data
    
    def saveReady_callback(self, msg: Bool):
        self.save_ready = msg.data
        string_msg = String()
        string_msg.data = 'savedReady'
        self.save_ready_publisher.publish(string_msg)

    def cmd_vel_callback(self, msg: Twist):
        # Assign values from the Twist message
        self.cmd_vel_x = msg.angular.x
        self.cmd_vel_z = msg.angular.z

    def state(self):
        msg = String()  # Correct instantiation of String message
        if self.clear_ready:
            msg.data = 'clear'
        else:
            msg.data = 'teleop'
        self.state_publisher.publish(msg)
        # self.get_logger().info(f'Published state: {msg.data}')

    def timer_callback(self):
        self.state()

def main(args=None):
    rclpy.init(args=args)
    node = Scheduler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
