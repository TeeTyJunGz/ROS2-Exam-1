#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist

class Scheduler(Node):
    def __init__(self):
        super().__init__('scheduler_node')
        self.pizza_ready = False  # Initialize default state
        self.clear_ready = False  # Initialize default state
        self.cmd_vel_x = 0.0  # Initialize cmd_vel x
        self.cmd_vel_z = 0.0  # Initialize cmd_vel z

        self.create_subscription(Bool, '/pizzaReady', self.pizzaReady_callback, 10)
        self.create_subscription(Bool, '/savedReady', self.saveReady_callback, 10)
        self.create_subscription(Bool, '/clearReady', self.clearReady_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.state_publisher = self.create_publisher(String, 'state', 10)
        self.pizza_ready_publisher = self.create_publisher(String, 'PizzaReady', 10)
        self.save_ready_publisher = self.create_publisher(String, 'SavedReady', 10)
        self.create_timer(0.01, self.timer_callback)  # Timer to call state method at 10Hz


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
