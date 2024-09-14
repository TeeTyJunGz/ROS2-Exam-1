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

        self.state_publisher = self.create_publisher(String, '/teleop' + self.get_namespace() + '/state', 10)
        self.pizza_ready_publisher = self.create_publisher(Bool, '/teleop' + self.get_namespace() + '/pizzaReady', 10)
        self.save_ready_publisher = self.create_publisher(Bool, '/teleop' + self.get_namespace() + '/savedReady', 10)
        self.create_timer(0.01, self.timer_callback)  # Timer to call state method at 10Hz


    def pizzaReady_callback(self, msg: Bool):
        # self.pizza_ready = msg.data
        bool_msg = Bool()
        bool_msg.data = msg.data
        self.pizza_ready_publisher.publish(bool_msg)
        
    def clearReady_callback(self, msg: Bool):
        self.clear_ready = msg.data
    
    def saveReady_callback(self, msg: Bool):
        # self.save_ready = msg.data
        bool_msg = Bool()
        bool_msg.data = msg.data
        self.save_ready_publisher.publish(bool_msg)

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
