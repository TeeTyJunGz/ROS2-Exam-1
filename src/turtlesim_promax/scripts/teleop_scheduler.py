#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

class Scheduler(Node):
    def __init__(self):
        super().__init__('scheduler_node')
        self.pizza_ready = False  # Initialize default state
        self.clear_ready = False  # Initialize default state
        self.create_subscription(Bool, '/pizzaReady', self.pizzaReady_callback, 10)
        self.create_subscription(Bool, '/clearReady', self.clearReady_callback, 10)
        self.state_publisher = self.create_publisher(String, 'state', 10)
        self.create_timer(0.01, self.timer_callback)  # Timer to call state method at 10Hz

    def clearReady_callback(self, msg: Bool):
        self.clear_ready = msg.data

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
