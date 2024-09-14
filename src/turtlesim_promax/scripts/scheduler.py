#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64,Bool,String

class Scheduler(Node):
    def __init__(self):
        super().__init__('scheduler_node')
        self.create_subscription(Bool, '/pizzaReady', self.pizzaReady_callback, 10)
        self.create_subscription(Bool, '/clearReady', self.clearReady_callback, 10)
        self.state_publisher = self.create_publisher(String, '/state', 10)
        self.create_timer(1.0 / self.freq, self.timer_callback)


    def pizzaReady_callback(self,msg : Bool):
        self.pizza_ready = msg.data

    def clearReady_callback(self,msg : Bool):
        self.clear_ready = msg.data

    def state(self):
        msg = String
        if self.clear_ready == True:
             msg.data = 'clear'
        else:
            msg.data = 'teleop'
        self.state_publisher.publish(msg)

    def timer_callback(self):
            self.target()


    


def main(args=None):
    rclpy.init(args=args)
    node = Scheduler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
