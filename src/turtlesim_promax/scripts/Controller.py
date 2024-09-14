#!/usr/bin/python3
import rclpy
from rclpy.node import Node


class SpawnPizza(Node):
    def __init__(self):
        super().__init__('SpawnPizza_node')

def main(args=None):
    rclpy.init(args=args)
    node = SpawnPizza()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
