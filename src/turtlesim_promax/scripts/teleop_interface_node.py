#!/usr/bin/env python3

import sys
import tty
import time
import rclpy
import termios
import threading

from rclpy.node import Node
from geometry_msgs.msg import Twist

msg = """
Control Your Turtlesim Pro Max
---------------------------
Moving around:
        w
    a   s   d

p       : spawn pizza
l       : save path
o       : clear

CTRL-C to quit
"""

# Key mappings

# Key mappings
move_bindings = {
    'w': (1.0, 0.0),  # Move forward
    's': (-1.0, 0.0),  # Move backward
    'a': (0.0, 1.0),  # Turn left
    'd': (0.0, -1.0),  # Turn right
    ' ': (0.0, 0.0)   # Stop
}


class KeyboardControl(Node):

    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speed = 4.5  # Linear speed (m/s)
        self.turn = 1.0  # Angular speed (rad/s)
        self.timeout_duration = 0.6  # Timeout for stop message (seconds)
        self.last_key_time = time.time()  # Time of the last key press
        print(msg)        
        
        # Create a separate thread to listen for key input
        self.key_listener_thread = threading.Thread(target=self.run_key_listener)
        self.key_listener_thread.daemon = True
        self.key_listener_thread.start()

        # Timer to check if the timeout has been exceeded
        self.create_timer(0.1, self.check_for_timeout)

    def get_key(self):
        """ Get keyboard input """
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run_key_listener(self):
        try:
            while True:
                key = self.get_key()
                if key in move_bindings:
                    x = move_bindings[key][0] * self.speed
                    z = move_bindings[key][1] * self.turn
                    self.last_key_time = time.time()  # Update last key press time
                else:
                    x = 0.0
                    z = 0.0
                    if key == '\x03':  # Ctrl+C to stop the node
                        break

                twist = Twist()
                twist.linear.x = x
                twist.angular.z = z
                self.publisher_.publish(twist)
                self.get_logger().info(f'Publishing: linear.x = {x}, angular.z = {z}')

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

        finally:
            self.stop_robot()

    def check_for_timeout(self):
        """ Check if the time since the last key press exceeds the timeout duration """
        if time.time() - self.last_key_time > self.timeout_duration:
            self.stop_robot()

    def stop_robot(self):
        """ Publish stop message to stop the robot """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)


def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = KeyboardControl()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

