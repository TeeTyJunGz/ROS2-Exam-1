#!/usr/bin/python3
import math
import yaml
import time
import rclpy
import numpy as np

from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Bool

from turtlesim.srv import Spawn
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty


class copy_controller(Node):
    def __init__(self):
        super().__init__('Copy_Controller_Node')
                
        self.turtle_client = self.create_client(Spawn, '/copy/spawn_turtle')
        self.pizza_client = self.create_client(GivePosition, '/copy/spawn_pizza')
        
        self.declare_parameter('Kp', 2.5)

        self.create_timer(0.01, self.timer_callback)

        self.turtle_pose = np.array([0.0, 0.0, 0.0]) #x, y, theta
        self.pizza_count = 0
        self.turtle_count = 0
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.finished_pub = self.create_publisher(Bool, 'finished_tasks', 10)
        
        self.go_right = self.create_subscription(Bool, '/go_right', self.go_right_callback, 10)
        self.pose_sub = self.create_subscription(Pose, 'pose', self.turtle_callback, 10)
        self.tt_sub = self.create_subscription(Bool, '/toggle_turtle', self.toggle_callback, 10)

        self.eaten_client = self.create_client(Empty, 'eat')
        
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value

        self.toggle = False
        self.state = 0
        self.index = 0
        self.target = []
        self.last_tasks = False
        self.last_state = 0
        
        self.add_on_set_parameters_callback(self.set_param_callback)

    def set_param_callback(self, params):
        for param in params:
            if param.name == 'Kp':
                self.get_logger().info(f'Updated Kp: {param.value}')
                self.Kp = param.value
            else:
                self.get_logger().warn(f'Unknown parameter: {param.name}')
                # Return failure result for unknown parameters
                return SetParametersResult(successful=False, reason=f'Unknown parameter: {param.name}')
        # If all parameters are known, return success
        return SetParametersResult(successful=True) 
            
    def loadYAML(self):
        with open('pizza_position/pizza_point.yaml', 'r') as file:
            data = yaml.safe_load(file)
            return data['pizza_position_' + self.get_namespace()[1:]]
        
    def cmd_vel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)
        
    def go_right_callback(self, msg: Bool):
        self.last_tasks = msg.data
        
    def turtle_callback(self, msg):
        self.turtle_pose[0] = msg.x
        self.turtle_pose[1] = msg.y
        self.turtle_pose[2] = msg.theta
                
    def toggle_callback(self, msg: Bool):
        self.toggle = msg.data
                
    def turtle_spawn(self):
        pos_req = Spawn.Request()
        pos_req.x = -5.0
        pos_req.y = -5.0
        pos_req.theta = 0.0
        pos_req.name = str(self.get_namespace())
        
        while not self.turtle_client.wait_for_service(2):
            continue
        
        if self.turtle_client.service_is_ready():
            self.turtle_client.call_async(pos_req)
            self.turtle_count += 1
            
    def give_pizza(self, pos):
        pos_req = GivePosition.Request()
        pos_req.x = pos[0]
        pos_req.y = pos[1]
        while not self.pizza_client.wait_for_service(2):
            continue
        
        time.sleep(0.1)
        
        if self.pizza_client.service_is_ready():
            self.pizza_client.call_async(pos_req)
            self.pizza_count += 1

    def eat_pizza(self):
        self.eaten_client.call_async(Empty.Request())
        self.pizza_count -= 1
        
    def timer_callback(self):

        if self.toggle:
            
            if self.turtle_count == 0:
                self.turtle_spawn()
                self.target = self.loadYAML()
                finish = Bool()
                finish.data = False
                self.finished_pub.publish(finish)
                
            
            if len(self.target) > 0 and self.state == 0:

                x = self.target[self.index][0]
                y = self.target[self.index][1]
                
                dx = x - self.turtle_pose[0]
                dy = y - self.turtle_pose[1]
                
                d = math.sqrt(pow(dx, 2) + pow(dy, 2))
                
                mouse_angle = math.atan2(dy, dx)
                turtle_angle = self.turtle_pose[2]
                angular_diff = mouse_angle - turtle_angle
                dta = math.atan2(math.sin(angular_diff), math.cos(angular_diff))
                flag = 1

                gdw = 0.1 * math.degrees(dta)
                gd = self.Kp * d
                
                self.cmd_vel(gd,gdw)
            
                if abs(d) < 0.01 and flag == 1:

                    gd = 0.0
                    pos = [0.0, 0.0]
                    pos[0] = self.turtle_pose[0]
                    pos[1] = self.turtle_pose[1]
                    self.give_pizza(pos)
                        
                    flag = 0
                    if self.index < len(self.target) - 1:
                        self.index += 1
                        
                    elif self.index == len(self.target) - 1:
                        self.target = []
                                                       
                        self.state = 1
                        self.last_state = 1
                        self.get_logger().info(f'clear target')
                        finish = Bool()
                        finish.data = True
                        self.finished_pub.publish(finish)
                
            else:
                self.target = []

                if self.last_tasks and self.last_state == 1:
                    x = 9.5
                    y = 9.5
                    
                    dx = x - self.turtle_pose[0]
                    dy = y - self.turtle_pose[1]
                    
                    d = math.sqrt(pow(dx, 2) + pow(dy, 2))
                    
                    mouse_angle = math.atan2(dy, dx)
                    turtle_angle = self.turtle_pose[2]
                    angular_diff = mouse_angle - turtle_angle
                    dta = math.atan2(math.sin(angular_diff), math.cos(angular_diff))
                    flag = 1

                    gdw = 0.1 * math.degrees(dta)
                    gd = self.Kp * d
                    
                    self.cmd_vel(gd,gdw)
                    
                    if abs(d) < 0.01:
                        self.last_state = 2
                
                if self.last_state == 2:
                    t = 0.0
                    if self.turtle_pose[2] != t:
                        dw = math.atan2(math.sin(0.0 - self.turtle_pose[2]), math.cos(0.0 - self.turtle_pose[2]))
                        dta = 2.5 * dw
                        self.cmd_vel(0.0, dta)
                    
                    self.index = 0
                    self.state = 0
        

def main(args=None):
    rclpy.init(args=args)
    node = copy_controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
