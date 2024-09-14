#!/usr/bin/python3
import math
import yaml
import rclpy
import numpy as np

from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Twist, Point, PoseStamped
from turtlesim.msg import Pose
from std_msgs.msg import String, Bool

from turtlesim.srv import Spawn, Kill
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty


class controller(Node):
    def __init__(self):
        super().__init__('Controller_Node')
               
        self.turtle_client = self.create_client(Spawn, '/teleop/spawn_turtle')
        self.pizza_client = self.create_client(GivePosition, '/teleop/spawn_pizza')
        self.kill_client = self.create_client(Kill, '/teleop/remove_turtle')

        self.declare_parameter('pizza_max', 20)
        self.declare_parameter('Kp', 1.5)
        self.declare_parameter('copy_name', ['Foxy','Noetic','Humble','Iron'])
        
        self.create_timer(0.01, self.timer_callback)

        self.turtle_pose = np.array([0.0, 0.0, 0.0]) #x, y, theta
        self.pizza_count = 0
        self.turtle_count = 0
        self.kill_count = 0
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.toggle_pub = self.create_publisher(Bool, '/toggle_turtle', 10)
        
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.pose_sub = self.create_subscription(Pose, 'pose', self.turtle_callback, 10)
        self.state_sub = self.create_subscription(String, '/teleop' + self.get_namespace() + '/state', self.state_callback, 10)
        self.pizza_sub = self.create_subscription(Bool, '/teleop' + self.get_namespace() + '/pizzaReady', self.pizza_callback, 10)
        self.saved_sub = self.create_subscription(Bool, '/teleop' + self.get_namespace() + '/savedReady', self.saved_callback, 10)

        self.pizzaReady = False
        self.state = 'teleop'
        self.eaten_client = self.create_client(Empty, 'eat')
                
        self.cmd_rc = np.array([0.0, 0.0])
        self.pos_list = []
        self.saved_count = 1
        
        self.pizza_max = self.get_parameter('pizza_max').get_parameter_value().integer_value
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.copy_name = self.get_parameter('copy_name').get_parameter_value().string_array_value
        
        # Add callback for parameter changes
        self.add_on_set_parameters_callback(self.set_param_callback)
        self.yaml_create()

        
    def set_param_callback(self, params):
        for param in params:
            if param.name == 'pizza_max':
                self.get_logger().info(f'Updated pizza_max: {param.value}')
                self.pizza_max = param.value
            elif param.name == 'Kp':
                self.get_logger().info(f'Updated Kp: {param.value}')
                self.Kp = param.value
            elif param.name == 'copy_name':
                self.get_logger().info(f'Updated copy name: {param.value}')
                self.copy_name = param.value
            else:
                self.get_logger().warn(f'Unknown parameter: {param.name}')
                # Return failure result for unknown parameters
                return SetParametersResult(successful=False, reason=f'Unknown parameter: {param.name}')
        # If all parameters are known, return success
        return SetParametersResult(successful=True)
    
    def yaml_create(self):
        empty_data = {}  # Or [] if you want an empty list

        # Write to the YAML file
        with open('pizza_position/test.yaml', 'w') as file:
            yaml.dump(empty_data, file)
            self.pizza_list = {'pizza_position_' + str(self.copy_name[0]): [],
                               'pizza_position_' + str(self.copy_name[1]): [],
                               'pizza_position_' + str(self.copy_name[2]): [], 
                               'pizza_position_' + str(self.copy_name[3]): [], 
                              }
     
    def yaml_write(self):
        with open('pizza_position/test.yaml', 'w') as file:
            yaml.dump(self.pizza_list, file)   
        if self.saved_count == 5:
            msg = Bool()
            msg.data = True
            self.toggle_pub.publish(msg)
            
    def cmd_vel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)
        
    def turtle_callback(self, msg):
        self.turtle_pose[0] = msg.x
        self.turtle_pose[1] = msg.y
        self.turtle_pose[2] = msg.theta
        
    def cmd_vel_callback(self, msg: Twist):
        self.cmd_rc[0] = msg.linear.x
        self.cmd_rc[1] = msg.angular.z
        
    def state_callback(self, msg: String):
        self.state = msg.data
        
    def pizza_callback(self, msg: Bool):
        if msg.data:
            if self.pizza_count <= self.pizza_max:
                if self.saved_count <= 4:
                    self.give_pizza(self.turtle_pose)
                    
                    pos = [0.0, 0.0]
                    pos[0] = float(self.turtle_pose[0])
                    pos[1] = float(self.turtle_pose[1])
                    
                    self.pos_list.append(pos)
                    
                    # self.get_logger().info(f'Added: {self.pos_list}')
                    # self.get_logger().info(f'Added Pizza No.: {len(self.pos_list)}')
                    self.get_logger().info(f'Added Pizza No.: {self.pizza_count} / {self.pizza_max:}')
                    
                
                # self.get_logger().info(f'Unknown parameter: {pos}')

                
                if self.saved_count == 1:
                    self.pizza_list['pizza_position_' + str(self.copy_name[0])].append(pos)
                    
                elif self.saved_count == 2:
                    self.pizza_list['pizza_position_' + str(self.copy_name[1])].append(pos)

                elif self.saved_count == 3:
                    self.pizza_list['pizza_position_' + str(self.copy_name[2])].append(pos)

                elif self.saved_count == 4:
                    self.pizza_list['pizza_position_' + str(self.copy_name[3])].append(pos)

                else:
                    pass
        
    def saved_callback(self, msg: Bool):
        if msg.data:
            self.pos_list = []
            
            if self.saved_count == 1:
                self.get_logger().info('Saved 1st Position')
                self.saved_count += 1
                
            elif self.saved_count == 2:
                self.get_logger().info('Saved 2nd Position')
                self.saved_count += 1

            elif self.saved_count == 3:
                self.get_logger().info('Saved 3rd Position')
                self.saved_count += 1

            elif self.saved_count == 4:
                self.get_logger().info('Saved 4th Position')
                self.saved_count += 1

            else:
                self.get_logger().info('Already saved 4 times!')
                
            
            self.yaml_write()
            
        
    def turtle_spawn(self):
        pos_req = Spawn.Request()
        pos_req.x = 5.0
        pos_req.y = 5.0
        pos_req.theta = 0.0
        pos_req.name = str(self.get_namespace())
        
        while not self.turtle_client.wait_for_service(2):
            continue
        
        if self.turtle_client.service_is_ready():
            # self.get_logger().info('Hee Tao')
            self.turtle_client.call_async(pos_req)
            self.turtle_count += 1
            
    def give_pizza(self, pos):
        pos_req = GivePosition.Request()
        pos_req.x = pos[0]
        pos_req.y = pos[1]
        self.pizza_client.call_async(pos_req)
        self.pizza_count += 1

    def eat_pizza(self):
        self.eaten_client.call_async(Empty.Request())
        self.get_logger().info(f'Removed Pizza No.: {self.pizza_count} / {self.pizza_max:}')
        self.pizza_count -= 1
    
    def kill(self):
        name_turtle = Kill.Request()
        name_turtle.name = 'turtle1'
        
        while not self.kill_client.wait_for_service(2):
            continue
        
        if self.kill_client.service_is_ready():
            # self.get_logger().info('Hee Tao')
            self.kill_client.call_async(name_turtle)
            self.kill_count += 1
        
    def timer_callback(self):
        
        if self.turtle_count == 0:
            # if str(self.get_namespace()) != '/turtle1':
            #     self.kill()
                
            # if self.kill_count == 1:
            self.turtle_spawn()
        
        if self.state == 'teleop':
            
            self.cmd_vel(self.cmd_rc[0], self.cmd_rc[1])
            
        elif self.state == 'clear':
            if len(self.pos_list) > 0:
                    
                x = self.pos_list[0][0]
                y = self.pos_list[0][1]
                
                dx = x - self.turtle_pose[0]
                dy = y - self.turtle_pose[1]
                
                d = math.sqrt(pow(dx, 2) + pow(dy, 2))
                
                mouse_angle = math.atan2(dy, dx)
                turtle_angle = self.turtle_pose[2]
                angular_diff = mouse_angle - turtle_angle
                dta = math.atan2(math.sin(angular_diff), math.cos(angular_diff))
                flag = 1
                
                # self.get_logger().info(f'Clearing dw: {math.degrees(dta)}')
                # self.get_logger().info(f'Clearing d: {d}')

                gdw = 0.1 * math.degrees(dta)
                gd = self.Kp * d
                
                self.cmd_vel(gd,gdw)
            
                if abs(d) < 0.1 and flag == 1:
                    # self.get_logger().info(f'size: {len(self.pos_list)}')
                    # self.get_logger().info(f'pizza: {self.pos_list[self.index]}')

                    gd = 0.0
                    self.eat_pizza()

                    self.pos_list.pop(0)
                        
                    flag = 0
            else:
                self.cmd_vel(0.0, 0.0)

            
        
  

        

def main(args=None):
    rclpy.init(args=args)
    node = controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
