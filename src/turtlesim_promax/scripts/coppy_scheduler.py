#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rcl_interfaces.msg import SetParametersResult

class Copy_Scheduler(Node):
    def __init__(self):
        super().__init__('copy_scheduler_node')
        self.declare_parameter('copy_name', ['Foxy','Noetic','Humble','Iron'])
        self.copy_name = self.get_parameter('copy_name').get_parameter_value().string_array_value

        self.create_subscription(Bool, '/' + self.copy_name[0] + '/finished_tasks', self.finished_taks_callback_1, 10)
        self.create_subscription(Bool, '/' + self.copy_name[1] + '/finished_tasks', self.finished_taks_callback_2, 10)
        self.create_subscription(Bool, '/' + self.copy_name[2] + '/finished_tasks', self.finished_taks_callback_3, 10)
        self.create_subscription(Bool, '/' + self.copy_name[3] + '/finished_tasks', self.finished_taks_callback_4, 10)

        self.go_right_publisher = self.create_publisher(Bool, '/go_right', 10)
        
        self.add_on_set_parameters_callback(self.set_param_callback)
        
        self.create_timer(0.01, self.timer_callback)  # Timer to call state method at 10Hz
        self.finish_1 = False
        self.finish_2 = False
        self.finish_3 = False
        self.finish_4 = False
        self.countT = 0

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

    def finished_taks_callback_2(self, msg: Bool):
        self.finish_2 = msg.data

    def finished_taks_callback_3(self, msg: Bool):
        self.finish_3 = msg.data

    def finished_taks_callback_4(self, msg: Bool):
        self.finish_4 = msg.data
            
    def timer_callback(self):
        
        if self.finish_1 and self.finish_2 and self.finish_3 and self.finish_4:
            self.countT += 1
            print(self.countT)
        
        if self.countT == 1:
            print("In")
            msg = Bool()
            msg.data = True
            self.go_right_publisher.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = Copy_Scheduler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()