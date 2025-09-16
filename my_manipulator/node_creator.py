#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class node_creator(Node):
    
    def __init__(self):
        super().__init__("node_creator")
        self.create_timer(0.1,self.timer_callback)
    
    def timer_callback(self):
        self.get_logger().info('hello from ros2')

def main(args=None):
    rclpy.init(args=args)
    node=node_creator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=='__main__':
    main()