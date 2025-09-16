#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class node_creator(Node):
    
    def __init__(self):
        super().__init__("node_creator")
        self.get_logger().info('Hello')

def main(args=None):
    rclpy.init(args=args)
    node=node_creator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=='__main__':
    main()