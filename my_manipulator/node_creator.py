#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class myNode(Node):
    
    def __init__(self):
        super().__init__("kinematics_node")
        self.get_logger().info('Hello')

def main(args=None):
    rclpy.init(args=args)
    node=myNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=='__main__':
    main()