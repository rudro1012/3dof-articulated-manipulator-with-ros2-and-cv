#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class tester(Node):
    
    def __init__(self):
        super().__init__("tester")
        self.create_timer(0.1,self.timer_callback)
        self.publisher=self.create_publisher(Float32MultiArray,'angles',10)
    
    def timer_callback(self):
        self.get_logger().info('hello ros2')
        amsg=Float32MultiArray()
        amsg.data=[float(17.54),float(97.53),float(56.18)]
        self.publisher.publish(amsg)

def main(args=None):
    rclpy.init(args=args)
    node=tester()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=='__main__':
    main()