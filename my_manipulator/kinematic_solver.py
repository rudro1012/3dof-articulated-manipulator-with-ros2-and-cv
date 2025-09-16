#!/home/samiul/Thesis_ws/tvm/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import numpy as np

def joint_description(joint_name:str, alpha:float, a:float, d:float, theta:float, lower_limit:float, higher_limit:float):
    for var_value in [alpha,a,d,theta,lower_limit,higher_limit]:
        if not isinstance(var_value,float):
            raise TypeError(f'{var_value} have to be a float')
    return URDFLink(
        name=joint_name,
        origin_translation=[a,0,d],
        origin_orientation=[alpha,0,theta],
        rotation=[0,0,1],
        bounds=[lower_limit,higher_limit]
    )

class KinematicSolver(Node):
    def __init__(self):
        super().__init__('kinematic_solver')

        # Subscriber to object positions (just triggers calculation)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'object_position',
            self.object_callback,
            10
        )

        # Publisher for joint angles
        self.publisher_ = self.create_publisher(Float32MultiArray, 'joint_angles', 10)

        # Define the manipulator chain
        joint1 = joint_description('joint1', 0.0, 0.0, 0.0, 0.0, 0.0, np.pi)
        joint2 = joint_description('joint2', -np.pi/2, 1.1, 0.0, 0.0, 0.0, np.pi)
        joint3 = joint_description('joint3', np.pi, 10.4, 0.0, 0.0, 0.0, np.pi)
        end_eff = joint_description('end_eff', 0.0, 13.4, 0.0, 0.0, 0.0, np.pi)

        self.chain = Chain(
            name='my_manipulator',
            links=[
                OriginLink(),
                joint1,
                joint2,
                joint3,
                end_eff
            ]
        )

        self.get_logger().info("IK Node Initialized. Waiting for trigger messages.")

    def object_callback(self, msg):
        # Whenever any message is received, calculate IK for fixed position
        desired_position = [15.0, 11.0, 3.0]  # fixed position
        joint_angles_rad = self.chain.inverse_kinematics(desired_position)
        joint_angles_deg = [angle * 180/np.pi for angle in joint_angles_rad]

        # Publish joint angles
        angles_msg = Float32MultiArray()
        angles_msg.data = joint_angles_deg
        self.publisher_.publish(angles_msg)

        self.get_logger().info(f"Triggered IK Calculation: {joint_angles_deg}")

def main(args=None):
    rclpy.init(args=args)
    node = KinematicSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down IK Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
