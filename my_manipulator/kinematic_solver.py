#!/home/samiul/Thesis_ws/tvm/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import numpy as np

# Function to declare a joint
def joint_description(joint_name: str, alpha: float, a: float, d: float, theta: float,
                      lower_limit: float, higher_limit: float):
    for var_value in [alpha, a, d, theta, lower_limit, higher_limit]:
        if not isinstance(var_value, float):
            raise TypeError(f'{var_value} has to be a float')
    return URDFLink(
        name=joint_name,
        origin_translation=[a, 0, d],
        origin_orientation=[alpha, 0, theta],
        rotation=[0, 0, 1],
        bounds=[lower_limit, higher_limit]
    )

class KinematicSolver(Node):
    def __init__(self):
        super().__init__('kinematic_solver')

        # Subscriber to receive object positions
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'object_position',  # topic from object detection node
            self.object_callback,
            10
        )

        # Publisher for joint angles
        self.publisher_ = self.create_publisher(Float32MultiArray, 'joint_angles', 10)

        # Create manipulator chain
        joint1 = joint_description('joint1', 0.0, 0.0, 0.0, 0.0, 0.0, np.pi)
        joint2 = joint_description('joint2', -np.pi/2, 1.1, 0.0, 0.0, 0.0, np.pi)
        joint3 = joint_description('joint3', np.pi, 10.4, 0.0, 0.0, 0.0, np.pi)
        end_eff = joint_description('end_eff', 0.0, 13.4, 0.0, 0.0, 0.0, np.pi)

        self.chain = Chain(
            name='my_manipulator',
            links=[OriginLink(), joint1, joint2, joint3, end_eff]
        )

        self.get_logger().info("KinematicSolver node initialized and ready for coordinates.")

    def object_callback(self, msg):
        # Check if message has enough data
        if len(msg.data) < 2:
            self.get_logger().warn("Received object_position message with insufficient data")
            return

        # Read x and y from message
        x, y = msg.data[0], msg.data[1]

        # Fixed z-coordinate
        z = 3.0

        self.get_logger().info(f"New object coordinate received: x={x}, y={y}, z={z}")

        # Compute joint angles using IK
        desired_position = [x, y, z]
        try:
            joint_angles_rad = self.chain.inverse_kinematics(desired_position)
        except Exception as e:
            self.get_logger().error(f"IK computation failed: {e}")
            return

        # Convert joint angles from radians to degrees
        joint_angles_deg = [round(angle * 180 / np.pi, 3) for angle in joint_angles_rad]

        # Publish joint angles
        angles_msg = Float32MultiArray()
        angles_msg.data = joint_angles_deg
        self.publisher_.publish(angles_msg)

        self.get_logger().info(f"Published joint angles (deg): {joint_angles_deg}")

def main(args=None):
    rclpy.init(args=args)
    node = KinematicSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down KinematicSolver node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
