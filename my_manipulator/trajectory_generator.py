#!/home/samiul/Thesis_ws/tvm/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time
import sys

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')

        # Subscriber to receive joint angles from IK node
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'joint_angles',
            self.joint_callback,
            10
        )

        # Publisher for executed joint angles (optional)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'executed_joint_angles', 10)

        # Serial connection to Arduino
        try:
            self.arduino_data = serial.Serial('/dev/ttyACM0', 9600)
            time.sleep(2)
            self.get_logger().info("Arduino serial connected")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            sys.exit(1)

        self.busy = False  # flag to prevent handling new messages while executing

    # Cubic polynomial trajectory generator
    def cubic_polynomial_trajectory_generator(self, θi, θf, t, n):
        a0 = θi
        a1 = 0
        a2 = 3*(θf-θi)/(t**2)
        a3 = -2*(θf-θi)/(t**3)
        point_list = []
        for i in range(1, n+1):
            step = i*t/n
            step_inc = a0 + a1*step + a2*step**2 + a3*step**3
            point_list.append(round(step_inc,3))
        return point_list

    # End-effector trajectory
    def end_eff_operation(self, θi, θf, n):
        increment = (θf-θi)/n
        actuation_points = []
        for i in range(n):
            step = θi + increment*(i+1)
            actuation_points.append(round(step,3))
        return actuation_points

    # Send data to Arduino
    def serial_transmit(self, angle_data):
        self.arduino_data.write(f"{angle_data}\n".encode())

    # Execute a full trajectory of joint tuples
    def trajectory_executor(self, trajectory_path):
        for angle_set in trajectory_path:
            self.serial_transmit(",".join(map(str, angle_set)))
            self.get_logger().info(f"Sending: {angle_set}")
            time.sleep(0.1)
            while True:
                response = self.arduino_data.readline().decode().strip()
                if response == "ACK":
                    break
                elif response == "":
                    self.get_logger().error("No acknowledgement received")
                    sys.exit(1)
            # Optional: publish executed angles
            msg = Float32MultiArray()
            msg.data = list(angle_set)
            self.publisher_.publish(msg)

    # Execute end-effector only
    def end_eff_execution(self, trajectory):
        for angle in trajectory:
            self.serial_transmit(angle)
            self.get_logger().info(f"Sending End-Effector: {angle}")
            time.sleep(0.1)
            while True:
                response = self.arduino_data.readline().decode().strip()
                if response == "ACK":
                    break
                elif response == "":
                    self.get_logger().error("No acknowledgement received")
                    sys.exit(1)

    # Event-based callback for new joint angles
    def joint_callback(self, msg):
        if self.busy:
            self.get_logger().info("Trajectory execution in progress, ignoring new message")
            return

        self.busy = True
        self.get_logger().info("New joint angles received, executing trajectory")

        # Assume msg.data = [joint1, joint2, joint3, end_effector]
        target_joints = msg.data
        initial_joints = [90, 90, 90, 90]  # modify if needed

        # Operation 1: move to pick position
        joint1_path = self.cubic_polynomial_trajectory_generator(initial_joints[0], target_joints[0], 5, 40)
        joint2_path = self.cubic_polynomial_trajectory_generator(initial_joints[1], target_joints[1], 5, 40)
        joint3_path = self.cubic_polynomial_trajectory_generator(initial_joints[2], target_joints[2], 5, 40)
        end_eff_path = self.end_eff_operation(initial_joints[3], target_joints[3], 40)
        trajectory_path = list(zip(joint1_path, joint2_path, joint3_path, end_eff_path))
        self.trajectory_executor(trajectory_path)

        # Pause 2 seconds at the target
        self.get_logger().info("Trajectory completed, waiting 2 seconds")
        time.sleep(2)

        self.busy = False
        self.get_logger().info("Ready for next joint angles")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TrajectoryGenerator node...")
    finally:
        node.arduino_data.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
