#!/home/samiul/Thesis_ws/tvm/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time
import sys

class trajectory_generator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')

        # Subscriber to trigger execution (from IK node)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'joint_angles',  # topic from IK node
            self.trigger_callback,
            10
        )

        # Publisher for executed joint angles (monitoring)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'executed_joint_angles', 10)

        # Initialize serial
        try:
            self.arduino_data = serial.Serial('/dev/ttyACM0', 9600)
            time.sleep(2)
            self.get_logger().info("Arduino serial connected")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            sys.exit(1)

        # Initial positions
        self.initial_pos = [90, 90, 90, 90]   # last is end-effector
        self.end_eff_open = 90
        self.end_eff_close = 60
        self.delivery_point = [150, 60, 50]

        # Busy flag to prevent multiple triggers
        self.busy = False

    # ===== Trajectory generators =====
    def cubic_polynomial_trajectory_generator(self, θi, θf, t, n):
        a0 = θi
        a1 = 0
        a2 = 3*(θf-θi)/(t**2)
        a3 = -2*(θf-θi)/(t**3)
        return [round(a0 + a1*(i*t/n) + a2*(i*t/n)**2 + a3*(i*t/n)**3, 3) for i in range(1, n+1)]

    def end_eff_operation(self, θi, θf, n):
        increment = (θf - θi)/n
        return [round(θi + increment*(i+1),3) for i in range(n)]

    # ===== Serial transmission =====
    def serial_transmit(self, angle_data):
        self.arduino_data.write(f"{angle_data}\n".encode())

    # ===== Execution functions =====
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
            # Publish executed angles
            msg = Float32MultiArray()
            msg.data = list(angle_set)
            self.publisher_.publish(msg)

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

    # ===== Trigger callback =====
    def trigger_callback(self, msg):
        if self.busy:
            self.get_logger().warn("Trajectory in progress, ignoring new trigger")
            return

        self.busy = True
        self.get_logger().info("Trigger received: executing full trajectory")

        # --- Operation 1: move to pick ---
        joint1_path_op1 = self.cubic_polynomial_trajectory_generator(self.initial_pos[0], 29, 10, 20)
        joint2_path_op1 = self.cubic_polynomial_trajectory_generator(self.initial_pos[1], 45, 10, 20)
        joint3_path_op1 = self.cubic_polynomial_trajectory_generator(self.initial_pos[2], 120, 10, 20)
        end_eff_opening = self.end_eff_operation(self.end_eff_close, self.end_eff_open, 20)
        trajectory_path_op1 = list(zip(joint1_path_op1, joint2_path_op1, joint3_path_op1, end_eff_opening))
        self.trajectory_executor(trajectory_path_op1)

        # --- Operation 2: close end-effector ---
        end_eff_closing_op2 = self.end_eff_operation(self.end_eff_open, self.end_eff_close, 20)
        self.end_eff_execution(end_eff_closing_op2)

        # --- Operation 3: move to delivery ---
        joint1_path_op3 = self.cubic_polynomial_trajectory_generator(29, self.delivery_point[0], 10, 20)
        joint2_path_op3 = self.cubic_polynomial_trajectory_generator(45, self.delivery_point[1], 10, 20)
        joint3_path_op3 = self.cubic_polynomial_trajectory_generator(120, self.delivery_point[2], 10, 20)
        trajectory_path_op3 = list(zip(joint1_path_op3, joint2_path_op3, joint3_path_op3))
        self.trajectory_executor(trajectory_path_op3)

        # --- Operation 4: drop object ---
        end_eff_opening_op4 = self.end_eff_operation(self.end_eff_close, self.end_eff_open, 10)
        self.end_eff_execution(end_eff_opening_op4)

        # --- Operation 5: return to initial ---
        joint1_path_op5 = self.cubic_polynomial_trajectory_generator(self.delivery_point[0], self.initial_pos[0], 10, 20)
        joint2_path_op5 = self.cubic_polynomial_trajectory_generator(self.delivery_point[1], self.initial_pos[1], 10, 20)
        joint3_path_op5 = self.cubic_polynomial_trajectory_generator(self.delivery_point[2], self.initial_pos[2], 10, 20)
        end_eff_closing_op5 = self.end_eff_operation(self.end_eff_open, self.end_eff_close, 20)
        trajectory_path_op5 = list(zip(joint1_path_op5, joint2_path_op5, joint3_path_op5, end_eff_closing_op5))
        self.trajectory_executor(trajectory_path_op5)

        # --- Wait 2 seconds before accepting new trigger ---
        time.sleep(2)
        self.busy = False
        self.get_logger().info("Trajectory completed. Node ready for new trigger.")

def main(args=None):
    rclpy.init(args=args)
    node = trajectory_generator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Trajectory Node...")
    finally:
        node.arduino_data.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
