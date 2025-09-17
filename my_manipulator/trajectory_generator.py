#!/home/samiul/Thesis_ws/tvm/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time
import sys

# ------------------ ROS 2 Node ------------------ #
class TrajectoryExecutorNode(Node):
    def __init__(self):
        super().__init__('trajectory_executor_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'joint_angles',
            self.joint_callback,
            10
        )
        self.get_logger().info("Trajectory Executor Node Initialized!")

        # Serial setup
        try:
            self.arduino_data = serial.Serial('/dev/ttyACM0', 9600)
            time.sleep(2)
        except Exception as e:
            self.get_logger().error(f"Serial Error: {e}")
            sys.exit(1)

        # Flag to avoid re-running sequence
        self.sequence_running = False

    # ------------------ Joint Callback ------------------ #
    def joint_callback(self, msg):
        if self.sequence_running:
            return  # Ignore new values while sequence is running

        self.sequence_running = True
        joint_angles = msg.data

        # IKPy adds extra links at first and last -> ignore
        real_joints = joint_angles[1:-1]
        self.get_logger().info(f"Received joint angles: {real_joints}")

        # Run the full manipulator sequence
        self.execute_full_sequence(real_joints)

    # ------------------ Trajectory Generators ------------------ #
    def cubic_polynomial_trajectory_generator(self, θi, θf, t, n):
        a0 = θi
        a1 = 0
        a2 = 3*(θf-θi)/(t**2)
        a3 = -2*(θf-θi)/(t**3)
        point_list = [round(θi,3)]  # start exactly at θi
        for i in range(1, n+1):
            step = i*t/n
            point_list.append(round(a0 + a1*step + a2*step**2 + a3*step**3, 3))
        return point_list

    def end_eff_operation(self, θi, θf, n):
        actuation = abs(θi-θf)
        increment = actuation/n
        points = [round(θi,3)]
        if θi < θf:
            step = θi
            for _ in range(1, n+1):
                step += increment
                points.append(round(step,3))
        else:
            step = θi
            for _ in range(1, n+1):
                step -= increment
                points.append(round(step,3))
        return points

    # ------------------ Serial Transmission ------------------ #
    def serial_transmit(self, angle_set):
        # Prevent duplicate sends
        if not hasattr(self, 'last_sent') or self.last_sent != angle_set:
            self.arduino_data.write(f"{','.join(map(str, angle_set))}\n".encode())
            self.last_sent = angle_set
            self.get_logger().info(f"Sending: {angle_set}")
            time.sleep(0.1)
            while True:
                response = self.arduino_data.readline().decode().strip()
                if response == "ACK":
                    self.get_logger().info("Arduino acknowledged")
                    break
                elif response == "":
                    self.get_logger().warn("No acknowledgement received")
                    sys.exit(1)

    # ------------------ Trajectory Executor ------------------ #
    def execute_trajectory(self, trajectory_path):
        for angle_set in trajectory_path:
            self.serial_transmit(angle_set)

    # ------------------ Full Sequence ------------------ #
    def execute_full_sequence(self, joints):
        # joints = [joint1, joint2, joint3] from topic
        j1, j2, j3 = joints
        end_eff_open = 70
        end_eff_close = 110
        via_point_j1, via_point_j2, via_point_j3 = 150, 60, 50
        delivery_j1, delivery_j2, delivery_j3 = 150, 20, 20
        t, n = 5, 40

        # --- Operation 1: move to target --- #
        joint1_path_op1 = self.cubic_polynomial_trajectory_generator(j1, via_point_j1, t, n)
        joint2_path_op1 = self.cubic_polynomial_trajectory_generator(j2, via_point_j2, t, n)
        joint3_path_op1 = self.cubic_polynomial_trajectory_generator(j3, via_point_j3, t, n)
        end_eff_opening = self.end_eff_operation(end_eff_close, end_eff_open, n)
        trajectory_op1 = list(zip(joint1_path_op1, joint2_path_op1, joint3_path_op1, end_eff_opening))
        self.execute_trajectory(trajectory_op1)

        # --- Operation 2: close end effector --- #
        end_eff_closing = self.end_eff_operation(end_eff_open, end_eff_close, n)
        self.execute_trajectory([(0,0,0,angle) for angle in end_eff_closing])  # only end effector moves

        # --- Operation 3: move to delivery via point --- #
        joint1_path_op3 = self.cubic_polynomial_trajectory_generator(via_point_j1, delivery_j1, t, n)
        joint2_path_op3 = self.cubic_polynomial_trajectory_generator(via_point_j2, delivery_j2, t, n)
        joint3_path_op3 = self.cubic_polynomial_trajectory_generator(via_point_j3, delivery_j3, t, n)
        trajectory_op3 = list(zip(joint1_path_op3, joint2_path_op3, joint3_path_op3))
        self.execute_trajectory(trajectory_op3)

        # --- Operation 4: open end effector to drop object --- #
        end_eff_opening_op4 = self.end_eff_operation(end_eff_close, end_eff_open, n)
        self.execute_trajectory([(0,0,0,angle) for angle in end_eff_opening_op4])

        # --- Operation 5: return to initial positions --- #
        joint1_path_op5 = self.cubic_polynomial_trajectory_generator(delivery_j1, j1, t, n)
        joint2_path_op5 = self.cubic_polynomial_trajectory_generator(delivery_j2, j2, t, n)
        joint3_path_op5 = self.cubic_polynomial_trajectory_generator(delivery_j3, j3, t, n)
        end_eff_closing_op5 = self.end_eff_operation(end_eff_open, end_eff_close, n)
        trajectory_op5 = list(zip(joint1_path_op5, joint2_path_op5, joint3_path_op5, end_eff_closing_op5))
        self.execute_trajectory(trajectory_op5)

        # --- 4-second rest after full sequence --- #
        self.get_logger().info("Sequence complete. Resting for 4 seconds...")
        time.sleep(4)
        self.sequence_running = False

        self.get_logger().info("Ready for new joint angles.")

    def destroy_node_safe(self):
        self.arduino_data.close()
        self.destroy_node()

# ------------------ Main ------------------ #
def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node_safe()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
