import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial
import time

class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        # ---- ROS Subscriber ----
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'joint_angles',
            self.listener_callback,
            10)

        # ---- Serial Setup ----
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.get_logger().info("Connected to Arduino at /dev/ttyUSB0")
        except serial.SerialException:
            self.get_logger().error("Failed to connect to Arduino. Check USB connection!")
            self.ser = None

        # ---- State ----
        self.last_angles = None      # Last received joint angles
        self.motion_active = False   # Prevent overlapping motions

    def listener_callback(self, msg):
        new_angles = list(msg.data)

        # Ignore empty data
        if not new_angles:
            return

        # If same as before, ignore (prevents spamming Arduino)
        if self.last_angles == new_angles:
            return

        self.last_angles = new_angles
        self.execute_trajectory(new_angles)

    def execute_trajectory(self, joint_angles):
        if self.motion_active:
            return  # wait until current motion is finished

        self.motion_active = True
        self.get_logger().info(f"Starting trajectory: {joint_angles}")

        # --- Generate cubic trajectory ---
        # Here you'd generate via-points. For now we just send final angles.
        # If you have via points, loop through them here.
        for angle in joint_angles:
            self.send_to_arduino(angle)
            time.sleep(0.05)  # allow Arduino to process

        self.get_logger().info("Trajectory complete. Waiting for next coordinates...")
        self.motion_active = False

    def send_to_arduino(self, angle):
        if self.ser is None:
            self.get_logger().warn(f"[NO SERIAL] Would send: {angle}")
            return

        # Format your message (adapt to your Arduino code)
        message = f"{angle}\n"
        self.ser.write(message.encode('utf-8'))
        self.get_logger().info(f"Sent: {message.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down trajectory node...")
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
