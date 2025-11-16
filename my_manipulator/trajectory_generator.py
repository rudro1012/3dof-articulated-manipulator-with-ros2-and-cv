#!/home/samiul/Thesis_ws/tvm/bin/python3
import rclpy
from rclpy.node import Node
import numpy as np
import time
from std_msgs.msg import Float32MultiArray,Bool
import sys
import serial

class trajectory_generator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        self.joint_subscriber=self.create_subscription(Float32MultiArray,'angles',self.joint_callback,10)
        self.busy_publisher=self.create_publisher(Bool,'status',10)
        self.get_logger().info('trajectory executor initiated,hello')
        self.last_target = None

        try:
            self.arduino_data=serial.Serial('/dev/ttyACM0', 115200)
            time.sleep(2)
        except Exception as e:
            self.get_logger().info(f'serial error:{e}')
            sys.exit(1)

        self.arm_in_motion=False
    
    def joint_callback(self,angles):
        if self.arm_in_motion:
            return
        
        if self.last_target is not None:
            diff = max(abs(a - b) for a, b in zip(angles, self.last_target))
            if diff < 2.0:  # less than 2 degrees difference
                return
        
        self.last_target = angles
        self.arm_in_motion=True
        self.status_publisher(self.arm_in_motion)
        
        joint_angles=angles.data
        self.get_logger().info(f'joint angles recieved :{joint_angles}')

        try:
            self.sequence_execution(joint_angles)

        finally:
            self.arm_in_motion=False
            self.status_publisher(self.arm_in_motion)

    def status_publisher(self,state):
        msg=Bool()
        msg.data=state
        self.busy_publisher.publish(msg)
    

    def trajectory_generator(self, θi:float, θf:float, t:float, n):
        a0 = θi
        a1 = 0
        a2 = 3*(θf-θi)/(t**2)
        a3 = -2*(θf-θi)/(t**3)
        via_points=[]
        i=1
        while(i<=n):
            step=i*t/n
            actuator_position=round(a0 + a1*step + a2*step**2 + a3*step**3, 2)
            via_points.append(actuator_position)
            i=i+1

        return via_points
    

    def ee_actuation(self,θi:float,θf:float,n):
        actuation=abs(θi-θf)
        increment=actuation/n
        actuation_points=[]
        if θi<θf:
            step=θi
            i=1
            while(i<=n):
                step=step+increment
                step_round=float(round(step,2))
                actuation_points.append(step_round)
                i=i+1

        elif θi>θf:
            step=θi
            i=1
            while(i<=n):
                step=step-increment
                step_round=round(step,2)
                actuation_points.append(step_round)
                i=i+1

        return actuation_points   
    

    def trajectory_execution(self,trajectory_path):
        for angle_set in trajectory_path:
            self.serial_transmit(angle_set)
            time.sleep(0.08)
    

    def ee_motion_execution(self,angle_set):
        for angle in angle_set:
            self.ee_point_transmit(angle)
            time.sleep(0.08)

    def serial_transmit(self,angle_set):
        if not hasattr(self,'last_arm_value') or self.last_arm_value!=angle_set:
            str_angles=f"{','.join(map(str,angle_set))}\n"
            self.arduino_data.write(str_angles.encode())
            time.sleep(0.1)
            self.last_arm_value=angle_set
            self.get_logger().info(f'sent angls:{str_angles}')
            while(True):
                arduino_response=self.arduino_data.readline().decode().strip()
                if arduino_response=='ACK':
                    self.get_logger().info('acknowledgement recieved for arm')
                    break

                elif arduino_response=='':
                    self.get_logger().info('no acknowledgment recieved for arm')
                    sys.exit(1)
    
    
    def ee_point_transmit(self,angle):
        if not hasattr(self,'last_ee_value') or self.last_ee_value !=angle:
            str_angle=f"{angle:.2f}\n"
            self.arduino_data.write(str_angle.encode())
            time.sleep(0.1)
            self.get_logger().info(f'end effector is at:{angle}')
            self.last_ee_value=angle
            while(True):
                arduino_response=self.arduino_data.readline().decode().strip()
                if arduino_response=='ACK':
                    self.get_logger().info('acknowledgment recieved for end effector')
                    break
                
                else:
                    self.get_logger().info('no acknowledgment recieved')
                    sys.exit(1)
    

    def sequence_execution(self,joints):

        joint1=joints[0]
        joint2=joints[1]
        joint3=joints[2]

        initial_pos_joint1=90.0
        initial_pos_joint2=150.0
        initial_pos_joint3=150.0

        via_point_joint1=150.0
        via_point_joint2=95.0
        via_point_joint3=100.0

        drop_point_joint1=180.0
        drop_point_joint2=30.0
        drop_point_joint3=75.0

        ee_open=70.0
        ee_close=110.0

        # end effector open
        ee_open_op1=self.ee_actuation(ee_close,ee_open,10)

        # object pickup
        joint1_path_pickup=self.trajectory_generator(initial_pos_joint1,joint1,10,40)
        joint2_path_pickup=self.trajectory_generator(initial_pos_joint2,joint2,10,40)
        joint3_path_pickup=self.trajectory_generator(initial_pos_joint3,joint3,10,40)
        # ee_open_pickup=self.ee_actuation(ee_close,ee_open,40)

        # pickup_op1=list(zip(joint1_path_pickup,joint2_path_pickup,joint3_path_pickup,ee_open_pickup))
        pickup_op2=list(zip(joint1_path_pickup,joint2_path_pickup,joint3_path_pickup))
        
        # object grabbing
        ee_close_grab_op3=self.ee_actuation(ee_open,ee_close,10)

        # moving to intermidiate points
        joint1_path_intrm=self.trajectory_generator(joint1,via_point_joint1,10,40)
        joint2_path_intrm=self.trajectory_generator(joint2,via_point_joint2,10,40)
        joint3_path_intrm=self.trajectory_generator(joint3,via_point_joint3,10,40)

        intrm_op4=list(zip(joint1_path_intrm,joint2_path_intrm,joint3_path_intrm))

        # moving to dropping points
        joint1_path_drop=self.trajectory_generator(via_point_joint1,drop_point_joint1,10,40)
        joint2_path_drop=self.trajectory_generator(via_point_joint2,drop_point_joint2,10,40)
        joint3_path_drop=self.trajectory_generator(via_point_joint3,drop_point_joint3,10,40)
        
        drop_op5=list(zip(joint1_path_drop,joint2_path_drop,joint3_path_drop))

        # dropping the object
        ee_open_drop_op6=self.ee_actuation(ee_close,ee_open,10)

        # returning to the initial position
        joint1_path_return=self.trajectory_generator(drop_point_joint1,initial_pos_joint1,10,40)
        joint2_path_return=self.trajectory_generator(drop_point_joint2,initial_pos_joint2,10,40)
        joint3_path_return=self.trajectory_generator(drop_point_joint3,initial_pos_joint3,10,40)

        ee_return=self.ee_actuation(ee_open,ee_close,40)

        return_op7=list(zip(joint1_path_return,joint2_path_return,joint3_path_return,ee_return))

        self.ee_motion_execution(ee_open_op1)

        # arm is moving to the desired position with 4 joint actuation
        self.trajectory_execution(pickup_op2)

        # end effector is grabbing object
        self.ee_motion_execution(ee_close_grab_op3)

        # arm is moving to the intermidiate points
        self.trajectory_execution(intrm_op4)

        # arm is moving to the dropping point
        self.trajectory_execution(drop_op5)

        # end effector is releasing the object in drop zone
        self.ee_motion_execution(ee_open_drop_op6)

        # arm is moving back to its initial postion
        self.trajectory_execution(return_op7)

def main(args=None):
    rclpy.init(args=args)
    node=trajectory_generator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()