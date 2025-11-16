#!/home/samiul/Thesis_ws/tvm/bin/python3
import rclpy
from rclpy.node import Node
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from std_msgs.msg import Float32MultiArray
import numpy as np
from rclpy.qos import QoSProfile,QoSHistoryPolicy, QoSReliabilityPolicy


class kinematic_solver(Node):
    def __init__(self):
        super().__init__('kinematic_solver') #node name


        # defining qos prfile for controlled communication
        qos_profile=QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # subscription to the coordinate topic for position
        self.subscription=self.create_subscription(Float32MultiArray, 'coordinates', self.send_angles,qos_profile)
        
        #publishing joint angles for trajectory
        self.publisher=self.create_publisher(Float32MultiArray, 'angles',qos_profile)

        # defing joint parameters
        joint1=self.joint_description('joint1',  0.0    , 0.0 , 0.0, 0.0, 0.0, np.pi)
        joint2=self.joint_description('joint2', -np.pi/2, 1.1 , 0.0, 0.0, 0.0, np.pi)
        joint3=self.joint_description('joint3',  np.pi  , 10.4, 0.0, 0.0, 0.0, np.pi)
        endeff=self.joint_description('joint4',  0.0    , 13.4, 0.0, 0.0, 0.0, np.pi)

        # 13.4

        #chain of the links of manipulator
        self.chain=Chain(
            name='my_manipulator',
            links=[OriginLink(), joint1, joint2, joint3, endeff]
        )

    #defining callback for publishing joint angles after recieving coordinates
    def send_angles(self,position):
        if len(position.data)<2:
            self.get_logger().error('incomplete coordinates')
            return
        

        # extracting the coordinates from camera
        x=position.data[0]
        y=position.data[1]
        z=6.2

        # performing inverse kinamtic operation
        try:
            joint_angles=self.chain.inverse_kinematics([x,y,z])
        except:
            self.get_logger().warning('failed to solve kinematic equations')
            return

        # rad to degree conversion
        joint_angles_degree=[]
        i=0
        for angle in joint_angles:
            angle=round((angle*180/np.pi),3)
            if i>0 and i<4:
                joint_angles_degree.append(angle)
            
            i+=1
        
        # publishing joint angles to angles topic 
        kinematic_result=Float32MultiArray()
        kinematic_result.data=joint_angles_degree

        self.publisher.publish(kinematic_result)

        self.get_logger().info(f"joint angles are : {joint_angles_degree}")
        

    # defining link parameters for ikpy to perform inverse kineamtics
    def joint_description(self, joint_name:str, alpha: float, a: float, d: float, theta: float, lower_limit:   float, higher_limit: float):
        for value in[alpha, a, d, theta, lower_limit, higher_limit]:
            if not isinstance(value, float):
                raise TypeError(f'{value} is not a float')
            
        joint_parameters=URDFLink(
            name=joint_name,
            origin_translation=[a,0,d],
            origin_orientation=[alpha,0,theta],
            rotation= [0, 0, 1],
            bounds=[lower_limit,higher_limit]
        )
        
        return joint_parameters


def main(args=None):
    rclpy.init(args=args)
    node=kinematic_solver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()