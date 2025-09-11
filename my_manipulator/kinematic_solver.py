from ikpy.chain import Chain
from ikpy.link import OriginLink,URDFLink
import numpy as np


# function to declare joint parameter for individual joint
def joint_description(joint_name:str,alpha:float,a:float,d:float,theta:float,lower_limit:float,higher_limit:float):
    # check the type of joint_name input
    if not isinstance(joint_name,str):
        raise TypeError('Joint name has to be an string followed by a number')
    
    # check the type of input parameters
    for var_value in [alpha,a,d,theta,lower_limit,higher_limit]:
        if not isinstance(var_value,float):
            raise TypeError(f'{var_value} have to be a float')
    
    # declare joint properties
    joint_property=URDFLink(
        name=joint_name,
        origin_translation=[a,0,d],
        origin_orientation=[alpha,0,theta],
        rotation=[0,0,1],
        bounds=[lower_limit,higher_limit]
    )
    return joint_property


# declare joints
joint1=joint_description('joint1', 0.0, 0.0, 0.0, 0.0, 0.0, np.pi)
joint2=joint_description('joint2', ((-np.pi)/2), 1.1, 0.0, 0.0, 0.0, np.pi)
joint3=joint_description('joint3', np.pi, 10.4, 0.0, 0.0, 0.0, np.pi)
end_eff=joint_description('end_eff', 0.0, 13.4, 0.0, 0.0, 0.0, np.pi)


# manually implement joint parameters in the kinematic chain
my_manipulator=Chain(
    name='my_manipulator',
    links=[
        OriginLink(),
        joint1,
        joint2,
        joint3,
        end_eff
    ]
)

# desired position from the forward kinematics
desired_position=[16,5,3]


# calculate joint angles by implementing inverse kinematics function of the ikpy
joint_angles=my_manipulator.inverse_kinematics(desired_position)
# joint angles are in radian

# list for the joint angles in degree
angles_deg = []


# convert joint angles from radian to degree
for angles in joint_angles:
    angle=angles*180/np.pi
    angles_deg.append(angle)


print(angles_deg)
