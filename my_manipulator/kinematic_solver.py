from ikpy.chain import Chain
from ikpy.link import OriginLink,URDFLink
import numpy as np
def joint_description(joint_name:str,alpha:float,a:float,d:float,theta:float,lower_limit:float,higher_limit:float):
    if not isinstance(joint_name,str):
        raise TypeError('Joint name has to be an string followed by a number')
    for var_value in [alpha,a,d,theta,lower_limit,higher_limit]:
        if not isinstance(var_value,float):
            raise TypeError(f'{var_value} have to be a float')
    
    joint_property=URDFLink(
        name=joint_name,
        origin_translation=[a,0,d],
        origin_orientation=[alpha,0,theta],
        rotation=[0,0,1],
        bounds=[]
    )



x=joint_description('my',1.00,4.00,0,0.0,np.pi)
