from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import numpy as np

my_chain = Chain(name='my_robot', links=[
    OriginLink(),
    URDFLink(
        name="joint1",
        origin_translation=[0, 0, 0],  # Translation vector
        origin_orientation=[0, 0, 0],  # Orientation (roll, pitch, yaw)
        rotation=[0, 0, 1],           # Rotation axis (x, y, z)
        bounds=(0, np.pi)             # Joint limits (0 to 180 degrees)
    ),
    URDFLink(
    name="joint2",
    origin_translation=[1.1,0,0],
    origin_orientation=[-90*np.pi/180,0,0],  # alpha=-pi/2
    rotation=[0,0,1],  # rotate around z
    bounds=(0,np.pi)

    ),
    URDFLink(
        name="joint3",
        origin_translation=[10.4, 0, 0],
        origin_orientation=[np.pi, 0, 0],
        rotation=[0, 0, 1],
        bounds=(0, np.pi)
    ),
    URDFLink(
        name="end_effector",
        origin_translation=[13.4, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 0]  # Fixed joint
    )
])


tfik = [10,18,3]

angles_rad = my_chain.inverse_kinematics(tfik)
angles_deg = [angle * 180 / np.pi for angle in angles_rad]

print("Joint angles (deg):", angles_deg)
