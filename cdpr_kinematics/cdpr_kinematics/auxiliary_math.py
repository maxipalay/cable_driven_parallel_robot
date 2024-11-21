import numpy as np
import math
import tf_transformations
from geometry_msgs.msg import Pose

class InverseKinematics():

    def __init__(self, pulley_distance_xy: float, pulley_height_top: float, pulley_height_bottom: float, effector_distance_xy: float, effector_height: float):
        self.pulley_distance_xy = pulley_distance_xy
        self.pulley_height_top = pulley_height_top
        self.pulley_height_bottom = pulley_height_bottom
        self.effector_distance_xy = effector_distance_xy
        self.effector_height = effector_height
        
        # a vectors (different than figure 2.3, these go up)
        self.a1 = np.array([self.pulley_distance_xy/2.0,self.pulley_distance_xy/2.0,self.pulley_height_bottom])
        self.a2 = np.array([-self.pulley_distance_xy/2.0,self.pulley_distance_xy/2.0,self.pulley_height_bottom])
        self.a3 = np.array([-self.pulley_distance_xy/2.0,-self.pulley_distance_xy/2.0,self.pulley_height_bottom])
        self.a4 = np.array([self.pulley_distance_xy/2.0,-self.pulley_distance_xy/2.0,self.pulley_height_bottom])
        self.a5 = np.array([self.pulley_distance_xy/2.0,self.pulley_distance_xy/2.0,self.pulley_height_top])
        self.a6 = np.array([-self.pulley_distance_xy/2.0,self.pulley_distance_xy/2.0,self.pulley_height_top])
        self.a7 = np.array([-self.pulley_distance_xy/2.0,-self.pulley_distance_xy/2.0,self.pulley_height_top])
        self.a8 = np.array([self.pulley_distance_xy/2.0,-self.pulley_distance_xy/2.0,self.pulley_height_top])

        # b vectors
        self.b1 = np.array([self.effector_distance_xy/2.0,self.effector_distance_xy/2.0,-self.effector_height/2.0])
        self.b2 = np.array([-self.effector_distance_xy/2.0,self.effector_distance_xy/2.0,-self.effector_height/2.0])
        self.b3 = np.array([-self.effector_distance_xy/2.0,-self.effector_distance_xy/2.0,-self.effector_height/2.0])
        self.b4 = np.array([self.effector_distance_xy/2.0,-self.effector_distance_xy/2.0,-self.effector_height/2.0])
        self.b5 = np.array([self.effector_distance_xy/2.0,self.effector_distance_xy/2.0,self.effector_height/2.0])
        self.b6 = np.array([-self.effector_distance_xy/2.0,self.effector_distance_xy/2.0,self.effector_height/2.0])
        self.b7 = np.array([-self.effector_distance_xy/2.0,-self.effector_distance_xy/2.0,self.effector_height/2.0])
        self.b8 = np.array([self.effector_distance_xy/2.0,-self.effector_distance_xy/2.0,self.effector_height/2.0])

        self.r = None
        self.R = None

    def calculate(self, pose = Pose):
        """ Calculate inverse kinematics from position (x,y,z) and orientation quaternion (x,y,z,w). """

        # the r vector, translation from world frame to desired ee position
        r = np.array([pose.position.x,pose.position.y,pose.position.z])

        # rotation matrix for calculations
        # Convert to a 4x4 transformation matrix
        matrix = tf_transformations.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

        # Extract the 3x3 rotation matrix from the 4x4 transformation matrix
        R = matrix[:3, :3]

        # calculate cable lengths
        cable1 = np.linalg.norm(self.a1-r-R@self.b1)
        cable2 = np.linalg.norm(self.a2-r-R@self.b2)
        cable3 = np.linalg.norm(self.a3-r-R@self.b3)
        cable4 = np.linalg.norm(self.a4-r-R@self.b4)
        cable5 = np.linalg.norm(self.a5-r-R@self.b5)
        cable6 = np.linalg.norm(self.a6-r-R@self.b6)
        cable7 = np.linalg.norm(self.a7-r-R@self.b7)
        cable8 = np.linalg.norm(self.a8-r-R@self.b8)

        self.r = r
        self.R = R

        return [cable1, cable2, cable3, cable4, cable5, cable6, cable7, cable8]

    def get_pulley_points(self):
        return [self.a1, self.a2, self.a3,
                   self.a4, self.a5, self.a6,
                   self.a7, self.a8]
    
    def get_effector_points(self):
        return [self.r + self.b1,
                    self.r + self.R@self.b2,
                    self.r + self.R@self.b3,
                    self.r + self.R@self.b4,
                    self.r + self.R@self.b5,
                    self.r + self.R@self.b6,
                    self.r + self.R@self.b7,
                    self.r + self.R@self.b8]
        


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


