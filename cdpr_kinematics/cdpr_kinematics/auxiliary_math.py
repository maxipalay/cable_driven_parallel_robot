import numpy as np
import math
import tf_transformations
from geometry_msgs.msg import Pose
import sympy as sp
from itertools import combinations
from scipy.spatial import ConvexHull

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
        R = np.eye(3,3)

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
    
    def calculate_with_tensions(self, current_pose: Pose, target_pose: Pose):
        """ Calculates inverse kinematics and feedforward tensions for going from 
            current_pose to target_pose applying a linear force f along the straight 
            line that goes from current_pose to target_pose. This does not take into
            consideration changes in pose angle. """
        
        # calculate force vector F
        #vec = [target_pose.position.x - current_pose.position.x, 
        #       target_pose.position.y - current_pose.position.y, 
        #       target_pose.position.z - current_pose.position.z]
        
        
        #F = np.array([-vec[0], -vec[1], -vec[2]-2.0, 0.0, 0.0])
        F = np.array([0.0, 0.0, -2.0, 0.0, 0.0])
        #F = F/np.linalg.norm(F)*5.0 # we're asking for a total force of 20N in the direction of displacement
        #F = F*10.0 # if the workspace is constrained, the module of F should always be less than 1 meter
        print(f"force vector: {F}")
        # make r and R
        # the r vector, translation from world frame to desired ee position
        #r = np.array([current_pose.position.x,current_pose.position.y,current_pose.position.z])
        r = np.array([target_pose.position.x,target_pose.position.y,target_pose.position.z])

        # rotation matrix for calculations
        # Convert to a 4x4 transformation matrix
        #matrix = tf_transformations.quaternion_matrix([current_pose.orientation.x, 
        #                                               current_pose.orientation.y, 
        #                                               current_pose.orientation.z, 
        #                                               current_pose.orientation.w])

        # Extract the 3x3 rotation matrix from the 4x4 transformation matrix
        #R = matrix[:3, :3]
        R = np.eye(3, 3)

        # calculate cable vectors
        l1 = self.a1-r-R@self.b1
        l2 = self.a2-r-R@self.b2
        l3 = self.a3-r-R@self.b3
        l4 = self.a4-r-R@self.b4
        l5 = self.a5-r-R@self.b5
        l6 = self.a6-r-R@self.b6
        l7 = self.a7-r-R@self.b7
        l8 = self.a8-r-R@self.b8

        # construct jacobian
        jacobian = np.zeros((8,6))
        jacobian[0,0:3] = l1/np.linalg.norm(l1)
        jacobian[1,0:3] = l2/np.linalg.norm(l2)
        jacobian[2,0:3] = l3/np.linalg.norm(l3)
        jacobian[3,0:3] = l4/np.linalg.norm(l4)
        jacobian[4,0:3] = l5/np.linalg.norm(l5)
        jacobian[5,0:3] = l6/np.linalg.norm(l6)
        jacobian[6,0:3] = l7/np.linalg.norm(l7)
        jacobian[7,0:3] = l8/np.linalg.norm(l8)
        jacobian[0,3:6] = l1/np.linalg.norm(l1)@skew_symmetric(R@self.b1)
        jacobian[1,3:6] = l2/np.linalg.norm(l2)@skew_symmetric(R@self.b2)
        jacobian[2,3:6] = l3/np.linalg.norm(l3)@skew_symmetric(R@self.b3)
        jacobian[3,3:6] = l4/np.linalg.norm(l4)@skew_symmetric(R@self.b4)
        jacobian[4,3:6] = l5/np.linalg.norm(l5)@skew_symmetric(R@self.b5)
        jacobian[5,3:6] = l6/np.linalg.norm(l6)@skew_symmetric(R@self.b6)
        jacobian[6,3:6] = l7/np.linalg.norm(l7)@skew_symmetric(R@self.b7)
        jacobian[7,3:6] = l8/np.linalg.norm(l8)@skew_symmetric(R@self.b8)

        # get the W matrix
        J = jacobian[:,:5]

        ######################## TENSION CALC ##########################
        f_min = np.array([10, 10, 10, 10, 10, 10, 10, 10])  # Minimum tensions
        f_max = np.array([60, 60, 60, 60, 60, 60, 60, 60])  # Maximum tensions
        A_T = J.T

        # Perform the QR decomposition with column pivoting
        Q, R = np.linalg.qr(A_T.T, mode='complete')  # Transpose A^T to make it 4x2

        # Determine the rank based on the diagonal of R
        tolerance = 1e-10  # Threshold to consider values as zero
        rank = np.sum(np.abs(np.diag(R)) > tolerance)

        # Extract the null space as the last columns of Q
        H = Q[:, rank:]  # Columns of Q corresponding to zero singular values

        # particular sol
        Q, R = np.linalg.qr(A_T, mode='complete')

        y = -Q.T @ F
        p = np.linalg.pinv(R)@y


        # Generate all combinations of r constraints
        m, r = H.shape  # m inequalities, r-dimensional subspace

        # Generate the equations corresponding to the inequalities f_min <= H*lambda <= f_max
        # For each row in H, we create two inequalities: one for f_min and one for f_max
        equations = []

        for i in range(H.shape[0]):  # Iterate over the rows of H
            # First inequality: H[i, :] * lambda >= f_min[i]
            equations.append((H[i, :], f_min[i]-p[i]))  # Coefficients and right-hand side for f_min
            
            # Second inequality: H[i, :] * lambda <= f_max[i]
            equations.append((H[i, :], f_max[i]-p[i]))  # Coefficients and right-hand side for f_max

        # List to store valid intersection points (solutions)
        valid_points = []

        # Generate all combinations of 2 equations (pairwise systems of 2 linear equations)
        for eqs in combinations(equations, r):
            # Extract the coefficients and right-hand sides of the pair of equations
            coeff1, rhs1 = eqs[0]
            coeff2, rhs2 = eqs[1]
            coeff3, rhs3 = eqs[2]
            
            # Form the system of equations: Ax = b
            A = np.array([coeff1, coeff2, coeff3])  # Coefficients matrix
            b = np.array([rhs1, rhs2, rhs3])      # Right-hand side vector

            try:
                # Solve the system using least squares (more robust for ill-conditioned systems)
                lambda_sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)

                # Check if the solution satisfies all inequalities
                # 6. Map CoG back to force space
                f_candidate = H @ lambda_sol + p
                # 7. Validate final force distribution
                if np.all(f_candidate >= f_min) and np.all(f_candidate <= f_max):
                # Since we are generating the equations directly from the bounds, this should already be valid
                    valid_points.append(tuple(lambda_sol))  # Store the valid solution as a tuple

            except np.linalg.LinAlgError:
                # In case the system is singular or there's no solution
                continue

        # Remove duplicate points by using a set
        valid_points = list(set(valid_points))

        # Convert back to NumPy array for further processing
        valid_points_array = np.array(valid_points)

        # # Output the valid intersection points
        # print("Valid Intersection Points:")
        # for point in valid_points_array:
        #     print(point)
        # 5. Compute the center of gravity (CoG) of the convex polyhedron
        f = []
        if len(valid_points_array) > 2:
            try:
                # hull = ConvexHull(valid_points_array)  # Convex hull of the vertices
                # cog = np.mean(valid_points_array[hull.vertices], axis=0)  # CoG of the polyhedron
                # Step 1: Compute the convex hull of the valid points
                hull = ConvexHull(valid_points_array)

                # Step 2: Calculate the centroids of each simplex (triangle, tetrahedron, etc.)
                centroids = []
                volumes = []

                for simplex in hull.simplices:
                    # Get the vertices of the simplex
                    simplex_points = valid_points_array[simplex]
                    
                    # Step 2a: Calculate the centroid of the simplex (average of its vertices)
                    centroid = np.mean(simplex_points, axis=0)
                    centroids.append(centroid)
                    
                    # Step 2b: Compute the volume of the simplex (for 3D, use the determinant)
                    # Here using 3D tetrahedron volume formula as an example:
                    v1, v2, v3 = simplex_points[0], simplex_points[1], simplex_points[2]
                    v4 = simplex_points[3] if len(simplex_points) == 4 else None  # for tetrahedron (4 points)
                    
                    # Volume (or area) calculation (adjust depending on dimensionality)
                    volume = np.abs(np.linalg.det([v1, v2, v3])) / 6.0  # For 3D, for 2D use cross product area
                    
                    volumes.append(volume)

                # Step 3: Compute the weighted center of gravity
                total_volume = np.sum(volumes)
                weighted_centroid = np.sum([centroid * volume for centroid, volume in zip(centroids, volumes)], axis=0) / total_volume

                # print(f"Center of Gravity: {weighted_centroid}")
                cog = weighted_centroid
            except Exception as e:
                print(e)
                return self.calculate(target_pose), []
            # 6. Map CoG back to force space
            f = H @ cog + p

            # 7. Validate final force distribution
            if not np.all(f >= f_min) or not np.all(f <= f_max):
                #raise ValueError("Computed force distribution violates constraints.")
                print(f"calculated tensions violate constraints")
                print(f)
                f = np.zeros(f.shape)
        else:
            print("Not enough valid points to compute a convex hull.")

        tensions = np.zeros(8)
        if len(list(f))>0:
            tensions = list(f)
        ######################### TENSION CALC - END ##########################

        lengths = self.calculate(target_pose)

        return lengths, tensions

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
        
def skew_symmetric(x : np.array):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

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