import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from cdpr_kinematics.auxiliary_math import *
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from cdpr_kinematics_interfaces.srv import IKrequest
from cdpr_kinematics_interfaces.msg import JointCommand

class CDPRKinematics(Node):
    def __init__(self, ):
        super().__init__("kinematics")

        # robot characteristics (these should be made parameters of the node)
        self.pulley_distance = 0.95#0.976 #
        self.pulley_height_high = 0.95#0.965 # height of pulley at its top point
        self.pulley_height_low = 1.0-0.95 # height of pulley at its top point
        self.head_anchor_distance = 0.0450 # FOR 2D MOTION

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.broadcast_static_transforms()

        self.srv_ik_request = self.create_service(IKrequest, "ik_request", self.cb_ik_request)

        self.pub_cable_lengths = self.create_publisher(JointCommand, "joint_commands", 10)

        markerQoS = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        
        self.pub_markers = self.create_publisher(
            Marker, "line_markers", markerQoS
        )

        self.ik_transform = TransformStamped()


        self._timer = self.create_timer(0.1, self.broadcast_ee_frame)

    def cb_ik_request(self, request, response):
        """ Callback for the inverse kinematics service call. """
        # get xyz from request
        x = request.position.x
        y = request.position.y
        z = request.position.z

        # a vectors (different than figure 2.3, these go up)
        a1 = np.array([self.pulley_distance/2.0,self.pulley_distance/2.0,self.pulley_height_low])
        a2 = np.array([-self.pulley_distance/2.0,self.pulley_distance/2.0,self.pulley_height_low])
        a3 = np.array([-self.pulley_distance/2.0,-self.pulley_distance/2.0,self.pulley_height_low])
        a4 = np.array([self.pulley_distance/2.0,-self.pulley_distance/2.0,self.pulley_height_low])
        a5 = np.array([self.pulley_distance/2.0,self.pulley_distance/2.0,self.pulley_height_high])
        a6 = np.array([-self.pulley_distance/2.0,self.pulley_distance/2.0,self.pulley_height_high])
        a7 = np.array([-self.pulley_distance/2.0,-self.pulley_distance/2.0,self.pulley_height_high])
        a8 = np.array([self.pulley_distance/2.0,-self.pulley_distance/2.0,self.pulley_height_high])

        # b vectors
        b1 = np.array([self.head_anchor_distance/2.0,self.head_anchor_distance/2.0,-0.025])
        b2 = np.array([-self.head_anchor_distance/2.0,self.head_anchor_distance/2.0,-0.025])
        b3 = np.array([-self.head_anchor_distance/2.0,-self.head_anchor_distance/2.0,-0.025])
        b4 = np.array([self.head_anchor_distance/2.0,-self.head_anchor_distance/2.0,-0.025])
        b5 = np.array([self.head_anchor_distance/2.0,self.head_anchor_distance/2.0,0.025])
        b6 = np.array([-self.head_anchor_distance/2.0,self.head_anchor_distance/2.0,0.025])
        b7 = np.array([-self.head_anchor_distance/2.0,-self.head_anchor_distance/2.0,0.025])
        b8 = np.array([self.head_anchor_distance/2.0,-self.head_anchor_distance/2.0,0.025])

        # rotation matrix for calculations (we assume no rotation, orientation lock)
        R = np.eye(3,3)

        # the r vector, translation from world frame to desired ee position
        r = np.array([x,y,z])

        # calculate cable lengths
        cable1 = np.linalg.norm(a1-r-R@b1)
        cable2 = np.linalg.norm(a2-r-R@b2)
        cable3 = np.linalg.norm(a3-r-R@b3)
        cable4 = np.linalg.norm(a4-r-R@b4)
        cable5 = np.linalg.norm(a5-r-R@b5)
        cable6 = np.linalg.norm(a6-r-R@b6)
        cable7 = np.linalg.norm(a7-r-R@b7)
        cable8 = np.linalg.norm(a8-r-R@b8)

        cable1_pulley_point = Point(x = a1[0], y = a1[1], z = a1[2])
        cable2_pulley_point = Point(x = a2[0], y = a2[1], z = a2[2])
        cable3_pulley_point = Point(x = a3[0], y = a3[1], z = a3[2])
        cable4_pulley_point = Point(x = a4[0], y = a4[1], z = a4[2])
        cable5_pulley_point = Point(x = a5[0], y = a5[1], z = a5[2])
        cable6_pulley_point = Point(x = a6[0], y = a6[1], z = a6[2])
        cable7_pulley_point = Point(x = a7[0], y = a7[1], z = a7[2])
        cable8_pulley_point = Point(x = a8[0], y = a8[1], z = a8[2])

        cable1_head_point = Point(x = (r+b1)[0], y = (r+b1)[1], z = (r+b1)[2])
        cable2_head_point = Point(x = (r+b2)[0], y = (r+b2)[1], z = (r+b2)[2])
        cable3_head_point = Point(x = (r+b3)[0], y = (r+b3)[1], z = (r+b3)[2])
        cable4_head_point = Point(x = (r+b4)[0], y = (r+b4)[1], z = (r+b4)[2])
        cable5_head_point = Point(x = (r+b5)[0], y = (r+b5)[1], z = (r+b5)[2])
        cable6_head_point = Point(x = (r+b6)[0], y = (r+b6)[1], z = (r+b6)[2])
        cable7_head_point = Point(x = (r+b7)[0], y = (r+b7)[1], z = (r+b7)[2])
        cable8_head_point = Point(x = (r+b8)[0], y = (r+b8)[1], z = (r+b8)[2])

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'world'
        marker.id = 1
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        # marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.points = [cable1_pulley_point, cable1_head_point, cable2_pulley_point, cable2_head_point, cable3_pulley_point, cable3_head_point, cable4_pulley_point, cable4_head_point,
                         cable5_pulley_point, cable5_head_point, cable6_pulley_point, cable6_head_point, cable7_pulley_point, cable7_head_point, cable8_pulley_point, cable8_head_point]
        self.pub_markers.publish(marker)

        # update the ik transform
        self.ik_transform.transform.translation.x = x
        self.ik_transform.transform.translation.y = y
        self.ik_transform.transform.translation.z = z
        self.ik_transform.transform.rotation.x = 0.0
        self.ik_transform.transform.rotation.y = 0.0
        self.ik_transform.transform.rotation.z = 0.0
        self.ik_transform.transform.rotation.w = 1.0
        
        self.broadcast_ee_frame()

        response.cable_lengths = [cable1, cable2, cable3, cable4, cable5, cable6, cable7, cable8]
        
        if request.execute:
            msg = JointCommand(cable1_length = cable1, cable2_length = cable2, cable3_length = cable3, cable4_length = cable4,
                               cable5_length = cable5, cable6_length = cable6, cable7_length = cable7, cable8_length = cable8)
            self.pub_cable_lengths.publish(msg)

        return response

    def broadcast_ee_frame(self):
        """ Broadcast the transform from world to end effector. """
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = "kin_ee"

        t.transform = self.ik_transform.transform

        self.tf_broadcaster.sendTransform(t)


    def broadcast_static_transforms(self):
        """ Broadcast static transforms of the pulleys in world frame. """
        pulleys = {"ids": [0, 1, 2, 3, 4, 5, 6, 7], "positions": [[self.pulley_distance/2.0,self.pulley_distance/2.0,self.pulley_height_low],
                                                      [-self.pulley_distance/2.0,self.pulley_distance/2.0,self.pulley_height_low],
                                                      [-self.pulley_distance/2.0,-self.pulley_distance/2.0,self.pulley_height_low],
                                                      [self.pulley_distance/2.0,-self.pulley_distance/2.0,self.pulley_height_low],
                                                      [self.pulley_distance/2.0,self.pulley_distance/2.0,self.pulley_height_high],
                                                      [-self.pulley_distance/2.0,self.pulley_distance/2.0,self.pulley_height_high],
                                                      [-self.pulley_distance/2.0,-self.pulley_distance/2.0,self.pulley_height_high],
                                                      [self.pulley_distance/2.0,-self.pulley_distance/2.0,self.pulley_height_high]]}

        transforms = []

        for id in pulleys["ids"]:

            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = "pulley"+str(id)

            t.transform.translation.x = pulleys["positions"][id][0]
            t.transform.translation.y = pulleys["positions"][id][1]
            t.transform.translation.z = pulleys["positions"][id][2]
            
            quat = quaternion_from_euler(0.0,0.0,0.0)
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            transforms.append(t)

        self.tf_static_broadcaster.sendTransform(transforms)


def main():
    rclpy.init()
    node = CDPRKinematics()
    rclpy.spin(node)
    rclpy.shutdown()