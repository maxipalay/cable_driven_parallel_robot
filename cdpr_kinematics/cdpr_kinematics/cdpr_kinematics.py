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

class CDPRKinematics(Node):
    def __init__(self, ):
        super().__init__("kinematics")

        # robot characteristics (these should be made parameters of the node)
        self.pulley_distance = 0.976 #
        self.pulley_height = 0.95 # height of pulley at its top point
        self.head_anchor_distance = 0.1 # FOR 2D MOTION

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.broadcast_static_transforms()

        self.srv_ik_request = self.create_service(IKrequest, "ik_request", self.cb_ik_request)

        markerQoS = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        )
        self.pub_markers = self.create_publisher(
            Marker, "line_markers", markerQoS
        )

        self.ik_transform = TransformStamped()


        self._timer = self.create_timer(0.1, self.broadcast_ee_frame)

    def cb_ik_request(self, request, response):
        x = request.position.x
        y = request.position.y
        z = request.position.z

        # a vectors (different than figure 2.3, these go up)
        a1 = np.array([self.pulley_distance/2.0,self.pulley_distance/2.0,self.pulley_height])
        a2 = np.array([-self.pulley_distance/2.0,self.pulley_distance/2.0,self.pulley_height])
        a3 = np.array([-self.pulley_distance/2.0,-self.pulley_distance/2.0,self.pulley_height])
        a4 = np.array([self.pulley_distance/2.0,-self.pulley_distance/2.0,self.pulley_height])

        # b vectors
        b1 = np.array([self.head_anchor_distance/2.0,self.head_anchor_distance/2.0,0.0])
        b2 = np.array([-self.head_anchor_distance/2.0,self.head_anchor_distance/2.0,0.0])
        b3 = np.array([-self.head_anchor_distance/2.0,-self.head_anchor_distance/2.0,0.0])
        b4 = np.array([self.head_anchor_distance/2.0,-self.head_anchor_distance/2.0,0.0])

        # R
        R = np.eye(3,3)

        r = np.array([x,y,z])

        cable1 = a1-r-R@b1
        cable2 = a2-r-R@b2
        cable3 = a3-r-R@b3
        cable4 = a4-r-R@b4

        cable1_pulley_point = Point()
        cable1_pulley_point.x = a1[0]
        cable1_pulley_point.y = a1[1]
        cable1_pulley_point.z = a1[2]

        cable2_pulley_point = Point()
        cable2_pulley_point.x = a2[0]
        cable2_pulley_point.y = a2[1]
        cable2_pulley_point.z = a2[2]

        cable3_pulley_point = Point()
        cable3_pulley_point.x = a3[0]
        cable3_pulley_point.y = a3[1]
        cable3_pulley_point.z = a3[2]

        cable4_pulley_point = Point()
        cable4_pulley_point.x = a4[0]
        cable4_pulley_point.y = a4[1]
        cable4_pulley_point.z = a4[2]

        cable1_head_point = Point(x = (r+b1)[0], y = (r+b1)[1], z = (r+b1)[2])
        cable2_head_point = Point(x = (r+b2)[0], y = (r+b2)[1], z = (r+b2)[2])
        cable3_head_point = Point(x = (r+b3)[0], y = (r+b3)[1], z = (r+b3)[2])
        cable4_head_point = Point(x = (r+b4)[0], y = (r+b4)[1], z = (r+b4)[2])

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'world'
        marker.id = 1
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.ik_transform.transform.translation.x = x
        self.ik_transform.transform.translation.y = y
        self.ik_transform.transform.translation.z = z
        self.ik_transform.transform.rotation.x = 0.0
        self.ik_transform.transform.rotation.y = 0.0
        self.ik_transform.transform.rotation.z = 0.0
        self.ik_transform.transform.rotation.w = 1.0

        marker.points = [cable1_pulley_point, cable1_head_point, cable2_pulley_point, cable2_head_point, cable3_pulley_point, cable3_head_point, cable4_pulley_point, cable4_head_point]

        self.pub_markers.publish(marker)
        self.broadcast_ee_frame()

        return response

    def broadcast_ee_frame(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = "kin_ee"

        t.transform = self.ik_transform.transform

        self.tf_broadcaster.sendTransform(t)


    def broadcast_static_transforms(self):

        pulleys = {"ids": [0, 1, 2, 3], "positions": [[self.pulley_distance/2.0,self.pulley_distance/2.0,self.pulley_height],
                                                      [-self.pulley_distance/2.0,self.pulley_distance/2.0,self.pulley_height],
                                                      [-self.pulley_distance/2.0,-self.pulley_distance/2.0,self.pulley_height],
                                                      [self.pulley_distance/2.0,-self.pulley_distance/2.0,self.pulley_height]]}

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