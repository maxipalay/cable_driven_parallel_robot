import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from cdpr_kinematics.auxiliary_math import *
from geometry_msgs.msg import Point
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from cdpr_kinematics.auxiliary_math import InverseKinematics
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from cdpr_kinematics_interfaces.srv import IKrequest
from cdpr_kinematics_interfaces.msg import JointCommand
from geometry_msgs.msg import PoseStamped
from cdpr_kinematics.auxiliary_math import InverseKinematics

class CDPRKinematics(Node):
    def __init__(self, ):
        super().__init__("kinematics")

        # robot characteristics (these should be made parameters of the node)
        self.pulley_distance = 0.95#0.976 #
        self.pulley_height_high = 0.95#0.965 # height of pulley at its top point
        self.pulley_height_low = 1.0-0.95 # height of pulley at its top point
        self.head_anchor_distance = 0.0450 # FOR 2D MOTION
        self.effector_height = 0.05
        self.visualization_enabled = True

        self.inv_kin = InverseKinematics(self.pulley_distance, self.pulley_height_high, self.pulley_height_low, self.head_anchor_distance, self.effector_height)

        self.sub_ik_request = self.create_subscription(PoseStamped, "ik_request", self.cb_ik_request, 10)

        self.pub_cable_lengths = self.create_publisher(JointCommand, "joint_commands", 10)

        self.ik_transform = TransformStamped()

        self._last_pose = None


        if self.visualization_enabled:
            self.tf_broadcaster = TransformBroadcaster(self)
            self.tf_static_broadcaster = StaticTransformBroadcaster(self)
            self.broadcast_static_transforms()
            self._timer = self.create_timer(0.1, self.broadcast_ee_frame)

            markerQoS = QoSProfile(
                depth=10,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
            
            self.pub_markers = self.create_publisher(
                Marker, "line_markers", markerQoS
            )
            

    def cb_ik_request(self, msg):
        """ Callback for the inverse kinematics service call. """
        if self._last_pose != msg:
            # if the last received pose and the new pose are different
            if self._last_pose is None or self._last_pose.header.stamp <= msg.header.stamp:
                #self._last_pose = msg
                pass

            cable1, cable2, cable3, cable4, cable5, cable6, cable7, cable8 = self.inv_kin.calculate(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)

            # publish cable lengths
            msg_joints = JointCommand(cable1_length = cable1, cable2_length = cable2, cable3_length = cable3, cable4_length = cable4,
                                cable5_length = cable5, cable6_length = cable6, cable7_length = cable7, cable8_length = cable8)
            
            self.pub_cable_lengths.publish(msg_joints)
            
            self.get_logger().info("publishing joint commands")

            if self.visualization_enabled:

                pulley_points = self.inv_kin.get_pulley_points()
                effector_points = self.inv_kin.get_effector_points()

                pulley_point_arr = []
                for point in pulley_points:
                    pulley_point_arr.append(Point(x = point[0], y = point[1], z = point[2]))

                effector_point_arr = []
                for point in effector_points:
                    effector_point_arr.append(Point(x = point[0], y = point[1], z = point[2]))

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

                marker_points = []

                for i in range(len(pulley_point_arr)):
                    marker_points.append(pulley_point_arr[i])
                    marker_points.append(effector_point_arr[i])

                marker.points = marker_points
                self.pub_markers.publish(marker)

                # update the ik transform
                self.ik_transform.transform.translation.x = msg.pose.position.x
                self.ik_transform.transform.translation.y = msg.pose.position.y
                self.ik_transform.transform.translation.z = msg.pose.position.z
                self.ik_transform.transform.rotation.x = 0.0
                self.ik_transform.transform.rotation.y = 0.0
                self.ik_transform.transform.rotation.z = 0.0
                self.ik_transform.transform.rotation.w = 1.0
                
                self.broadcast_ee_frame()
            

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