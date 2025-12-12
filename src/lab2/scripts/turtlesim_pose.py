#!/usr/bin/python3

from lab2.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
import numpy as np


class DummyNode(Node):
    def __init__(self):
        super().__init__("turtlesim_pose_node")
        self.create_subscription(Pose, "/turtle1/pose", self.pose1_callback, 10)
        self.create_subscription(Pose, "/turtle2/pose", self.pose2_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)
        self.tf = TransformBroadcaster(self)

        self.turtle1_pose = np.array([0.0, 0.0, 0.0])
        self.turtle2_pose = np.array([0.0, 0.0, 0.0])

    def pose1_callback(self, msg):
        self.turtle1_pose[0] = msg.x
        self.turtle1_pose[1] = msg.y
        self.turtle1_pose[2] = msg.theta

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "odom1"

        odom.pose.pose.position.x = self.turtle1_pose[0]
        odom.pose.pose.position.y = self.turtle1_pose[1]

        q = quaternion_from_euler(0, 0, self.turtle1_pose[2])
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.odom_publisher.publish(odom)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "odom1"

        t.transform.translation.x = self.turtle1_pose[0] - 5.44
        t.transform.translation.y = self.turtle1_pose[1] - 5.44
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf.sendTransform(t)

    def pose2_callback(self, msg):
        self.turtle2_pose[0] = msg.x
        self.turtle2_pose[1] = msg.y
        self.turtle2_pose[2] = msg.theta

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "odom2"

        odom.pose.pose.position.x = self.turtle2_pose[0]
        odom.pose.pose.position.y = self.turtle2_pose[1]

        q = quaternion_from_euler(0, 0, self.turtle2_pose[2])
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.odom_publisher.publish(odom)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "odom2"

        t.transform.translation.x = self.turtle2_pose[0] - 5.44
        t.transform.translation.y = self.turtle2_pose[1] - 5.44
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
