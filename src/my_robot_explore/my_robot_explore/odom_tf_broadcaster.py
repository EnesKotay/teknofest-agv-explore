#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class OdomTFBroadcaster(Node):
    """
    /odom (nav_msgs/Odometry) -> TF yayınlar: odom -> base_footprint
    Gazebo sadece /odom verip TF vermiyorsa SLAM + Nav2 için şart.
    """

    def __init__(self):
        super().__init__("odom_tf_broadcaster")

        # NOT: use_sim_time paramını declare ETMİYORUZ (ROS zaten declare ediyor)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        self.odom_topic = self.get_parameter("odom_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.br = tf2_ros.TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.cb, 50)

        self.get_logger().info(
            f"Publishing TF {self.odom_frame} -> {self.base_frame} from {self.odom_topic}"
        )

    def cb(self, msg: Odometry):
        t = TransformStamped()

        # stamp'i /odom'dan al (sim_time ile uyumlu olur)
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame or msg.header.frame_id

        # child frame
        child = msg.child_frame_id if msg.child_frame_id else self.base_frame
        t.child_frame_id = child

        # pose -> transform
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
