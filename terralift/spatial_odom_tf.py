#!/usr/bin/env python3
"""Bridge Gazebo odometry into the dynamic odom -> base_link TF edge."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class OdomToTF(Node):
    def __init__(self) -> None:
        super().__init__("spatial_odom_tf")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_to_odom_x", -17.3)
        self.declare_parameter("map_to_odom_y", -12.0)
        self.broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        map_to_odom = TransformStamped()
        map_to_odom.header.frame_id = str(self.get_parameter("map_frame").value)
        map_to_odom.child_frame_id = str(self.get_parameter("odom_frame").value)
        map_to_odom.transform.translation.x = float(self.get_parameter("map_to_odom_x").value)
        map_to_odom.transform.translation.y = float(self.get_parameter("map_to_odom_y").value)
        map_to_odom.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(map_to_odom)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self.callback, 30)

    def callback(self, message: Odometry) -> None:
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = str(self.get_parameter("odom_frame").value)
        transform.child_frame_id = str(self.get_parameter("base_frame").value)
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
