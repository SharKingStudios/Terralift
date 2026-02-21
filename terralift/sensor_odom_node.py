#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    # yaw (Z) from quaternion
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class SlamPoseState:
    x: float
    y: float
    yaw: float
    stamp: Time


class SensorOdomNode(Node):
    """
    Publish odom->base_link from sensor estimates:
      - translation: derived from slam_toolbox PoseStamped deltas
      - rotation: derived from IMU yaw (BNO055)
    This avoids using commanded velocity to "fake" motion.
    """

    def __init__(self) -> None:
        super().__init__('sensor_odom')

        # Params
        self.pose_topic = self.declare_parameter('slam_pose_topic', '/pose').value
        self.imu_topic = self.declare_parameter('imu_topic', '/imu/data').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value

        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value

        # Reject slam jumps bigger than this (meters). Loop closures can jump a lot.
        self.max_delta_m = float(self.declare_parameter('max_delta_m', 0.75).value)

        # If true, use IMU quaternion as-is. If false, use IMU yaw only with flat roll/pitch.
        self.use_full_imu_quat = bool(self.declare_parameter('use_full_imu_quat', True).value)

        # Internal state
        self.last_slam: Optional[SlamPoseState] = None
        self.have_imu = False
        self.imu_q = (0.0, 0.0, 0.0, 1.0)
        self.imu_yaw = 0.0

        self.odom_x = 0.0
        self.odom_y = 0.0

        # Pub/sub
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_pub = TransformBroadcaster(self)

        self.create_subscription(Imu, self.imu_topic, self.imu_cb, 50)
        self.create_subscription(PoseStamped, self.pose_topic, self.pose_cb, 50)

        self.get_logger().info(
            f"sensor_odom: using slam pose {self.pose_topic} + IMU {self.imu_topic} -> {self.odom_topic} and TF"
        )

    def imu_cb(self, msg: Imu) -> None:
        q = msg.orientation
        self.imu_q = (q.x, q.y, q.z, q.w)
        self.imu_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.have_imu = True

    def pose_cb(self, msg: PoseStamped) -> None:
        # Slam pose is in map frame. We use deltas only.
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        q = msg.pose.orientation
        slam_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        stamp = Time.from_msg(msg.header.stamp)

        if self.last_slam is None:
            self.last_slam = SlamPoseState(x=x, y=y, yaw=slam_yaw, stamp=stamp)
            # initialize odom at origin; no jump
            self.publish(stamp)
            return

        dx = x - self.last_slam.x
        dy = y - self.last_slam.y
        d = math.hypot(dx, dy)

        # Reject huge jumps (loop closure) so odom doesn't teleport.
        if d <= self.max_delta_m:
            self.odom_x += dx
            self.odom_y += dy
        else:
            self.get_logger().warn(
                f"Rejecting slam jump Δ={d:.3f} m (dx={dx:.3f}, dy={dy:.3f}) > max_delta_m={self.max_delta_m}"
            )

        self.last_slam = SlamPoseState(x=x, y=y, yaw=slam_yaw, stamp=stamp)
        self.publish(stamp)

    def publish(self, stamp: Time) -> None:
        # Orientation: IMU truth if available, else keep yaw=0
        if self.have_imu:
            ox, oy, oz, ow = self.imu_q if self.use_full_imu_quat else (0.0, 0.0, math.sin(self.imu_yaw/2.0), math.cos(self.imu_yaw/2.0))
        else:
            ox, oy, oz, ow = (0.0, 0.0, 0.0, 1.0)

        # Odometry msg
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(self.odom_x)
        odom.pose.pose.position.y = float(self.odom_y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = float(ox)
        odom.pose.pose.orientation.y = float(oy)
        odom.pose.pose.orientation.z = float(oz)
        odom.pose.pose.orientation.w = float(ow)

        # We don't publish twist (no encoders); leave zeros.
        self.odom_pub.publish(odom)

        # TF odom->base_link
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = float(self.odom_x)
        t.transform.translation.y = float(self.odom_y)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = float(ox)
        t.transform.rotation.y = float(oy)
        t.transform.rotation.z = float(oz)
        t.transform.rotation.w = float(ow)
        self.tf_pub.sendTransform(t)


def main():
    rclpy.init()
    node = SensorOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()