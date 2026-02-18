#!/usr/bin/env python3

"""Open-loop odometry (no wheel encoders).

This node exists specifically to make SLAM + Nav2 usable on the real robot when
no wheel odometry is available.

It integrates commanded base-frame velocities (Twist) into an `odom` pose and
publishes:
  - nav_msgs/Odometry on `/odom`
  - TF transform `odom -> base_link`

Yaw is taken from the IMU (recommended) when available; otherwise it will
integrate the commanded angular velocity.
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, Twist
try:
    from geometry_msgs.msg import TwistStamped
except Exception:  # pragma: no cover
    TwistStamped = None  # type: ignore
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from tf2_ros import TransformBroadcaster


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Return yaw (rad) from quaternion."""
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quat_from_yaw(yaw: float):
    """Return quaternion (x,y,z,w) for yaw-only rotation."""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


class OpenLoopOdomNode(Node):
    def __init__(self):
        super().__init__('open_loop_odom')

        # Frames
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.publish_tf = bool(self.declare_parameter('publish_tf', True).value)

        # Topics
        self.cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        self.imu_topic = self.declare_parameter('imu_topic', '/imu/data').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value

        # Integration
        self.use_imu_yaw = bool(self.declare_parameter('use_imu_yaw', True).value)
        self.odom_rate_hz = float(self.declare_parameter('rate_hz', 50.0).value)

        self.x = float(self.declare_parameter('initial_x', 0.0).value)
        self.y = float(self.declare_parameter('initial_y', 0.0).value)
        self.yaw = float(self.declare_parameter('initial_yaw', 0.0).value)

        self._last_time = self.get_clock().now()
        self._last_cmd: Twist = Twist()
        self._imu_yaw: Optional[float] = None

        self._tf_broadcaster = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        self._cmd_sub = None
        self._discover_timer = self.create_timer(1.0, self._discover_cmd_topic)
        self.create_subscription(Imu, self.imu_topic, self._on_imu, 10)

        self._timer = self.create_timer(1.0 / self.odom_rate_hz, self._on_timer)
        self.get_logger().info(
            f"Open-loop odom integrating {self.cmd_vel_topic} + {self.imu_topic} -> {self.odom_topic} and TF ({self.odom_frame}->{self.base_frame})"
        )

    def _on_cmd_vel(self, msg: Twist):
        self._last_cmd = msg

    def _on_cmd_vel_stamped(self, msg):
        # TwistStamped -> Twist
        self._last_cmd = msg.twist

    def _discover_cmd_topic(self):
        """Auto-detect Twist vs TwistStamped on the cmd_vel topic."""
        if self._cmd_sub is not None:
            return

        try:
            topics = dict(self.get_topic_names_and_types())
        except Exception:
            return

        types = topics.get(self.cmd_vel_topic, [])
        if not types:
            return

        # Prefer TwistStamped if available.
        if any(t.endswith('geometry_msgs/msg/TwistStamped') for t in types) and TwistStamped is not None:
            self._cmd_sub = self.create_subscription(TwistStamped, self.cmd_vel_topic, self._on_cmd_vel_stamped, 10)
            self.get_logger().info(f"Subscribed to {self.cmd_vel_topic} as TwistStamped")
        elif any(t.endswith('geometry_msgs/msg/Twist') for t in types):
            self._cmd_sub = self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)
            self.get_logger().info(f"Subscribed to {self.cmd_vel_topic} as Twist")

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        self._imu_yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)

    def _on_timer(self):
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self._last_time = now

        vx = float(self._last_cmd.linear.x)
        vy = float(self._last_cmd.linear.y)
        wz = float(self._last_cmd.angular.z)

        # Update yaw
        if self.use_imu_yaw and self._imu_yaw is not None:
            self.yaw = self._imu_yaw
        else:
            self.yaw += wz * dt

        # Integrate position in odom frame (Twist is in base frame)
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        dx = (vx * cy - vy * sy) * dt
        dy = (vx * sy + vy * cy) * dt
        self.x += dx
        self.y += dy

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        qx, qy, qz, qw = _quat_from_yaw(self.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self._odom_pub.publish(odom)

        # Publish TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
