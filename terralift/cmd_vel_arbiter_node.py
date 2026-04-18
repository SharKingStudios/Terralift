#!/usr/bin/env python3
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


def twist_magnitude(msg: Twist) -> float:
    return math.sqrt(
        msg.linear.x * msg.linear.x +
        msg.linear.y * msg.linear.y +
        msg.angular.z * msg.angular.z
    )


def copy_twist(src: Twist) -> Twist:
    out = Twist()
    out.linear.x = float(src.linear.x)
    out.linear.y = float(src.linear.y)
    out.linear.z = float(src.linear.z)
    out.angular.x = float(src.angular.x)
    out.angular.y = float(src.angular.y)
    out.angular.z = float(src.angular.z)
    return out


class CmdVelArbiter(Node):
    """Prefer recently active teleop, otherwise pass through Nav2 safe commands."""

    def __init__(self) -> None:
        super().__init__('cmd_vel_arbiter')

        self.nav_topic = str(self.declare_parameter('nav_topic', '/cmd_vel_nav_safe').value)
        self.teleop_topic = str(self.declare_parameter('teleop_topic', '/cmd_vel_teleop').value)
        self.output_topic = str(self.declare_parameter('output_topic', '/cmd_vel_demo_drive').value)
        self.source_topic = str(self.declare_parameter('source_topic', '/demo/drive_source').value)
        self.publish_hz = float(self.declare_parameter('publish_hz', 50.0).value)
        self.teleop_msg_timeout = float(
            self.declare_parameter('teleop_msg_timeout_sec', 0.50).value
        )
        self.teleop_override_timeout = float(
            self.declare_parameter('teleop_override_timeout_sec', 0.35).value
        )
        self.nav_timeout = float(self.declare_parameter('nav_timeout_sec', 0.50).value)
        self.teleop_deadband = float(self.declare_parameter('teleop_deadband', 0.03).value)

        self.nav_cmd = Twist()
        self.teleop_cmd = Twist()
        self.last_nav_stamp = None
        self.last_teleop_stamp = None
        self.last_teleop_motion_stamp = None
        self.last_source = 'idle'

        self.cmd_pub = self.create_publisher(Twist, self.output_topic, 10)
        self.source_pub = self.create_publisher(String, self.source_topic, 10)

        self.create_subscription(Twist, self.nav_topic, self._nav_cb, 10)
        self.create_subscription(Twist, self.teleop_topic, self._teleop_cb, 10)

        self.timer = self.create_timer(1.0 / max(1.0, self.publish_hz), self._publish_cmd)

        self.get_logger().info(
            f'Arbitrating {self.nav_topic} and {self.teleop_topic} -> {self.output_topic}'
        )

    def _nav_cb(self, msg: Twist) -> None:
        self.nav_cmd = copy_twist(msg)
        self.last_nav_stamp = self.get_clock().now()

    def _teleop_cb(self, msg: Twist) -> None:
        self.teleop_cmd = copy_twist(msg)
        now = self.get_clock().now()
        self.last_teleop_stamp = now
        if twist_magnitude(msg) > self.teleop_deadband:
            self.last_teleop_motion_stamp = now

    def _is_fresh(self, stamp, timeout_sec: float) -> bool:
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return age <= timeout_sec

    def _publish_cmd(self) -> None:
        source = 'idle'
        out = Twist()

        teleop_active = (
            self._is_fresh(self.last_teleop_stamp, self.teleop_msg_timeout) and
            self._is_fresh(self.last_teleop_motion_stamp, self.teleop_override_timeout)
        )

        if teleop_active:
            source = 'teleop'
            out = copy_twist(self.teleop_cmd)
        elif self._is_fresh(self.last_nav_stamp, self.nav_timeout):
            source = 'nav'
            out = copy_twist(self.nav_cmd)

        self.cmd_pub.publish(out)

        if source != self.last_source:
            self.last_source = source
            self.source_pub.publish(String(data=source))
            self.get_logger().info(f'Drive source -> {source}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
