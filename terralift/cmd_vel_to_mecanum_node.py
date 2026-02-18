#!/usr/bin/env python3

"""Convert Nav2 `/cmd_vel` (m/s, rad/s) to Terralift `/cmd_mecanum` (-1..1).

Your drivetrain node expects normalized commands in a Twist message:
  linear.x, linear.y, angular.z in [-1, 1]

Nav2 publishes real units on `/cmd_vel`. This adapter lets you keep the
codebase stable and tune scaling with 3 parameters.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
try:
    from geometry_msgs.msg import TwistStamped
except Exception:  # pragma: no cover
    TwistStamped = None  # type: ignore


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class CmdVelToMecanum(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_mecanum')

        self.input_topic = self.declare_parameter('input_topic', '/cmd_vel').value
        self.output_topic = self.declare_parameter('output_topic', '/cmd_mecanum').value

        # These should match your real robot's achievable speeds.
        self.max_vx = float(self.declare_parameter('max_vx_mps', 0.6).value)
        self.max_vy = float(self.declare_parameter('max_vy_mps', 0.6).value)
        self.max_wz = float(self.declare_parameter('max_wz_rps', 1.8).value)

        self.pub = self.create_publisher(Twist, self.output_topic, 10)

        self.sub = None
        self._discover_timer = self.create_timer(1.0, self._discover_input_topic)
        self.get_logger().info(
            f"Scaling {self.input_topic} -> {self.output_topic} using max_vx={self.max_vx} m/s, max_vy={self.max_vy} m/s, max_wz={self.max_wz} rad/s"
        )

    def _cb(self, msg: Twist):
        out = Twist()

        # Avoid divide-by-zero if someone sets a max to 0.
        if self.max_vx > 1e-6:
            out.linear.x = _clamp(msg.linear.x / self.max_vx, -1.0, 1.0)
        if self.max_vy > 1e-6:
            out.linear.y = _clamp(msg.linear.y / self.max_vy, -1.0, 1.0)
        if self.max_wz > 1e-6:
            out.angular.z = _clamp(msg.angular.z / self.max_wz, -1.0, 1.0)

        self.pub.publish(out)

    def _cb_stamped(self, msg):
        self._cb(msg.twist)

    def _discover_input_topic(self):
        if self.sub is not None:
            return

        try:
            topics = dict(self.get_topic_names_and_types())
        except Exception:
            return

        types = topics.get(self.input_topic, [])
        if not types:
            return

        if any(t.endswith('geometry_msgs/msg/TwistStamped') for t in types) and TwistStamped is not None:
            self.sub = self.create_subscription(TwistStamped, self.input_topic, self._cb_stamped, 10)
            self.get_logger().info(f"Subscribed to {self.input_topic} as TwistStamped")
        elif any(t.endswith('geometry_msgs/msg/Twist') for t in types):
            self.sub = self.create_subscription(Twist, self.input_topic, self._cb, 10)
            self.get_logger().info(f"Subscribed to {self.input_topic} as Twist")


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToMecanum()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
