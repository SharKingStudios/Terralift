#!/usr/bin/env python3
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Joy
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


def clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


def wrap_pi(angle: float) -> float:
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    while angle > math.pi:
        angle -= 2.0 * math.pi
    return angle


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CmdVelArbiter(Node):
    """Prefer recently active teleop, otherwise pass through Nav2 safe commands."""

    def __init__(self) -> None:
        super().__init__('cmd_vel_arbiter')

        self.nav_topic = str(self.declare_parameter('nav_topic', '/cmd_vel_nav_safe').value)
        self.teleop_topic = str(self.declare_parameter('teleop_topic', '/cmd_vel_teleop').value)
        self.output_topic = str(self.declare_parameter('output_topic', '/cmd_vel_demo_drive').value)
        self.source_topic = str(self.declare_parameter('source_topic', '/demo/drive_source').value)
        self.joy_topic = str(self.declare_parameter('joy_topic', '/joy').value)
        self.odom_topic = str(self.declare_parameter('odom_topic', '/odom').value)
        self.publish_hz = float(self.declare_parameter('publish_hz', 50.0).value)
        self.teleop_msg_timeout = float(
            self.declare_parameter('teleop_msg_timeout_sec', 0.50).value
        )
        self.teleop_override_timeout = float(
            self.declare_parameter('teleop_override_timeout_sec', 0.35).value
        )
        self.nav_timeout = float(self.declare_parameter('nav_timeout_sec', 0.50).value)
        self.odom_timeout = float(self.declare_parameter('odom_timeout_sec', 0.50).value)
        self.teleop_deadband = float(self.declare_parameter('teleop_deadband', 0.03).value)
        self.field_oriented_teleop = bool(
            self.declare_parameter('field_oriented_teleop', True).value
        )
        self.reset_heading_button_index = int(
            self.declare_parameter('reset_heading_button_index', 3).value
        )

        self.nav_cmd = Twist()
        self.teleop_cmd = Twist()
        self.last_nav_stamp = None
        self.last_teleop_stamp = None
        self.last_teleop_motion_stamp = None
        self.last_odom_stamp = None
        self.last_source = 'idle'
        self.current_yaw = 0.0
        self.have_odom_heading = False
        self.teleop_heading_reference = None
        self.prev_reset_heading_pressed = False

        self.cmd_pub = self.create_publisher(Twist, self.output_topic, 10)
        self.source_pub = self.create_publisher(String, self.source_topic, 10)

        self.create_subscription(Twist, self.nav_topic, self._nav_cb, 10)
        self.create_subscription(Twist, self.teleop_topic, self._teleop_cb, 10)
        self.create_subscription(Joy, self.joy_topic, self._joy_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 20)

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

    def _joy_cb(self, msg: Joy) -> None:
        pressed = self._button_pressed(msg, self.reset_heading_button_index)
        if pressed and not self.prev_reset_heading_pressed:
            if self._is_fresh(self.last_odom_stamp, self.odom_timeout) and self.have_odom_heading:
                self.teleop_heading_reference = self.current_yaw
                self.get_logger().info(
                    f'Field-oriented teleop heading reset -> yaw={self.current_yaw:.3f} rad'
                )
            else:
                self.get_logger().warn('Ignoring teleop heading reset because /odom yaw is unavailable')
        self.prev_reset_heading_pressed = pressed

    def _odom_cb(self, msg: Odometry) -> None:
        self.current_yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.have_odom_heading = True
        self.last_odom_stamp = self.get_clock().now()
        if self.teleop_heading_reference is None:
            self.teleop_heading_reference = self.current_yaw

    def _is_fresh(self, stamp, timeout_sec: float) -> bool:
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return age <= timeout_sec

    def _button_pressed(self, joy: Joy, index: int) -> bool:
        if index < 0 or index >= len(joy.buttons):
            return False
        return bool(joy.buttons[index])

    def _field_orient_teleop(self, src: Twist) -> Twist:
        out = copy_twist(src)
        if not self.field_oriented_teleop:
            return out
        if not self._is_fresh(self.last_odom_stamp, self.odom_timeout):
            return out
        if not self.have_odom_heading or self.teleop_heading_reference is None:
            return out

        relative_yaw = wrap_pi(self.current_yaw - self.teleop_heading_reference)
        c = math.cos(relative_yaw)
        s = math.sin(relative_yaw)
        field_x = float(src.linear.x)
        field_y = float(src.linear.y)
        out.linear.x = clamp(field_x * c + field_y * s, -1.0, 1.0)
        out.linear.y = clamp(-field_x * s + field_y * c, -1.0, 1.0)
        return out

    def _publish_cmd(self) -> None:
        source = 'idle'
        out = Twist()

        teleop_active = (
            self._is_fresh(self.last_teleop_stamp, self.teleop_msg_timeout) and
            self._is_fresh(self.last_teleop_motion_stamp, self.teleop_override_timeout)
        )

        if teleop_active:
            source = 'teleop'
            out = self._field_orient_teleop(self.teleop_cmd)
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
