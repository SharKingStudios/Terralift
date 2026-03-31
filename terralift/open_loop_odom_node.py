#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def wrap_pi(a: float) -> float:
    while a <= -math.pi:
        a += 2.0 * math.pi
    while a > math.pi:
        a -= 2.0 * math.pi
    return a


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class OpenLoopOdom(Node):
    """Command-driven planar odometry with IMU-based heading.

    This intentionally avoids integrating IMU linear acceleration, which is very
    prone to sideways drift on low-cost IMUs. Instead, it uses commanded body
    velocities as the short-term motion prior and stabilizes heading with the IMU.
    The output is meant to be fused/smoothed by robot_localization before being
    used by SLAM / Nav2.
    """

    def __init__(self):
        super().__init__('open_loop_odom')

        self.imu_topic = str(self.declare_parameter('imu_topic', '/imu/data').value)
        self.cmd_vel_topic = str(self.declare_parameter('cmd_vel_topic', '/cmd_vel').value)
        self.odom_topic = str(self.declare_parameter('odom_topic', '/open_loop_odom').value)
        self.odom_frame = str(self.declare_parameter('odom_frame', 'odom').value)
        self.base_frame = str(self.declare_parameter('base_frame', 'base_link').value)
        self.rate_hz = float(self.declare_parameter('rate_hz', 50.0).value)
        self.publish_tf = bool(self.declare_parameter('publish_tf', False).value)
        self.reset_pose_topic = str(self.declare_parameter('reset_pose_topic', '').value)

        # Command / motion model
        self.cmd_timeout = float(self.declare_parameter('cmd_timeout', 0.35).value)
        self.vel_response_tau = float(self.declare_parameter('vel_response_tau', 0.18).value)
        self.idle_velocity_decay = float(self.declare_parameter('idle_velocity_decay', 4.0).value)
        self.max_vx = float(self.declare_parameter('max_vx_mps', 1.22).value)
        self.max_vy = float(self.declare_parameter('max_vy_mps', 1.22).value)
        self.max_wz = float(self.declare_parameter('max_wz_rps', 3.14).value)

        # Heading stabilization
        self.use_imu_orientation = bool(self.declare_parameter('use_imu_orientation', True).value)
        self.use_imu_gyro = bool(self.declare_parameter('use_imu_gyro', True).value)
        self.imu_yaw_blend_gain = float(self.declare_parameter('imu_yaw_blend_gain', 2.0).value)
        self.imu_wz_lpf_gain = float(self.declare_parameter('imu_wz_lpf_gain', 12.0).value)
        self.require_imu_yaw = bool(self.declare_parameter('require_imu_yaw', False).value)

        self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.imu_cb, 50)
        self.sub_cmd = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_cb, 20)
        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        if self.reset_pose_topic:
            self.sub_reset = self.create_subscription(PoseStamped, self.reset_pose_topic, self.reset_pose_cb, 10)
            self.get_logger().info(f"Reset pose enabled: topic={self.reset_pose_topic} frame={self.odom_frame}")

        # World state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Body-frame velocity estimate (m/s)
        self.vx_body = 0.0
        self.vy_body = 0.0

        # Latest command target (m/s, rad/s)
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0
        self.last_cmd_time = self.get_clock().now()

        # IMU state
        self.have_imu_yaw = False
        self.imu_yaw_raw: Optional[float] = None
        self.imu_wz = 0.0
        self.imu_wz_filt = 0.0
        self.yaw_offset = 0.0

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / max(1.0, self.rate_hz), self.update)

        self.get_logger().info(
            f"open_loop_odom using {self.cmd_vel_topic} + {self.imu_topic} -> {self.odom_topic}; "
            f"publish_tf={self.publish_tf}"
        )

    def imu_cb(self, msg: Imu):
        self.imu_wz = float(msg.angular_velocity.z)
        q = msg.orientation
        if (q.x, q.y, q.z, q.w) != (0.0, 0.0, 0.0, 0.0):
            self.imu_yaw_raw = yaw_from_quat(q)
            self.have_imu_yaw = True

    def cmd_cb(self, msg: Twist):
        self.cmd_vx = max(-self.max_vx, min(self.max_vx, float(msg.linear.x)))
        self.cmd_vy = max(-self.max_vy, min(self.max_vy, float(msg.linear.y)))
        self.cmd_wz = max(-self.max_wz, min(self.max_wz, float(msg.angular.z)))
        self.last_cmd_time = self.get_clock().now()

    def reset_pose_cb(self, msg: PoseStamped):
        if msg.header.frame_id and msg.header.frame_id != self.odom_frame:
            self.get_logger().warn(
                f"Ignoring reset_pose in frame '{msg.header.frame_id}' (expected '{self.odom_frame}')"
            )
            return

        self.x = float(msg.pose.position.x)
        self.y = float(msg.pose.position.y)
        desired_yaw = yaw_from_quat(msg.pose.orientation)

        if self.have_imu_yaw and self.imu_yaw_raw is not None:
            self.yaw_offset = wrap_pi(desired_yaw - self.imu_yaw_raw)
            self.yaw = desired_yaw
        else:
            self.yaw = desired_yaw

        self.vx_body = 0.0
        self.vy_body = 0.0
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0
        self.last_time = self.get_clock().now()

        self.get_logger().info(
            f"open_loop_odom SNAP -> x={self.x:.3f} y={self.y:.3f} yaw={self.yaw:.3f}"
        )

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_time = now

        # Determine current target command
        cmd_age = (now - self.last_cmd_time).nanoseconds * 1e-9
        if cmd_age > self.cmd_timeout:
            target_vx = 0.0
            target_vy = 0.0
            target_wz = 0.0
        else:
            target_vx = self.cmd_vx
            target_vy = self.cmd_vy
            target_wz = self.cmd_wz

        # Smooth body velocity response toward command target
        tau = max(1e-3, self.vel_response_tau)
        alpha = 1.0 - math.exp(-dt / tau)
        self.vx_body += alpha * (target_vx - self.vx_body)
        self.vy_body += alpha * (target_vy - self.vy_body)

        if abs(target_vx) < 1e-5 and abs(target_vy) < 1e-5:
            decay = math.exp(-max(0.0, self.idle_velocity_decay) * dt)
            self.vx_body *= decay
            self.vy_body *= decay

        # Update yaw from gyro, then gently pull toward IMU orientation if available
        if self.use_imu_gyro:
            wz_alpha = 1.0 - math.exp(-max(1e-3, self.imu_wz_lpf_gain) * dt)
            self.imu_wz_filt += wz_alpha * (self.imu_wz - self.imu_wz_filt)
            self.yaw = wrap_pi(self.yaw + self.imu_wz_filt * dt)
        else:
            self.imu_wz_filt = target_wz
            self.yaw = wrap_pi(self.yaw + target_wz * dt)

        if self.use_imu_orientation and self.have_imu_yaw and self.imu_yaw_raw is not None:
            yaw_meas = wrap_pi(self.imu_yaw_raw + self.yaw_offset)
            blend = 1.0 - math.exp(-max(1e-3, self.imu_yaw_blend_gain) * dt)
            err = wrap_pi(yaw_meas - self.yaw)
            self.yaw = wrap_pi(self.yaw + blend * err)
        elif self.require_imu_yaw and not self.have_imu_yaw:
            # Hold pose if a yaw estimate is mandatory and unavailable.
            self.publish(now)
            return

        # Integrate body velocities in odom frame
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        vx_odom = self.vx_body * cy - self.vy_body * sy
        vy_odom = self.vx_body * sy + self.vy_body * cy

        self.x += vx_odom * dt
        self.y += vy_odom * dt

        self.publish(now)

    def publish(self, stamp):
        qx, qy, qz, qw = quat_from_yaw(self.yaw)
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = float(self.vx_body)
        odom.twist.twist.linear.y = float(self.vy_body)
        odom.twist.twist.angular.z = float(self.imu_wz_filt if self.use_imu_gyro else self.cmd_wz)

        # Conservative covariance to tell downstream filters this is only an approximate prior.
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.08
        odom.twist.covariance[0] = 0.08
        odom.twist.covariance[7] = 0.08
        odom.twist.covariance[35] = 0.04

        self.pub_odom.publish(odom)

        if self.tf_broadcaster is not None:
            from geometry_msgs.msg import TransformStamped
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(self.x)
            t.transform.translation.y = float(self.y)
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopOdom()
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
