#!/usr/bin/env python3
"""IMU + SLAM odometry (no wheel encoders, no cmd_vel).

Purpose
-------
Nav2 expects a smooth /odom and TF (odom->base_link). With no wheel encoders and
no use of cmd_vel, we can still produce a usable odom estimate by:

  1) Propagating planar velocity/position at high rate using IMU linear
     acceleration (x/y) + IMU yaw.
  2) Correcting that drift toward SLAM Toolbox pose (/pose) using a bounded
     complementary update.

Key requirements
----------------
- ZERO command interference: this node does not subscribe to cmd_vel.
- Yaw source is IMU only (with resettable yaw offset).
- SLAM pose is used only as a correction signal (won't teleport /odom on
  loop-closures).

Frames
------
- SLAM pose (/pose) is in map frame.
- This node publishes /odom and TF with frame_id=odom_frame.
- We treat SLAM pose as a measurement of position and keep an internal SLAM
  reference so that "reset" makes pose return to (0,0,0) in odom.

Assumptions
-----------
- IMU linear_acceleration is *gravity-compensated* OR robot stays level enough
  that planar x/y accel is usable.
- IMU axes roughly align with base_link axes. (If not, fix imu_link->base_link TF)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a: float) -> float:
    # (-pi, pi]
    while a <= -math.pi:
        a += 2.0 * math.pi
    while a > math.pi:
        a -= 2.0 * math.pi
    return a


def yaw_to_quat(yaw: float):
    h = 0.5 * yaw
    return (0.0, 0.0, math.sin(h), math.cos(h))


@dataclass
class SlamMeas:
    x: float
    y: float
    stamp: Time


class SensorOdomNode(Node):
    def __init__(self):
        super().__init__('sensor_odom')

        # Topics/frames
        self.slam_pose_topic = self.declare_parameter('slam_pose_topic', '/pose').value
        self.imu_topic = self.declare_parameter('imu_topic', '/imu/data').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.publish_tf = bool(self.declare_parameter('publish_tf', True).value)

        # Publishing rate
        self.rate_hz = float(self.declare_parameter('rate_hz', 50.0).value)

        # IMU usage
        self.use_full_imu_quat = bool(self.declare_parameter('use_full_imu_quat', True).value)
        self.require_imu_yaw = bool(self.declare_parameter('require_imu_yaw', True).value)
        self.use_imu_linear_accel = bool(self.declare_parameter('use_imu_linear_accel', True).value)

        # SLAM correction tuning
        self.slam_pos_gain = float(self.declare_parameter('slam_pos_gain', 0.35).value)  # 0..1
        self.slam_vel_gain = float(self.declare_parameter('slam_vel_gain', 0.50).value)  # 0..1
        self.max_slam_step_m = float(self.declare_parameter('max_slam_step_m', 0.50).value)  # meters/update

        # Accel integration tuning
        self.vel_decay = float(self.declare_parameter('vel_decay', 0.20).value)  # 1/s
        self.bias_learn_rate = float(self.declare_parameter('bias_learn_rate', 0.05).value)  # 1/s
        self.stationary_accel_thresh = float(self.declare_parameter('stationary_accel_thresh', 0.25).value)  # m/s^2
        self.stationary_gyro_thresh = float(self.declare_parameter('stationary_gyro_thresh', 0.25).value)  # rad/s
        self.stationary_vel_decay = float(self.declare_parameter('stationary_vel_decay', 2.0).value)  # 1/s

        # Safety clamp
        self.max_speed = float(self.declare_parameter('max_speed', 2.0).value)  # m/s

        # State (odom frame)
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0

        # IMU state
        self.have_imu = False
        self.imu_q = (0.0, 0.0, 0.0, 1.0)
        self.imu_yaw_raw: Optional[float] = None
        self.yaw_offset = 0.0
        self.yaw = 0.0
        self.imu_ax = 0.0
        self.imu_ay = 0.0
        self.imu_wz = 0.0

        # Bias estimate for accel (base frame)
        self.bias_ax = 0.0
        self.bias_ay = 0.0

        # SLAM measurement bookkeeping
        self._slam_ref_x: Optional[float] = None
        self._slam_ref_y: Optional[float] = None
        self._last_slam_meas: Optional[SlamMeas] = None

        # Timing
        self._last_update_time = self.get_clock().now()

        # ROS I/O
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_pub = TransformBroadcaster(self)

        self.create_subscription(Imu, self.imu_topic, self.imu_cb, 50)
        self.create_subscription(PoseStamped, self.slam_pose_topic, self.pose_cb, 10)

        self._timer = self.create_timer(1.0 / max(1.0, self.rate_hz), self._on_timer)
        self._reset_srv = self.create_service(Empty, '~/reset', self._on_reset)

        self.get_logger().info(
            f"sensor_odom: IMU(yaw+accel) + SLAM(/pose) correction -> {self.odom_topic} and TF {self.odom_frame}->{self.base_frame} "
            f"(NO cmd_vel). rate={self.rate_hz}Hz"
        )

    # ---------------- IMU ----------------
    def imu_cb(self, msg: Imu):
        q = msg.orientation
        # Some IMUs publish zero quaternion before fully initialized.
        if (q.x, q.y, q.z, q.w) != (0.0, 0.0, 0.0, 0.0):
            self.imu_q = (q.x, q.y, q.z, q.w)
            self.imu_yaw_raw = quat_to_yaw(q.x, q.y, q.z, q.w)
            self.have_imu = True

        self.imu_ax = float(msg.linear_acceleration.x)
        self.imu_ay = float(msg.linear_acceleration.y)
        self.imu_wz = float(msg.angular_velocity.z)

    # ---------------- SLAM pose measurement ----------------
    def pose_cb(self, msg: PoseStamped):
        slam_x = float(msg.pose.position.x)
        slam_y = float(msg.pose.position.y)
        stamp = Time.from_msg(msg.header.stamp)

        # Establish reference so that after reset, pose is relative to the reset point.
        if self._slam_ref_x is None or self._slam_ref_y is None:
            self._slam_ref_x = slam_x
            self._slam_ref_y = slam_y
            self._last_slam_meas = SlamMeas(0.0, 0.0, stamp)
            self.get_logger().info('sensor_odom: got first SLAM pose; set SLAM reference (pose becomes relative).')
            return

        x_meas = slam_x - self._slam_ref_x
        y_meas = slam_y - self._slam_ref_y

        # Bounded position correction step
        rx = x_meas - self.x
        ry = y_meas - self.y
        r = math.hypot(rx, ry)
        if self.max_slam_step_m > 0.0 and r > self.max_slam_step_m:
            s = self.max_slam_step_m / r
            rx *= s
            ry *= s

        self.x += self.slam_pos_gain * rx
        self.y += self.slam_pos_gain * ry

        # Velocity correction from SLAM delta
        if self._last_slam_meas is not None:
            dt = (stamp - self._last_slam_meas.stamp).nanoseconds * 1e-9
            if dt > 1e-3:
                vmx = (x_meas - self._last_slam_meas.x) / dt
                vmy = (y_meas - self._last_slam_meas.y) / dt
                self.vx += self.slam_vel_gain * (vmx - self.vx)
                self.vy += self.slam_vel_gain * (vmy - self.vy)

        self._last_slam_meas = SlamMeas(x_meas, y_meas, stamp)

    # ---------------- Main update loop ----------------
    def _on_timer(self):
        now = self.get_clock().now()
        dt = (now - self._last_update_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self._last_update_time = now

        # Yaw (IMU only)
        if self.imu_yaw_raw is not None:
            self.yaw = wrap_pi(self.imu_yaw_raw - self.yaw_offset)
        elif self.require_imu_yaw:
            # No yaw available yet -> hold yaw at 0
            self.yaw = 0.0

        # IMU accel propagate
        if self.use_imu_linear_accel:
            # Bias-corrected accel (base frame)
            ax = self.imu_ax - self.bias_ax
            ay = self.imu_ay - self.bias_ay

            accel_mag = math.hypot(ax, ay)
            gyro_mag = abs(self.imu_wz)

            # Bias learning + stronger damping when stationary
            if accel_mag < self.stationary_accel_thresh and gyro_mag < self.stationary_gyro_thresh:
                self.bias_ax += self.bias_learn_rate * (self.imu_ax - self.bias_ax) * dt
                self.bias_ay += self.bias_learn_rate * (self.imu_ay - self.bias_ay) * dt

                damp = max(0.0, 1.0 - self.stationary_vel_decay * dt)
                self.vx *= damp
                self.vy *= damp

            # Rotate base accel -> odom frame using yaw
            cy = math.cos(self.yaw)
            sy = math.sin(self.yaw)
            aox = ax * cy - ay * sy
            aoy = ax * sy + ay * cy

            self.vx += aox * dt
            self.vy += aoy * dt

            # Drift damping
            if self.vel_decay > 0.0:
                damp = max(0.0, 1.0 - self.vel_decay * dt)
                self.vx *= damp
                self.vy *= damp

            # Clamp
            spd = math.hypot(self.vx, self.vy)
            if self.max_speed > 0.0 and spd > self.max_speed:
                s = self.max_speed / spd
                self.vx *= s
                self.vy *= s

            # Integrate position
            self.x += self.vx * dt
            self.y += self.vy * dt

        self.publish(now)

    # ---------------- Publish ----------------
    def publish(self, stamp: Time) -> None:
        # Orientation
        if self.have_imu and self.use_full_imu_quat:
            # Use full IMU quaternion, but apply yaw offset by rebuilding yaw-only quat.
            # This keeps roll/pitch from IMU out of Nav2 costmaps (safer on ground robots).
            qx, qy, qz, qw = yaw_to_quat(self.yaw)
        else:
            qx, qy, qz, qw = yaw_to_quat(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = float(qx)
        odom.pose.pose.orientation.y = float(qy)
        odom.pose.pose.orientation.z = float(qz)
        odom.pose.pose.orientation.w = float(qw)

        # Publish twist in base frame (Nav2 expects base-frame twist)
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        # v_base = R(-yaw) * v_odom
        odom.twist.twist.linear.x = float(self.vx * cy + self.vy * sy)
        odom.twist.twist.linear.y = float(-self.vx * sy + self.vy * cy)
        odom.twist.twist.angular.z = float(self.imu_wz)

        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(self.x)
            t.transform.translation.y = float(self.y)
            t.transform.translation.z = 0.0
            t.transform.rotation.x = float(qx)
            t.transform.rotation.y = float(qy)
            t.transform.rotation.z = float(qz)
            t.transform.rotation.w = float(qw)
            self.tf_pub.sendTransform(t)

    # ---------------- Reset ----------------
    def _on_reset(self, request, response):
        """Reset robot pose to (0,0,0) in odom and re-zero velocities.

        Command:
            ros2 service call /sensor_odom/reset std_srvs/srv/Empty "{}"
        """
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0

        if self.imu_yaw_raw is not None:
            self.yaw_offset = self.imu_yaw_raw
            self.yaw = 0.0

        # Reinitialize SLAM reference on next /pose so measurement becomes relative
        self._slam_ref_x = None
        self._slam_ref_y = None
        self._last_slam_meas = None

        self.get_logger().info('sensor_odom: reset -> (x,y,yaw)=(0,0,0), velocities zeroed; SLAM reference will reset on next /pose')
        return response


def main():
    rclpy.init()
    node = SensorOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
