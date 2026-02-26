#!/usr/bin/env python3
"""
IMU-only odometry (no wheel encoders, no cmd_vel).

Goal:
- Fix the "chicken/egg" TF issue by ALWAYS publishing:
    * nav_msgs/Odometry on /odom
    * TF odom -> base_link
  from startup, even if IMU is missing.

Sensors used:
- IMU orientation yaw (rotation) ONLY
- IMU linear_acceleration.x/y integrated to estimate planar velocity (vx, vy)

Notes:
- This will drift (IMU accel integration always drifts), but SLAM can correct globally via map->odom.
- If IMU is not publishing, we still publish odom+TF with zero velocities so Nav2/TF tree stays valid.

Services:
- /open_loop_odom/reset      (std_srvs/Empty) : zero x,y, zero v, yaw = current IMU yaw if available else 0
- /open_loop_odom/reset_hard (std_srvs/Empty) : same + clears accel bias estimates
"""

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty

from tf2_ros import TransformBroadcaster


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


class OpenLoopOdomNode(Node):
    def __init__(self):
        super().__init__('open_loop_odom')

        # Frames / topics
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.imu_topic = self.declare_parameter('imu_topic', '/imu/data').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.publish_tf = bool(self.declare_parameter('publish_tf', True).value)

        # Rate
        self.rate_hz = float(self.declare_parameter('rate_hz', 50.0).value)

        # Require IMU yaw? (IMPORTANT: even if True we still publish TF/odom; we just hold yaw)
        self.require_imu_yaw = bool(self.declare_parameter('require_imu_yaw', True).value)

        # Pose init
        self.x = float(self.declare_parameter('initial_x', 0.0).value)
        self.y = float(self.declare_parameter('initial_y', 0.0).value)
        self.yaw = float(self.declare_parameter('initial_yaw', 0.0).value)

        # ---- IMU accel integration tuning ----
        # Stationary detection: if accel & gyro are small for a bit, treat as stationary and ZUPT.
        self.stationary_accel_thresh = float(self.declare_parameter('stationary_accel_thresh', 0.15).value)  # m/s^2
        self.stationary_gyro_thresh = float(self.declare_parameter('stationary_gyro_thresh', 0.10).value)    # rad/s
        self.stationary_hold_time = float(self.declare_parameter('stationary_hold_time', 0.20).value)        # s

        # Bias learning and damping when stationary
        self.bias_learn_rate = float(self.declare_parameter('bias_learn_rate', 0.6).value)   # 1/s
        self.zupt_vel_damp = float(self.declare_parameter('zupt_vel_damp', 6.0).value)       # 1/s

        # Always-on velocity decay to fight drift
        self.vel_decay = float(self.declare_parameter('vel_decay', 0.15).value)              # 1/s

        # Clamp speed
        self.max_speed = float(self.declare_parameter('max_speed', 2.5).value)               # m/s

        # State
        self._last_time = self.get_clock().now()

        self._imu_yaw: Optional[float] = None
        self._imu_ax = 0.0
        self._imu_ay = 0.0
        self._imu_wz = 0.0

        self._bias_ax = 0.0
        self._bias_ay = 0.0

        self._vx = 0.0  # base frame
        self._vy = 0.0  # base frame

        self._stationary_time = 0.0

        # ROS IO
        self._tf_broadcaster = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        # IMPORTANT: QoS for IMU should match sensor_data style; but subscriber will still connect if publisher is RELIABLE.
        self.create_subscription(Imu, self.imu_topic, self._on_imu, 50)

        self.create_service(Empty, '~/reset', self._srv_reset)
        self.create_service(Empty, '~/reset_hard', self._srv_reset_hard)

        self._timer = self.create_timer(1.0 / self.rate_hz, self._on_timer)

        self.get_logger().info(
            f"open_loop_odom IMU-only: imu={self.imu_topic}, odom={self.odom_topic}, TF {self.odom_frame}->{self.base_frame}, "
            f"rate={self.rate_hz}Hz, require_imu_yaw={self.require_imu_yaw}"
        )

        self._warn_no_imu_every_n = int(max(1, self.rate_hz))  # ~1 Hz warnings
        self._warn_ctr = 0

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        # Some IMUs publish all zeros until calibrated; treat that as "not available"
        if not (q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0):
            self._imu_yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)

        # linear_acceleration is assumed roughly gravity-compensated (BNO055 NDOF usually is),
        # and robot is roughly flat. If not, this will drift badly (still SLAM-correctable).
        self._imu_ax = float(msg.linear_acceleration.x)
        self._imu_ay = float(msg.linear_acceleration.y)

        # gyro z for stationary detection
        self._imu_wz = float(msg.angular_velocity.z)

    def _srv_reset(self, req, resp):
        # Soft reset: keep biases, just zero pose & velocity
        self.x = 0.0
        self.y = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._stationary_time = 0.0
        if self._imu_yaw is not None:
            self.yaw = self._imu_yaw
        else:
            self.yaw = 0.0
        self.get_logger().info("Reset: x=y=0, v=0, yaw=current IMU yaw (or 0 if unavailable).")
        return resp

    def _srv_reset_hard(self, req, resp):
        # Hard reset: also clear accel bias
        self._bias_ax = 0.0
        self._bias_ay = 0.0
        return self._srv_reset(req, resp)

    def _on_timer(self):
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self._last_time = now

        # 1) Yaw: IMU-only (or held if unavailable)
        if self._imu_yaw is not None:
            self.yaw = self._imu_yaw
        else:
            # Hold yaw; but KEEP publishing odom+TF so TF tree is valid.
            self._warn_ctr += 1
            if self.require_imu_yaw and (self._warn_ctr % self._warn_no_imu_every_n == 0):
                self.get_logger().warn("IMU yaw not available; holding yaw (require_imu_yaw=true).")

        # 2) Stationary detection -> ZUPT + bias learning
        accel_mag = math.hypot(self._imu_ax - self._bias_ax, self._imu_ay - self._bias_ay)
        gyro_mag = abs(self._imu_wz)

        is_stationary_now = (accel_mag < self.stationary_accel_thresh) and (gyro_mag < self.stationary_gyro_thresh)
        if is_stationary_now:
            self._stationary_time += dt
        else:
            self._stationary_time = 0.0

        stationary = self._stationary_time >= self.stationary_hold_time

        # Bias learning when stationary: assume true accel ~ 0 in base frame
        if stationary:
            self._bias_ax += self.bias_learn_rate * (self._imu_ax - self._bias_ax) * dt
            self._bias_ay += self.bias_learn_rate * (self._imu_ay - self._bias_ay) * dt

        ax = self._imu_ax - self._bias_ax
        ay = self._imu_ay - self._bias_ay

        # 3) Integrate accel to velocity (base frame)
        self._vx += ax * dt
        self._vy += ay * dt

        # Damping to fight drift
        if self.vel_decay > 0.0:
            decay = max(0.0, 1.0 - self.vel_decay * dt)
            self._vx *= decay
            self._vy *= decay

        # ZUPT damping (stronger) when stationary
        if stationary and self.zupt_vel_damp > 0.0:
            zupt = max(0.0, 1.0 - self.zupt_vel_damp * dt)
            self._vx *= zupt
            self._vy *= zupt

        # Clamp speed
        spd = math.hypot(self._vx, self._vy)
        if self.max_speed > 0.0 and spd > self.max_speed:
            s = self.max_speed / spd
            self._vx *= s
            self._vy *= s

        # 4) Integrate velocity to position in odom frame
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        dx = (self._vx * cy - self._vy * sy) * dt
        dy = (self._vx * sy + self._vy * cy) * dt
        self.x += dx
        self.y += dy

        # 5) Publish /odom
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
        odom.twist.twist.linear.x = self._vx
        odom.twist.twist.linear.y = self._vy
        odom.twist.twist.angular.z = self._imu_wz  # informational only
        self._odom_pub.publish(odom)

        # 6) Publish TF odom->base_link (ALWAYS, fixes chicken/egg)
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
