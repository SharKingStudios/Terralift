#!/usr/bin/env python3
import math
import yaml

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster


def quat_from_rpy(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    # x, y, z, w
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rotate(q, v):
    # v' = q * (v,0) * q_conj
    vx, vy, vz = v
    qv = (vx, vy, vz, 0.0)
    return quat_mul(quat_mul(q, qv), quat_conj(q))[:3]


def tf_to_tq(tf: TransformStamped):
    t = tf.transform.translation
    r = tf.transform.rotation
    return (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)


def tq_inv(t, q):
    qinv = quat_conj(q)
    tinv = quat_rotate(qinv, (-t[0], -t[1], -t[2]))
    return tinv, qinv


def tq_mul(t1, q1, t2, q2):
    # T = T1 * T2
    t2r = quat_rotate(q1, t2)
    t = (t1[0] + t2r[0], t1[1] + t2r[1], t1[2] + t2r[2])
    q = quat_mul(q1, q2)
    return t, q


def yaw_from_quat(q):
    x, y, z, w = q
    # yaw (Z)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class TagLocalizer(Node):
    def __init__(self):
        super().__init__("tag_localizer")

        self.declare_parameter("tag_map_file", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera")

        self.declare_parameter("publish_initialpose", True)
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("publish_tag_pose", True)
        self.declare_parameter("tag_pose_topic", "/tag_pose")

        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("min_publish_period_s", 0.5)

        # Only publish /initialpose if the correction is "worth it"
        self.declare_parameter("min_xy_error_m", 0.20)
        self.declare_parameter("min_yaw_error_rad", 0.25)

        # Covariance for /initialpose (2D-ish)
        self.declare_parameter("cov_xy", 0.05)      # meters (1-sigma)
        self.declare_parameter("cov_yaw", 0.25)     # rad (1-sigma)

        tag_map_file = self.get_parameter("tag_map_file").get_parameter_value().string_value
        if not tag_map_file:
            raise RuntimeError("tag_map_file parameter is empty")

        with open(tag_map_file, "r") as f:
            data = yaml.safe_load(f)

        self.map_frame = data.get("map_frame", self.get_parameter("map_frame").value)
        self.base_frame = self.get_parameter("base_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value

        self.tags = data.get("tags", [])
        if not self.tags:
            raise RuntimeError("No tags found in tag_map_file")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Publish map->tag_* static TFs
        self._publish_static_tag_frames()

        self.pub_init = None
        if self.get_parameter("publish_initialpose").value:
            self.pub_init = self.create_publisher(
                PoseWithCovarianceStamped,
                self.get_parameter("initialpose_topic").value,
                10,
            )

        self.pub_tag_pose = None
        if self.get_parameter("publish_tag_pose").value:
            self.pub_tag_pose = self.create_publisher(
                PoseWithCovarianceStamped,
                self.get_parameter("tag_pose_topic").value,
                10,
            )

        self.last_pub_time = self.get_clock().now()

        rate_hz = float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(1.0 / max(1.0, rate_hz), self.tick)

        self.get_logger().info(f"TagLocalizer active. map={self.map_frame} base={self.base_frame} cam={self.camera_frame}")

    def _publish_static_tag_frames(self):
        msgs = []
        now = self.get_clock().now().to_msg()
        for t in self.tags:
            msg = TransformStamped()
            msg.header.stamp = now
            msg.header.frame_id = self.map_frame
            msg.child_frame_id = t["frame"]

            msg.transform.translation.x = float(t["x"])
            msg.transform.translation.y = float(t["y"])
            msg.transform.translation.z = float(t.get("z", 0.0))

            q = quat_from_rpy(float(t.get("roll", 0.0)), float(t.get("pitch", 0.0)), float(t.get("yaw", 0.0)))
            msg.transform.rotation.x = q[0]
            msg.transform.rotation.y = q[1]
            msg.transform.rotation.z = q[2]
            msg.transform.rotation.w = q[3]
            msgs.append(msg)

        self.static_broadcaster.sendTransform(msgs)
        self.get_logger().info(f"Published {len(msgs)} static tag frames under {self.map_frame}")

    def _make_pose_msg(self, x, y, yaw, stamp_msg):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp_msg
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        q = quat_from_rpy(0.0, 0.0, float(yaw))
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        cov_xy = float(self.get_parameter("cov_xy").value)
        cov_yaw = float(self.get_parameter("cov_yaw").value)

        cov = [0.0] * 36
        cov[0] = cov_xy * cov_xy
        cov[7] = cov_xy * cov_xy
        cov[35] = cov_yaw * cov_yaw
        cov[14] = 999.0  # z
        cov[21] = 999.0  # roll
        cov[28] = 999.0  # pitch
        msg.pose.covariance = cov
        return msg

    def tick(self):
        # Need camera<-base for composition (tf2 will invert your base->camera static TF automatically)
        try:
            tf_cam_base = self.tf_buffer.lookup_transform(self.camera_frame, self.base_frame, Time())
        except Exception:
            return

        t_cb, q_cb = tf_to_tq(tf_cam_base)

        best = None  # (dist, tag_frame, tf_cam_tag)
        for tag in self.tags:
            tag_frame = tag["frame"]
            try:
                tf_cam_tag = self.tf_buffer.lookup_transform(self.camera_frame, tag_frame, Time())
                t_ct, _q_ct = tf_to_tq(tf_cam_tag)
                dist = math.hypot(t_ct[0], t_ct[1])
                if best is None or dist < best[0]:
                    best = (dist, tag_frame, tf_cam_tag)
            except Exception:
                continue

        if best is None:
            return

        _dist, tag_frame, tf_cam_tag = best
        t_ct, q_ct = tf_to_tq(tf_cam_tag)          # cam<-tag
        t_tc, q_tc = tq_inv(t_ct, q_ct)            # tag<-cam

        # map<-tag is static (we broadcast it), so we can lookup map<-tag
        try:
            tf_map_tag = self.tf_buffer.lookup_transform(self.map_frame, tag_frame, Time())
        except Exception:
            return

        t_mt, q_mt = tf_to_tq(tf_map_tag)          # map<-tag

        # map<-cam = (map<-tag) * (tag<-cam)
        t_mc, q_mc = tq_mul(t_mt, q_mt, t_tc, q_tc)

        # map<-base = (map<-cam) * (cam<-base)
        t_mb, q_mb = tq_mul(t_mc, q_mc, t_cb, q_cb)

        yaw = yaw_from_quat(q_mb)
        stamp_msg = self.get_clock().now().to_msg()
        pose_msg = self._make_pose_msg(t_mb[0], t_mb[1], yaw, stamp_msg)

        if self.pub_tag_pose is not None:
            self.pub_tag_pose.publish(pose_msg)

        # Decide if we should publish /initialpose (don’t spam)
        if self.pub_init is None:
            return

        now = self.get_clock().now()
        min_period = float(self.get_parameter("min_publish_period_s").value)
        if (now - self.last_pub_time).nanoseconds < int(min_period * 1e9):
            return

        # Compare against current map<-base if it exists; only correct when off by threshold
        try:
            tf_map_base = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
            t_mb2, q_mb2 = tf_to_tq(tf_map_base)
            dx = t_mb[0] - t_mb2[0]
            dy = t_mb[1] - t_mb2[1]
            dyaw = (yaw - yaw_from_quat(q_mb2) + math.pi) % (2.0 * math.pi) - math.pi

            if math.hypot(dx, dy) < float(self.get_parameter("min_xy_error_m").value) and abs(dyaw) < float(self.get_parameter("min_yaw_error_rad").value):
                return
        except Exception:
            pass  # if no current pose, publish initialpose anyway

        self.pub_init.publish(pose_msg)
        self.last_pub_time = now
        self.get_logger().info(f"/initialpose published from {tag_frame}: x={t_mb[0]:.2f} y={t_mb[1]:.2f} yaw={yaw:.2f}rad")


def main():
    rclpy.init()
    node = TagLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()